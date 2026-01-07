#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
manip2_safety_supervisor.py

Two-stage safety / emergency node for 5-DOF arm.

Design goals (this version):
- DO NOT stage-2 on single comm/read glitches ("There is no status packet!")
- DO stage-2 when servo error text says "not OK"
- DO stage-2 when comm stays bad for long
- DO stage-2 when joint is really out of range (after hold)
- BUT: ignore spiky joint_states
- BUT: allow joints that park exactly on limit (e.g. J3 = -105 deg)
- Order of importance for telemetry: degree -> LOAD -> TEMP -> VOLT
- Takeover order: publish safe pose -> (optional small delay) -> lockout -> call ROS services
- Can be reset without restarting node
"""

import json
import time
import math
import threading
from typing import List, Optional, Dict

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger, TriggerResponse

# try SI first, fallback to raw
try:
    from manip2_msgs.msg import ManipTelemetrySI as TelemetryMsgSI
    HAVE_SI = True
except Exception:
    HAVE_SI = False
    try:
        from manip2_msgs.msg import ManipTelemetry as TelemetryMsgRaw
    except Exception:
        TelemetryMsgRaw = None


def _iso_now() -> str:
    """Generate ISO 8601 timestamp with microsecond precision."""
    return time.strftime("%Y-%m-%dT%H:%M:%S.", time.gmtime()) + f"{int((time.time()%1)*1e6):06d}Z"


class SafetySupervisor(object):
    def __init__(self):
        # ================== PARAMS ==================

        # joint limits (deg)
        self.deg_limits: List[List[float]] = rospy.get_param(
            "~deg_limits",
            [[-90,90],[-30,50],[-106,0],[-77,113],[-103,103]],
        )

        # global margin, per-joint override
        self.joint_warn_margin_deg: float = float(rospy.get_param("~joint_warn_margin_deg", 5.0))
        self.joint_warn_margin_per_joint: List[float] = rospy.get_param("~joint_warn_margin_per_joint", [5,1.0,0.5,5,5])
        # allow resting exactly on limit (for joints that are designed like that)
        self.allow_at_limit: bool = bool(rospy.get_param("~allow_at_limit", True))
        self.limit_epsilon_deg: float = float(rospy.get_param("~limit_epsilon_deg", 2.0))

        # if joint is outside limit, how long before we say CRIT
        self.joint_exceed_hold_s: float = float(rospy.get_param("~joint_exceed_hold_s", 0.8))

        # drop spiky joint samples to avoid comm-noise → fake out-of-range
        self.drop_spike_samples: bool = bool(rospy.get_param("~drop_spike_samples", True))
        self.max_joint_step_deg: float = float(rospy.get_param("~max_joint_step_deg", 15.0))

        # skip joint-limit checks when robot looks offline / comm bad
        self.skip_joint_when_comm_bad: bool = bool(rospy.get_param("~skip_joint_when_comm_bad", True))

        # temp thresholds
        self.p1_temp_warn_c = self._opt_float("~p1_temp_warn_c", 60)
        self.p1_temp_crit_c = self._opt_float("~p1_temp_crit_c", 80)
        self.p2_temp_warn_c = self._opt_float("~p2_temp_warn_c", 45)
        self.p2_temp_crit_c = self._opt_float("~p2_temp_crit_c", 55)

        # VOLTAGE – split P1/P2
        self.p1_volt_warn_low_v  = self._opt_float("~p1_volt_warn_low_v",  None)
        self.p1_volt_crit_low_v  = self._opt_float("~p1_volt_crit_low_v",  None)
        self.p1_volt_warn_high_v = self._opt_float("~p1_volt_warn_high_v", None)
        self.p1_volt_crit_high_v = self._opt_float("~p1_volt_crit_high_v", None)

        self.p2_volt_warn_low_v  = self._opt_float("~p2_volt_warn_low_v",  None)
        self.p2_volt_crit_low_v  = self._opt_float("~p2_volt_crit_low_v",  None)
        self.p2_volt_warn_high_v = self._opt_float("~p2_volt_warn_high_v", None)
        self.p2_volt_crit_high_v = self._opt_float("~p2_volt_crit_high_v", None)

        # load / current (P1, P2)
        self.p1_load_warn_pct = self._opt_float("~p1_load_warn_pct", 80)
        self.p1_load_crit_pct = self._opt_float("~p1_load_crit_pct", 90)

        self.p2_current_warn_a = self._opt_float("~p2_current_warn_a", 0.8)
        self.p2_current_crit_a = self._opt_float("~p2_current_crit_a", 1.4)
        self.p2_load_cont_warn_pct = self._opt_float("~p2_load_cont_warn_pct", 80)
        self.p2_load_cont_crit_pct = self._opt_float("~p2_load_cont_crit_pct", 95)

        # comm softening: how long non-OK diagnostic must persist
        self.comm_error_crit_s: float = float(rospy.get_param("~comm_error_crit_s", 0.5))

        # warning hold
        self.warn_hold_s: float = float(rospy.get_param("~warn_hold_s", 0.1))

        # takeover settings
        self.stage2_services: List[str] = rospy.get_param(
            "~stage2_services",
            ["/manip2/go_initial_and_shutdown", "/manip2/emergency_shutdown"],
        )
        # how long to wait for EACH service call to respond
        self.service_timeout_s: float = float(rospy.get_param("~service_timeout_s", 0.1))
        # how long to let robot move to safe pose BEFORE we lock/kill
        # self.lockout_delay_s: float = float(rospy.get_param("~lockout_delay_s", 0.25))
        # whether to set /manip2/lockout on Stage 2
        self.enable_lockout: bool = bool(rospy.get_param("~enable_lockout", True))
        self.lockout_param_name: str = rospy.get_param("~lockout_param_name", "/manip2/lockout")

        # Emit safety events even when stage doesn't change:
        # - on reason-set change
        # - periodically while reasons exist (reminder)
        self.event_emit_on_reason_change = bool(rospy.get_param("~event_emit_on_reason_change", True))
        self.event_reminder_period_s = float(rospy.get_param("~event_reminder_period_s", 0.5))  # 0 disables periodic reminders

        # topics in
        self.topic_joint_states = rospy.get_param("~topic_joint_states", "/joint_states")
        self.topic_telem_si     = rospy.get_param("~topic_telem_si", "/manip2/telemetry_si")
        self.topic_telem_raw    = rospy.get_param("~topic_telem_raw", "/manip2/telemetry_raw")
        self.topic_errors_diag  = rospy.get_param("~topic_errors", "/manip2/error_status")
        self.topic_errors_text  = rospy.get_param("~topic_errors_text", "/manip2/error_status_text")

        # topics out
        self.topic_event        = rospy.get_param("~topic_event", "/manip2/safety/event")
        self.topic_state        = rospy.get_param("~topic_state", "/manip2/safety/state")
        self.topic_diag         = rospy.get_param("~topic_diag", "/manip2/safety/diagnostics")

        # ================== STATE ==================
        self._lock = threading.RLock()
        self._last_deg: Optional[List[float]] = None
        self._last_js_ts: Optional[float] = None
        self._last_telem = None
        self._last_telem_ts: Optional[float] = None
        self._last_errors: Dict[int, int] = {}
        self._last_error_text: str = ""
        self._last_comm_err_t0: Optional[float] = None
        self._breach_t0: Dict[str, float] = {}
        self._stage: int = 0
        self._last_state_json: str = ""
        self._event_seq = 0
        self._last_reasons_sig = ""
        self._last_reminder_t = 0.0
        self.max_joint_state_age_s = float(rospy.get_param("~max_joint_state_age_s", 1))

        # ================== PUB / SUB ==================
        self.pub_event        = rospy.Publisher(self.topic_event, String, queue_size=20)
        self.pub_state        = rospy.Publisher(self.topic_state, String, queue_size=3, latch=True)
        self.pub_diag         = rospy.Publisher(self.topic_diag, DiagnosticArray, queue_size=10)

        rospy.Subscriber(self.topic_joint_states, JointState, self._on_js, queue_size=50)
        if HAVE_SI:
            rospy.Subscriber(self.topic_telem_si, TelemetryMsgSI, self._on_telem_si, queue_size=10)
        elif 'TelemetryMsgRaw' in globals() and TelemetryMsgRaw is not None:
            rospy.Subscriber(self.topic_telem_raw, TelemetryMsgRaw, self._on_telem_raw, queue_size=10)
        else:
            rospy.logwarn("Safety: no telemetry topic available, only joint+error checks will work.")

        rospy.Subscriber(self.topic_errors_diag, DiagnosticArray, self._on_errors_diag, queue_size=10)
        rospy.Subscriber(self.topic_errors_text, String, self._on_errors_text, queue_size=10)

        # reset service
        rospy.Service("/manip2/safety/reset", Trigger, self._on_reset)

        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", 50.0))

        # main loop
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    # ==========================================================
    # SUBSCRIBERS
    # ==========================================================
    def _on_js(self, msg: JointState):
        if not msg.position:
            return
        new_deg = [math.degrees(p) for p in msg.position]
        now = time.time()
        with self._lock:
            if self.drop_spike_samples and self._last_deg is not None:
                n = min(len(new_deg), len(self._last_deg))
                for i in range(n):
                    if abs(new_deg[i] - self._last_deg[i]) > self.max_joint_step_deg:
                        spiky = rospy.logwarn_throttle(1.0, "Safety: dropped spiky joint_states sample")
                        # IMPORTANT: do NOT update _last_js_ts here
                        return
            self._last_deg = [round(x, 3) for x in new_deg]
            self._last_js_ts = now

    def _on_telem_si(self, msg):
        now = time.time()
        with self._lock:
            self._last_telem = msg
            self._last_telem_ts = now

    def _on_telem_raw(self, msg):
        # minimal conversion so we can still run thresholds
        si = type("Shim", (), {})()
        try:
            si.p1_ids = list(getattr(msg, "ids", []))
            # raw -> V
            si.p1_voltage_v = [0.1 * float(v) for v in getattr(msg, "p1_voltage_raw", [])]
            si.p1_temp_c    = list(getattr(msg, "p1_temp_c", []))

            def _p1pct(lr):
                mag = lr & 0x3FF
                sign = -1.0 if (lr & 0x400) else 1.0
                return sign * (100.0 * (mag / 1023.0))
            si.p1_load_percent = [_p1pct(int(x)) for x in getattr(msg, "p1_load_raw", [])]

            # P2
            raw_cur = int(getattr(msg, "p2_current_raw", 0))
            if raw_cur & 0x8000:
                raw_cur -= 0x10000
            si.p2_current_a = float(raw_cur) * 0.00402832
            si.p2_voltage_v = 0.1 * float(getattr(msg, "p2_voltage_in_raw", 0))
            si.p2_temp_c    = int(getattr(msg, "p2_temp_c", 0))
            si.p2_id        = int(rospy.get_param("~dxl_id_p2", 2))
        except Exception as e:
            rospy.logwarn_throttle(1.0, "Safety: raw→SI conversion failed: %s", str(e))
            return

        now = time.time()
        with self._lock:
            self._last_telem = si
            self._last_telem_ts = now

    def _on_errors_diag(self, msg: DiagnosticArray):
        m = {}
        for st in (msg.status or []):
            try:
                jid = int(st.hardware_id) if st.hardware_id else -1
            except Exception:
                jid = -1
            m[jid] = int(st.level)
        with self._lock:
            self._last_errors = m

    def _on_errors_text(self, msg: String):
        with self._lock:
            self._last_error_text = (msg.data or "").strip()

    # ==========================================================
    # RESET SERVICE
    # ==========================================================
    def _on_reset(self, _req):
        self._stage = 0
        self._breach_t0.clear()
        self._last_comm_err_t0 = None
        self._last_reasons_sig = ""
        self._last_reminder_t = 0.0
        self._last_deg = None
        self._last_js_ts = None
        # unlock robot
        if self.enable_lockout:
            try:
                rospy.set_param(self.lockout_param_name, False)
            except Exception:
                pass
        self._publish_state()
        return TriggerResponse(success=True, message="safety reset: stage=0, lockout=FALSE")

    # ==========================================================
    # MAIN LOOP
    # ==========================================================
    def _loop(self):
        rate = rospy.Rate(self.loop_rate_hz)
        while not rospy.is_shutdown() and not self._stop.is_set():
            try:
                self._tick()
            except Exception as e:
                rospy.logwarn_throttle(1.0, "Safety loop error: %s", str(e))
            rate.sleep()

    def _tick(self):
        now = time.time()
        with self._lock:
            deg     = self._last_deg[:] if self._last_deg else None
            js_ts   = self._last_js_ts
            telem   = self._last_telem
            errors  = dict(self._last_errors)
            err_txt = self._last_error_text

        # is robot online?
        online = self._robot_online(now)

        reasons = []

        # ========== 1) JOINT CHECKS ==========
        js_fresh = (js_ts is not None) and ((now - js_ts) < self.max_joint_state_age_s)
        if deg is not None and js_fresh and self.deg_limits and (online or not self.skip_joint_when_comm_bad):
            n = min(len(deg), len(self.deg_limits))
            for i in range(n):
                lo, hi = self.deg_limits[i]
                q = deg[i]

                # allow exactly at limit
                if self.allow_at_limit:
                    if abs(q - lo) <= self.limit_epsilon_deg:
                        q = lo
                    elif abs(q - hi) <= self.limit_epsilon_deg:
                        q = hi
                at_limit = self.allow_at_limit and (q == lo or q == hi)

                # per-joint margin
                margin = self.joint_warn_margin_deg
                if self.joint_warn_margin_per_joint and i < len(self.joint_warn_margin_per_joint):
                    margin = float(self.joint_warn_margin_per_joint[i])

                # critical exceed (with hold)
                if q < lo or q > hi:
                    key = f"joint{i+1}:exceed"
                    t0 = self._breach_t0.get(key, now)
                    self._breach_t0[key] = t0
                    if (now - t0) >= self.joint_exceed_hold_s:
                        reasons.append(("crit", "joint_limit_exceeded",
                                        {"joint": i+1, "q_deg": q, "limit": [lo, hi]}))
                else:
                    self._breach_t0.pop(f"joint{i+1}:exceed", None)

                # warning near limit
                if (not at_limit) and (((q - lo) <= margin) or ((hi - q) <= margin)):
                    key = f"joint{i+1}:margin"
                    t0 = self._breach_t0.get(key, now)
                    self._breach_t0[key] = t0
                    if (now - t0) >= self.warn_hold_s:
                        reasons.append(("warn", "joint_near_limit",
                                        {"joint": i+1, "q_deg": q, "limit": [lo, hi], "margin_deg": margin}))
                else:
                    self._breach_t0.pop(f"joint{i+1}:margin", None)

        # ========== 2) TELEMETRY (LOAD → TEMP → VOLT) ==========
        if telem is not None:
            # P1 array
            try:
                p1_ids  = list(getattr(telem, "p1_ids", []))
                p1_v    = list(getattr(telem, "p1_voltage_v", []))
                p1_t    = list(getattr(telem, "p1_temp_c", []))
                p1_load = list(getattr(telem, "p1_load_percent", []))
            except Exception:
                p1_ids, p1_v, p1_t, p1_load = [], [], [], []

            for idx, sid in enumerate(p1_ids):
                v = p1_v[idx] if idx < len(p1_v) else None
                t = p1_t[idx] if idx < len(p1_t) else None
                ld = p1_load[idx] if idx < len(p1_load) else None

                # 1) LOAD
                if ld is not None:
                    ald = abs(ld)
                    if self.p1_load_crit_pct is not None and ald >= self.p1_load_crit_pct:
                        reasons.append(("crit", "p1_load_crit", {"id": sid, "load_pct": ld}))
                    elif self.p1_load_warn_pct is not None and ald >= self.p1_load_warn_pct:
                        reasons.append(("warn", "p1_load_warn", {"id": sid, "load_pct": ld}))

                # 2) TEMP
                if t is not None:
                    if self.p1_temp_crit_c is not None and t >= self.p1_temp_crit_c:
                        reasons.append(("crit", "p1_temp_crit", {"id": sid, "temp_c": t}))
                    elif self.p1_temp_warn_c is not None and t >= self.p1_temp_warn_c:
                        reasons.append(("warn", "p1_temp_warn", {"id": sid, "temp_c": t}))

                # 3) VOLT
                if v is not None:
                    if self.p1_volt_crit_low_v is not None and v <= self.p1_volt_crit_low_v:
                        reasons.append(("crit", "p1_volt_low_crit", {"id": sid, "volt_v": v}))
                    elif self.p1_volt_warn_low_v is not None and v <= self.p1_volt_warn_low_v:
                        reasons.append(("warn", "p1_volt_low_warn", {"id": sid, "volt_v": v}))
                    if self.p1_volt_crit_high_v is not None and v >= self.p1_volt_crit_high_v:
                        reasons.append(("crit", "p1_volt_high_crit", {"id": sid, "volt_v": v}))
                    elif self.p1_volt_warn_high_v is not None and v >= self.p1_volt_warn_high_v:
                        reasons.append(("warn", "p1_volt_high_warn", {"id": sid, "volt_v": v}))

            # P2 single
            p2_t  = getattr(telem, "p2_temp_c", None)
            p2_v  = getattr(telem, "p2_voltage_v", None)
            p2_i  = getattr(telem, "p2_current_a", None)
            p2_id = getattr(telem, "p2_id", 2)
            p2_cont = getattr(telem, "p2_load_cont_pct", None)

            # 1) LOAD / CURRENT
            if p2_i is not None:
                if self.p2_current_crit_a is not None and abs(p2_i) >= self.p2_current_crit_a:
                    reasons.append(("crit", "p2_current_crit", {"id": p2_id, "current_a": p2_i}))
                elif self.p2_current_warn_a is not None and abs(p2_i) >= self.p2_current_warn_a:
                    reasons.append(("warn", "p2_current_warn", {"id": p2_id, "current_a": p2_i}))

            if p2_cont is not None:
                if self.p2_load_cont_crit_pct is not None and abs(p2_cont) >= self.p2_load_cont_crit_pct:
                    reasons.append(("crit", "p2_load_cont_crit", {"id": p2_id, "load_cont_pct": p2_cont}))
                elif self.p2_load_cont_warn_pct is not None and abs(p2_cont) >= self.p2_load_cont_warn_pct:
                    reasons.append(("warn", "p2_load_cont_warn", {"id": p2_id, "load_cont_pct": p2_cont}))

            # 2) TEMP
            if p2_t is not None:
                if self.p2_temp_crit_c is not None and p2_t >= self.p2_temp_crit_c:
                    reasons.append(("crit", "p2_temp_crit", {"id": p2_id, "temp_c": p2_t}))
                elif self.p2_temp_warn_c is not None and p2_t >= self.p2_temp_warn_c:
                    reasons.append(("warn", "p2_temp_warn", {"id": p2_id, "temp_c": p2_t}))

            # 3) VOLT
            if p2_v is not None:
                if self.p2_volt_crit_low_v is not None and p2_v <= self.p2_volt_crit_low_v:
                    reasons.append(("crit", "p2_volt_low_crit", {"id": p2_id, "volt_v": p2_v}))
                elif self.p2_volt_warn_low_v is not None and p2_v <= self.p2_volt_warn_low_v:
                    reasons.append(("warn", "p2_volt_low_warn", {"id": p2_id, "volt_v": p2_v}))
                if self.p2_volt_crit_high_v is not None and p2_v >= self.p2_volt_crit_high_v:
                    reasons.append(("crit", "p2_volt_high_crit", {"id": p2_id, "volt_v": p2_v}))
                elif self.p2_volt_warn_high_v is not None and p2_v >= self.p2_volt_warn_high_v:
                    reasons.append(("warn", "p2_volt_high_warn", {"id": p2_id, "volt_v": p2_v}))

        # ========== 3) DIAGNOSTICS (soft) ==========
        persistent_comm_crit = False
        if errors:
            for sid, lvl in errors.items():
                if lvl != DiagnosticStatus.OK:
                    if self._last_comm_err_t0 is None:
                        self._last_comm_err_t0 = now
                    reasons.append(("warn", "comm_glitch", {"id": sid, "level": int(lvl)}))
                    if (now - self._last_comm_err_t0) >= self.comm_error_crit_s:
                        persistent_comm_crit = True
                else:
                    self._last_comm_err_t0 = None
        if persistent_comm_crit:
            reasons.append(("crit", "comm_glitch_persistent", {}))

        # ========== 4) HUMAN ERROR TEXT (hard trigger) ==========
        if err_txt:
            if not self._all_ids_ok(err_txt):
                reasons.append(("crit", "servo_error_text", {"text": err_txt}))

        # ========== STAGE DECISION ==========
        has_crit = any(r[0] == "crit" for r in reasons)
        has_warn = any(r[0] == "warn" for r in reasons)
        new_stage = 2 if has_crit else (1 if has_warn else 0)

        # publish diagnostics
        self.pub_diag.publish(self._reasons_to_diag(reasons))

        # human readable
        human_list = self._reasons_to_human(new_stage, reasons)

        reasons_sig = self._reasons_signature(reasons) if reasons else ""
        stage_changed = (new_stage != self._stage)

        # handle stage change
        if stage_changed:
            # always publish event/state immediately
            self._emit_event(new_stage, "stage_change", reasons, human_list)
            self._publish_state(new_stage)
            self._stage = new_stage
            # IMPORTANT: update tracking so we don't immediately "reasons_update" after stage_change
            self._last_reasons_sig = reasons_sig
            self._last_reminder_t = now

            if new_stage == 2:
                # if robot looks offline (just reconnected), don't try to move right now
                if not online:
                    if self.enable_lockout:
                        try:
                            rospy.set_param(self.lockout_param_name, True)
                        except Exception:
                            pass
                    self._emit_event(2, "takeover_skipped_offline", reasons,
                                     ["Robot offline, takeover deferred"])
                else:
                    self._do_takeover(reasons)
            
        else:
            # Stage didn't change, but we still want events while reasons exist.
            if reasons and new_stage == 1:
                # A) Emit immediately if the *reason set* changed (not the numeric values)
                if self.event_emit_on_reason_change and (reasons_sig != self._last_reasons_sig):
                    self._emit_event(new_stage, "reasons_update", reasons, human_list)
                    self._last_reasons_sig = reasons_sig
                    self._last_reminder_t = now

                # B) Otherwise emit periodic reminders
                elif self.event_reminder_period_s > 0 and (now - self._last_reminder_t) >= self.event_reminder_period_s:
                    self._emit_event(new_stage, "reminder", reasons, human_list)
                    self._last_reminder_t = now

            elif not reasons:
                # No reasons: clear signature so next warning emits properly
                self._last_reasons_sig = ""


        # periodic state refresh
        if not self._last_state_json or (time.time() % 1.0) < 0.1:
            self._publish_state(self._stage)

    # ==========================================================
    # TAKEOVER LOGIC
    # ==========================================================
    def _do_takeover(self, reasons):
        # 3) set lockout so user/agent can't override
        if self.enable_lockout:
            try:
                rospy.set_param(self.lockout_param_name, True)
            except Exception:
                pass

        # 4) call services in order
        for srv in (self.stage2_services or []):
            try:
                rospy.wait_for_service(srv, timeout=self.service_timeout_s)
            except Exception:
                continue
            try:
                call = rospy.ServiceProxy(srv, Trigger)
                resp = call()
                ok = bool(getattr(resp, "success", False))
                msg = getattr(resp, "message", "")
                self._emit_event(2, "takeover_service_call", reasons, [f"called {srv}: {ok} {msg}"])
                if ok:
                    break
            except Exception as e:
                self._emit_event(2, "takeover_service_call", reasons,
                                 [f"call {srv} failed: {str(e)}"])
                continue

    # ==========================================================
    # HELPERS
    # ==========================================================
    def _robot_online(self, now=None, max_age=1.0) -> bool:
        if now is None:
            now = time.time()
        if self._last_js_ts is not None and (now - self._last_js_ts) < max_age:
            return True
        if self._last_telem_ts is not None and (now - self._last_telem_ts) < max_age:
            return True
        return False

    def _opt_float(self, name: str, default: Optional[float]) -> Optional[float]:
        val = rospy.get_param(name, None if default is None else float(default))
        if val is None or (isinstance(val, str) and not val.strip()):
            return None
        try:
            return float(val)
        except Exception:
            return None

    def _all_ids_ok(self, txt: str) -> bool:
        parts = [p.strip() for p in txt.split(";") if p.strip()]
        if not parts:
            return True
        for p in parts:
            if not p.endswith("OK") and not p.endswith("OK."):
                return False
        return True

    def _reasons_to_diag(self, reasons):
        arr = DiagnosticArray()
        arr.header = Header(stamp=rospy.Time.now())
        for sev, code, detail in reasons:
            st = DiagnosticStatus()
            st.level = DiagnosticStatus.ERROR if sev == "crit" else DiagnosticStatus.WARN
            st.name = f"Safety/{code}"
            st.message = sev.upper()
            st.values = [KeyValue(key=k, value=str(v)) for k, v in (detail or {}).items()]
            arr.status.append(st)
        return arr

    def _reasons_to_human(self, stage: int, reasons):
        msgs = []
        if stage == 1:
            msgs.append("SAFETY: Stage 1 (warning).")
        elif stage == 2:
            msgs.append("SAFETY: Stage 2 (takeover).")
        for sev, code, detail in reasons:
            sid = detail.get("id") if isinstance(detail, dict) else None
            jid = detail.get("joint") if isinstance(detail, dict) else None
            if code == "joint_limit_exceeded":
                msgs.append(f"CRIT: joint {jid} out of range ({detail.get('q_deg')} deg).")
            elif code == "joint_near_limit":
                msgs.append(f"WARN: joint {jid} near limit ({detail.get('q_deg')} deg).")
            elif code == "p1_load_warn":
                msgs.append(f"WARN: P1 ID {sid} high load ({detail.get('load_pct')}%).")
            elif code == "p1_load_crit":
                msgs.append(f"CRIT: P1 ID {sid} overload ({detail.get('load_pct')}%).")
            elif code == "p1_temp_warn":
                msgs.append(f"WARN: P1 ID {sid} temp {detail.get('temp_c')}°C.")
            elif code == "p1_temp_crit":
                msgs.append(f"CRIT: P1 ID {sid} over temp {detail.get('temp_c')}°C.")
            elif code == "p2_current_warn":
                msgs.append(f"WARN: P2 ID {sid} current {detail.get('current_a')}A.")
            elif code == "p2_current_crit":
                msgs.append(f"CRIT: P2 ID {sid} current {detail.get('current_a')}A.")
            elif code == "p2_temp_warn":
                msgs.append(f"WARN: P2 ID {sid} temp {detail.get('temp_c')}°C.")
            elif code == "p2_temp_crit":
                msgs.append(f"CRIT: P2 ID {sid} over temp {detail.get('temp_c')}°C.")
            elif code == "comm_glitch":
                msgs.append(f"WARN: comm glitch on ID {sid}.")
            elif code == "comm_glitch_persistent":
                msgs.append("CRIT: comm error persistent.")
            elif code == "servo_error_text":
                msgs.append(f"CRIT: servo error: {detail.get('text')}")
            else:
                msgs.append(f"{sev.upper()}: {code} {detail}")
        return msgs

    def _emit_event(self, stage: int, event: str, reasons, human_list):
        payload = {
            "schema": "manip2.safety.v1",
            "ts_iso": _iso_now(),
            "ts_unix_ns": time.time_ns(),
            "seq": self._event_seq,
            "stage": int(stage),
            "event": event,
            "reasons": reasons,
            "human": human_list,
        }
        self._event_seq += 1
        self.pub_event.publish(String(data=json.dumps(payload)))

    def _reasons_signature(self, reasons) -> str:
        # Signature ignores noisy numeric values; focuses on "what kind of reason" and "which joint/id"
        items = []
        for sev, code, detail in (reasons or []):
            ident = {}
            if isinstance(detail, dict):
                if "joint" in detail: ident["joint"] = int(detail["joint"])
                if "id" in detail: ident["id"] = int(detail["id"])
            items.append([str(sev), str(code), ident])
        items.sort(key=lambda x: (x[0], x[1], json.dumps(x[2], sort_keys=True)))
        return json.dumps(items, sort_keys=True)

    def _publish_state(self, stage=None):
        if stage is None:
            stage = self._stage
        payload = {
            "schema": "manip2.safety.v1",
            "ts_iso": _iso_now(),
            "stage": int(stage),
        }
        s = json.dumps(payload)
        if s != self._last_state_json:
            self._last_state_json = s
            self.pub_state.publish(String(data=s))

    def shutdown(self):
        self._stop.set()


def main():
    rospy.init_node("manip2_safety_supervisor", anonymous=False)
    node = SafetySupervisor()
    rospy.loginfo("manip2_safety_supervisor started (anti-spike, soft-comm, load-first).")
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()

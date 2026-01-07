#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
manip2_web_bridge_node.py  (revised)

- Robust parsing for ~dxl_ids (accepts list, YAML/JSON string "[1,2,3,4,5]", or "1,2,3,4,5")
- Crops/handles error_status_raw length mismatches without spamming
- Builds joints[] MQTT payload as before; computes P2 load_stl/load_cont from current
"""

import json, math, time, threading, ast, datetime
from typing import List, Tuple

import rospy
from std_msgs.msg import UInt8MultiArray, String, Header
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String

# Custom messages (define in your manip2_msgs package)
from manip2_msgs.msg import ManipTelemetry, ManipTelemetrySI

# Optional MQTT
try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

import tf2_ros
import datetime

try:
    from tf.transformations import euler_from_quaternion
except Exception:
    euler_from_quaternion = None


# ---------------- Helpers ----------------

def _now_ns() -> int:
    return int(time.time() * 1e9)

def _iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S.", time.gmtime()) + f"{int((time.time()%1)*1e6):06d}Z"

def p2_raw_to_amps(current_raw: int) -> float:
    """
    PRO-42 (M-42-Pro) Present Current raw -> Amps.
    A[mA] = raw * 8250 / 2048  => A = raw * 0.00402832
    """
    return float(current_raw) * 0.00402832

def p2_load_percentages(cur_a: float, stall_a: float, cont_a: float):
    """Return (load_stl, load_cont) signed percentages ([-100,100])."""
    sgn = 1.0 if cur_a >= 0 else -1.0
    def pct(ref):
        if ref <= 0: return 0.0
        return sgn * min(100.0, (abs(cur_a) / ref) * 100.0)
    return round(pct(stall_a), 2), round(pct(cont_a), 2)


class Manip2WebBridge:
    # ---- Robust list parser for dxl_ids ----
    @staticmethod
    def _parse_id_list(val, default):
        """Accept list OR strings like '[1,2,3,4,5]' OR '1,2,3,4,5'.
           If unresolved substitution (e.g. '$(arg dxl_ids)'), fallback to default."""
        if isinstance(val, list):
            try:   return [int(x) for x in val]
            except: pass
        if isinstance(val, str):
            s = val.strip()
            if s.startswith("$(") and s.endswith(")"):
                rospy.logwarn("~dxl_ids looks unresolved (%s); using default %s", s, default)
                return list(default)
            # try literal eval (handles JSON/YAML-like lists)
            try:
                parsed = ast.literal_eval(s)
                if isinstance(parsed, list):
                    return [int(x) for x in parsed]
            except Exception:
                pass
            # try comma-separated
            try:
                return [int(x) for x in s.split(",") if x.strip() != ""]
            except Exception:
                pass
        rospy.logwarn("~dxl_ids malformed (%r); using default %s", val, default)
        return list(default)

    def __init__(self):
        # --- Parameters ---
        raw_ids = rospy.get_param("~dxl_ids", [1, 2, 3, 4, 5])
        self.dxl_ids: List[int] = self._parse_id_list(raw_ids, [1, 2, 3, 4, 5])
        self.dxl_id_p2: int = int(rospy.get_param("~dxl_id_p2", 2))

        # Topic names
        self.topic_js = rospy.get_param("~topic_joint_states", "/joint_states")
        self.topic_telem_raw = rospy.get_param("~topic_telem_raw", "/manip2/telemetry_raw")
        self.topic_telem_si = rospy.get_param("~topic_telem_si", "/manip2/telemetry_si")
        self.topic_err_raw = rospy.get_param("~topic_error_raw", "/manip2/error_status_raw")
        self.topic_err_text = rospy.get_param("~topic_error_text", "/manip2/error_status_text")
        self.topic_err_diag = rospy.get_param("~topic_error_diag", "/manip2/error_status")
        self.topic_safety_event = rospy.get_param("~topic_safety_event", "/manip2/safety/event")

        # Optional: downsample joint state mirroring to MQTT / web
        self.js_downsample_hz = float(rospy.get_param("~js_downsample_hz", 10.0))

        # MQTT (optional)
        self.mqtt_enable = bool(rospy.get_param("~mqtt_enable", True))
        self.mqtt_host = rospy.get_param("~mqtt_host", "8.215.67.35")
        self.mqtt_port = int(rospy.get_param("~mqtt_port", 1883))
        self.mqtt_base = rospy.get_param("~mqtt_base", "robot/arm")
        self.mqtt_client = None
        self.mqtt_username = rospy.get_param("~mqtt_username", "aliefpal")
        self.mqtt_password = rospy.get_param("~mqtt_password", "Unpad2020")

        # Reference currents for M-42-Pro (tune if your datasheet label differs)
        self.p2_stall_current_a = float(rospy.get_param("~p2_stall_current_a", 2.0))
        self.p2_cont_current_a  = float(rospy.get_param("~p2_cont_current_a", 0.6))

        # --- Internal state ---
        self._lock = threading.RLock()
        self._last_js = None            # type: JointState
        self._last_js_pub = 0.0
        self._last_err_text = ""
        self._last_telem = None         # ManipTelemetrySI (engineering)
        self._last_errmap = {}          # dict[int -> (level:int, message:str)]

        # --- Publishers ---
        self.pub_telem_si = rospy.Publisher(self.topic_telem_si, ManipTelemetrySI, queue_size=10)
        self.pub_err_diag = rospy.Publisher(self.topic_err_diag, DiagnosticArray, queue_size=10)

        # --- Subscribers ---
        rospy.Subscriber(self.topic_js, JointState, self._on_joint_state, queue_size=20)
        rospy.Subscriber(self.topic_telem_raw, ManipTelemetry, self._on_telem_raw, queue_size=10)
        rospy.Subscriber(self.topic_err_raw, UInt8MultiArray, self._on_error_raw, queue_size=10)
        rospy.Subscriber(self.topic_err_text, String, self._on_error_text, queue_size=10)
        rospy.Subscriber(self.topic_safety_event, String, self._on_safety_event, queue_size=10)
        rospy.Subscriber("/manip2/eval_result", String, self._on_eval_result, queue_size=10)

        # --- MQTT optional ---
        if self.mqtt_enable and mqtt is not None:
            try:
                # Keep default callback API; warning is harmless. (V2 requires different callbacks.)
                self.mqtt_client = mqtt.Client(client_id="manip2_web_bridge")
                # NEW: set username/password if provided
                if self.mqtt_username:
                    self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
                self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
                self.mqtt_client.loop_start()
                rospy.loginfo("MQTT connected to %s:%d", self.mqtt_host, self.mqtt_port)
            except Exception as e:
                rospy.logwarn("MQTT disabled (connect failed): %s", str(e))
                self.mqtt_client = None
        elif self.mqtt_enable and mqtt is None:
            rospy.logwarn("paho-mqtt not available, MQTT disabled.")

        # --- EE Pose publishing (TF -> MQTT) ---
        self.ee_enable = bool(rospy.get_param("~ee_enable", True))
        self.ee_parent_frame = rospy.get_param("~ee_parent_frame", "base_link")
        self.ee_child_frame  = rospy.get_param("~ee_child_frame", "end_effector")
        self.ee_pub_hz = float(rospy.get_param("~ee_pub_hz", 30.0))
        self.ee_timeout_s = float(rospy.get_param("~ee_timeout_s", 0.05))

        # MQTT topic suffix under mqtt_base
        # Full topic becomes: f"{mqtt_base}/{mqtt_topic_ee_pose}"
        self.mqtt_topic_ee_pose = rospy.get_param("~mqtt_topic_ee_pose", "ee/state")
        self.ee_retain = bool(rospy.get_param("~ee_retain", False))

        self._ee_seq = 0
        self._err_seq = 0  # monotonic sequence for MQTT errors stream

        # TF listener (needed to read base_link -> end_effector)
        self._tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf)

        # Timer to publish EE pose at fixed rate
        self._ee_timer = None
        if self.ee_enable and self.ee_pub_hz > 0:
            self._ee_timer = rospy.Timer(
                rospy.Duration(1.0 / self.ee_pub_hz),
                self._on_ee_timer,
                oneshot=False
            )

        # Timer BARU untuk membatasi frekuensi publish Joint State ke MQTT (Throttling)
        # Ini mencegah packet loss karena burst data
        if self.mqtt_enable and self.js_downsample_hz > 0:
            rospy.Timer(rospy.Duration(1.0 / self.js_downsample_hz), self._on_js_mqtt_timer)

        rospy.loginfo("manip2_web_bridge_node initialized.")

    # ---------------- Unit conversion helpers ----------------

    @staticmethod
    def _p1_voltage_v(v_raw: int) -> float:
        # Present Voltage LSB = 0.1 V (Protocol 1 series like RX/MX)
        return 0.1 * float(v_raw)

    @staticmethod
    def _p1_load_percent(load_raw: int) -> float:
        # Bits 0..9 magnitude, bit 10 = direction (0=CCW +, 1=CW -)
        mag = load_raw & 0x3FF
        sign = -1.0 if (load_raw & 0x400) else 1.0
        return round(sign * (100.0 * (mag / 1023.0)), 2)

    @staticmethod
    def _p2_current_a(cur_raw_signed: int) -> float:
        return p2_raw_to_amps(cur_raw_signed)

    @staticmethod
    def _p2_voltage_v(vin_raw: int) -> float:
        # Present Input Voltage LSB = 0.1 V
        return 0.1 * float(vin_raw)

    # ---------------- Error decoding helpers ----------------

    _P1_BITS = [
        (0, "Input Voltage"),
        (1, "Angle Limit"),
        (2, "Overheating"),
        (3, "Range"),
        (4, "Checksum"),
        (5, "Overload"),
        (6, "Instruction"),
    ]

    _P2_HW_BITS = [
        (0, "Input Voltage"),
        (1, "Motor Hall Sensor"),
        (2, "Overheating"),
        (3, "Motor Encoder"),
        (4, "Electrical Shock"),
        (5, "Overload"),
        # 6,7 unused
    ]

    @classmethod
    def _labels_from_bits(cls, value: int, table: List[Tuple[int, str]]) -> List[str]:
        return [name for (bit, name) in table if ((value >> bit) & 0x1)]

    # ---------------- ROS Callbacks ----------------

    def _on_joint_state(self, msg: JointState):
        with self._lock:
            self._last_js = msg

    def _on_js_mqtt_timer(self, event):
        """Publish joint state ke MQTT dengan frekuensi stabil (Timer)."""
        if self.mqtt_client is None: return

        # Ambil copy data terbaru secara thread-safe
        js_copy = None
        with self._lock:
            if self._last_js:
                js_copy = self._last_js

        if js_copy is None: return

        try:
            joint_deg = [math.degrees(p) for p in (js_copy.position or [])]

            # Gunakan _now_ns() (Waktu Server Saat Ini) agar latency terhitung benar
            payload = {
                "ts_unix_ns": _now_ns(),
                "ts_iso": _iso_now(),
                "seq": js_copy.header.seq if js_copy.header else 0,
                "joint_names": js_copy.name,
                "joint_deg": joint_deg,
            }
            self.mqtt_client.publish(f"{self.mqtt_base}/joints/state",
                                     json.dumps(payload), qos=1, retain=False)
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"MQTT joint state pub failed: {e}")

    def _on_telem_raw(self, msg: ManipTelemetry):
        # Build engineering-units message for ROS
        out = ManipTelemetrySI()
        # out.header = Header()
        # out.header.stamp = rospy.Time.now()
        out.header = msg.header  # propagate stamp + seq from C++ telem

        # IDs for P1 (the subset, in-order with raw arrays)
        out.p1_ids = list(getattr(msg, "ids", []))

        # Convert P1 arrays
        out.p1_voltage_v   = [self._p1_voltage_v(v)   for v in getattr(msg, "p1_voltage_raw", [])]
        out.p1_load_percent= [self._p1_load_percent(l) for l in getattr(msg, "p1_load_raw", [])]
        out.p1_temp_c      = list(getattr(msg, "p1_temp_c", []))

        # P2 engineering values (signed current)
        cur = int(getattr(msg, "p2_current_raw", 0))
<<<<<<< HEAD
        # if cur & 0x8000:
        #    cur = cur - 0x10000
=======
>>>>>>> 80cc0e98fc7742ae3b1e3be2bed94b23fb1e62f3
        out.p2_id        = int(self.dxl_id_p2)
        out.p2_current_a = self._p2_current_a(cur)
        out.p2_voltage_v = self._p2_voltage_v(int(getattr(msg, "p2_voltage_in_raw", 0)))
        out.p2_temp_c    = int(getattr(msg, "p2_temp_c", 0))

        # Publish SI message
        self.pub_telem_si.publish(out)

        # Cache for payload build
        with self._lock:
            self._last_telem = out

        # MQTT snapshot (joints[] payload)
        if self.mqtt_client is not None:
            try:
                payload = self._build_joints_payload()
                self.mqtt_client.publish(f"{self.mqtt_base}/telemetry",
                                         json.dumps(payload), qos=1, retain=False)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "MQTT telemetry pub failed: %s", str(e))

    def _on_error_text(self, msg: String):
        with self._lock:
            self._last_err_text = msg.data or ""

    def _on_error_raw(self, msg: UInt8MultiArray):
        # msg.data aligned to dxl_ids; for P2 slot, driver places latched HW Error Status byte
        raw = list(msg.data or [])

        # Crop to the shorter length to avoid repeated warnings; still log once per second.
        if len(raw) != len(self.dxl_ids):
            k = min(len(raw), len(self.dxl_ids))
            rospy.logwarn_throttle(1.0,
                "error_status_raw length %d != dxl_ids length %d; cropping to %d",
                len(raw), len(self.dxl_ids), k)
        else:
            k = len(raw)

        arr = DiagnosticArray()
        arr.header = Header()
        arr.header.stamp = rospy.Time.now()

        errmap = {}  # id -> (level, message)

        # Build one DiagnosticStatus per available entry
        for i in range(k):
            sid = self.dxl_ids[i]
            try:
                sid_int = int(sid)
            except Exception:
                rospy.logwarn_throttle(1.0, "Bad dxl_id '%r' at index %d; skipping", sid, i)
                continue

            st = DiagnosticStatus()
            st.name = f"DYNAMIXEL/{sid_int}"
            st.hardware_id = str(sid_int)
            st.level = DiagnosticStatus.OK
            st.message = "OK"

            eb = int(raw[i]) & 0xFF
            fields = []

            if sid_int == self.dxl_id_p2:
                labels = self._labels_from_bits(eb, self._P2_HW_BITS)
                if labels:
                    st.level = DiagnosticStatus.ERROR
                    st.message = " | ".join(labels)
                fields.append(KeyValue(key="hardware_error_status_hex", value=f"0x{eb:02X}"))
                for lb in labels:
                    fields.append(KeyValue(key="fault", value=lb))
            else:
                labels = self._labels_from_bits(eb, self._P1_BITS)
                if labels:
                    st.level = DiagnosticStatus.WARN
                    st.message = " | ".join(labels)
                fields.append(KeyValue(key="p1_error_byte_hex", value=f"0x{eb:02X}"))
                for lb in labels:
                    fields.append(KeyValue(key="fault", value=lb))

            txt = ""
            with self._lock:
                txt = self._last_err_text
            if txt:
                fields.append(KeyValue(key="driver_summary", value=txt))

            st.values = fields
            arr.status.append(st)
            errmap[sid_int] = (int(st.level), st.message)

        # Publish diagnostics & cache
        self.pub_err_diag.publish(arr)
        with self._lock:
            self._last_errmap = errmap

        # Optional MQTT error summary
        if self.mqtt_client is not None and arr.status:
            try:
                summary = [{"id": int(st.hardware_id), "level": int(st.level), "message": st.message}
                           for st in arr.status]
                payload = {"ts_unix_ns": _now_ns(), "ts_iso": _iso_now(), "seq": self._err_seq, "errors": summary}
                self.mqtt_client.publish(f"{self.mqtt_base}/errors",
                                         json.dumps(payload), qos=1, retain=False)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "MQTT error pub failed: %s", str(e))

    
    def _on_safety_event(self, msg: String):
        if self.mqtt_client is None: return
        try:
            self.mqtt_client.publish(f"{self.mqtt_base}/safety/events", msg.data, qos=1, retain=False)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "MQTT safety event pub failed: %s", str(e))


    def _on_eval_result(self, msg: String):
        """Forward MATLAB evaluation results to MQTT"""
        if self.mqtt_client is None: return
        try:
            # We assume msg.data is already a JSON string from MATLAB
            # We wrap it in a standard envelope
            payload = {
                "type": "eval_result",
                "ts_unix_ns": _now_ns(),
                "ts_iso": _iso_now(),
                "data": json.loads(msg.data) # Parse to ensure valid JSON, or just pass string
            }
            self.mqtt_client.publish(f"{self.mqtt_base}/eval_result", 
                                    json.dumps(payload), qos=1, retain=False)
        except Exception as e:
            rospy.logwarn(f"Failed to forward eval result: {e}")

    def _on_ee_timer(self, _event):
        # Only publish if MQTT is active
        if self.mqtt_client is None:
            return
        if not self.ee_enable:
            return

        try:
            tr = self._tf_buf.lookup_transform(
                self.ee_parent_frame,
                self.ee_child_frame,
                rospy.Time(0),
                rospy.Duration(self.ee_timeout_s)
            )

            # Prefer TF stamp if valid; otherwise use now
            # stamp = tr.header.stamp
            # secs = stamp.to_sec() if stamp and stamp.to_sec() > 0 else time.time()

            # Gunakan waktu SEKARANG untuk timestamp pengiriman
            ts_unix_ns = _now_ns()
            ts_iso = _iso_now()

            # Hitung umur data TF hanya untuk info (debugging)
            # tf_age = (rospy.Time.now() - tr.header.stamp).to_sec()

            q = tr.transform.rotation
            t = tr.transform.translation

            payload = {
                "schema": "manip2.ee_state.v1",
                "ts_unix_ns": ts_unix_ns,
                "ts_iso": ts_iso,
                "seq": self._ee_seq,
                # "tf_age_s": round(tf_age, 4), # Opsional: Info seberapa tua data TF
                "pos_m": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
                "quat":  {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
            }

            if euler_from_quaternion is not None:
                roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                payload["rpy_deg"] = {
                    "roll":  float(roll  * 180.0 / math.pi),
                    "pitch": float(pitch * 180.0 / math.pi),
                    "yaw":   float(yaw   * 180.0 / math.pi),
                }

            self._ee_seq += 1

            topic = f"{self.mqtt_base}/{self.mqtt_topic_ee_pose}"
            self.mqtt_client.publish(topic, json.dumps(payload), qos=1, retain=self.ee_retain)

        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "EE TF lookup/publish failed (%s -> %s): %s",
                self.ee_parent_frame, self.ee_child_frame, str(e)
            )

    # ---------------- Payload Builder ----------------

    def _build_joints_payload(self) -> dict:
        """
        {
          "schema": "manip2.v1",
          "ts_unix_ns": ...,
          "ts_iso": "...",
          "joints": [
            { "joint":"joint1","id":1,"proto":1,"pos_deg":...,"temp_c":..,"volt_v":..,"load_pct":..,"err":"OK","err_level":0 },
            { "joint":"joint2","id":2,"proto":2,"pos_deg":...,"temp_c":..,"volt_v":..,"load_stl":..,"load_cont":..,"err":"Overload","err_level":2 }
          ]
        }
        """
        with self._lock:
            js = self._last_js
            t  = self._last_telem
            errmap = dict(self._last_errmap)

        if js is None:
            raise RuntimeError("No joint_states yet")

        names = list(js.name) if js.name else [f"joint{i+1}" for i in range(len(js.position or []))]
        pos_deg = [round(math.degrees(x), 3) for x in (js.position or [])]

        # P1 telemetry maps by ID
        p1v = p1t = p1l = {}
        if t is not None and getattr(t, "p1_ids", None):
            ids = list(t.p1_ids)
            def _m(arr): return {i: arr[k] for k,i in enumerate(ids) if k < len(arr)}
            if getattr(t, "p1_voltage_v", None)   is not None: p1v = _m(list(t.p1_voltage_v))
            if getattr(t, "p1_temp_c", None)      is not None: p1t = _m(list(t.p1_temp_c))
            if getattr(t, "p1_load_percent", None) is not None: p1l = _m(list(t.p1_load_percent))

        # P2 values (single actuator)
        p2_id    = getattr(t, "p2_id", self.dxl_id_p2) if t is not None else self.dxl_id_p2
        p2_volt  = getattr(t, "p2_voltage_v", None) if t is not None else None
        p2_temp  = getattr(t, "p2_temp_c", None) if t is not None else None
        p2_cur_a = getattr(t, "p2_current_a", None) if t is not None else None

        load_stl = load_cont = None
        if p2_cur_a is not None:
            load_stl, load_cont = p2_load_percentages(p2_cur_a,
                                                      self.p2_stall_current_a,
                                                      self.p2_cont_current_a)

        joints = []
        n = max(len(self.dxl_ids), len(names), len(pos_deg))
        for i in range(n):
            name = names[i] if i < len(names) else f"joint{i+1}"
            if i < len(self.dxl_ids):
                jid = self.dxl_ids[i]
            else:
                jid = i+1
            try:
                jid_int = int(jid)
            except Exception:
                rospy.logwarn_throttle(1.0, "Bad dxl_id '%r' at index %d; skipping", jid, i)
                continue

            proto = 2 if jid_int == p2_id else 1
            obj = {"joint": name, "id": jid_int, "proto": proto}

            if i < len(pos_deg):
                obj["pos_deg"] = float(pos_deg[i])

            if proto == 1:
                if jid_int in p1v: obj["volt_v"] = float(p1v[jid_int])
                if jid_int in p1t: obj["temp_c"] = int(round(p1t[jid_int]))
                if jid_int in p1l: obj["load_pct"] = float(p1l[jid_int])
            else:
                if p2_volt is not None: obj["volt_v"] = float(p2_volt)
                if p2_temp is not None: obj["temp_c"] = int(round(p2_temp))
                if load_stl is not None:  obj["load_stl"]  = float(load_stl)
                if load_cont is not None: obj["load_cont"] = float(load_cont)
                if p2_cur_a is not None:  obj["current_a"] = float(round(p2_cur_a, 4))  # optional

            if jid_int in errmap:
                lvl, msg = errmap[jid_int]
                obj["err_level"] = int(lvl)
                obj["err"] = str(msg)

            joints.append(obj)

        # ---- Build timestamps & seq from telemetry header ----
        ts_unix_ns = _now_ns()
        ts_iso = _iso_now()
        seq = None

        if t is not None:
            hdr = getattr(t, "header", None)
            if hdr is not None:
                # sequence number
                if hasattr(hdr, "seq"):
                    try:
                        seq = int(hdr.seq)
                    except Exception:
                        seq = None

                # timestamp from header.stamp -> ISO + ns
                stamp = getattr(hdr, "stamp", None)
                if stamp is not None:
                    try:
                        secs = stamp.to_sec()
                        ts_unix_ns = int(secs * 1e9)
                        ts_iso = datetime.datetime.fromtimestamp(
                            secs, datetime.timezone.utc
                        ).isoformat()
                    except Exception:
                        # fall back to "now" helpers if conversion fails
                        ts_unix_ns = _now_ns()
                        ts_iso = _iso_now()

        payload = {
            "schema": "manip2.v1",
            "ts_unix_ns": ts_unix_ns,
            "ts_iso": ts_iso,
            "joints": joints,
        }
        if seq is not None:
            payload["seq"] = seq

        return payload


def main():
    rospy.init_node("manip2_web_bridge", anonymous=False)
    _ = Manip2WebBridge()
    rospy.loginfo("manip2_web_bridge running.")
    rospy.spin()


if __name__ == "__main__":
    main()

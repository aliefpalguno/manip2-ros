#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, threading, signal, sys, ast
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from dynamixel_sdk import PortHandler, PacketHandler

# ---------- param helpers ----------
def _ensure_list(obj):
    if isinstance(obj, list): return obj
    if isinstance(obj, tuple): return list(obj)
    if isinstance(obj, str):
        try: return ast.literal_eval(obj)
        except Exception: pass
    return [obj]
def _to_list_floats(obj): return [float(x) for x in _ensure_list(obj)]
def _to_list_ints(obj):   return [int(x)   for x in _ensure_list(obj)]
def _to_list2_floats(obj):
    m = _ensure_list(obj); out=[]
    for row in m: out.append([float(x) for x in _ensure_list(row)])
    return out

# ---------- int32 helpers for Protocol 2 ----------
def _i32_from_u32(u):
    u &= 0xFFFFFFFF
    return u if u < 0x80000000 else u - 0x100000000
def _i32_to_u32(i):
    return i & 0xFFFFFFFF

# ---------- defaults ----------
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1000000
DEFAULT_IDS  = [1,2,3,4,5]
DEFAULT_P2ID = 2
SPEED_P1_VALUE = 50
SPEED_P2_VALUE = 350
DEG_LIMITS = [[-90,90],[-30,36],[-105,0],[-75,75],[-90,90]]
TICK_OFFSETS_NOM  = [491, -4251, 1720, 510, 498]
DEGS_PER_TICK_NOM = [300.0/1024.0, 360.0/303750.0, 250.92/4095.0, -300.0/1024.0, 300.0/1024.0]

# P2 signed range (ticks)
P2_MIN_I32 = -151875
P2_MAX_I32 =  151875
# P1 ranges (unsigned)
J1_MAX = 1023
J3_MAX = 4095
J4_MAX = 1023
J5_MAX = 1023

PROTOCOL_VERSION1 = 1.0
PROTOCOL_VERSION2 = 2.0
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# P1 control table
ADDR_MX_CW_ANGLE_LIMIT     = 6
ADDR_MX_CCW_ANGLE_LIMIT    = 8
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

ADDR_MX_RETURN_DELAY_TIME = 5
ADDR_MX_STATUS_RETURN_LEVEL = 16

# P2 control table (signed int32 positions)
ADDR_PRO_OPERATING_MODE    = 11   # 3 = Position mode
ADDR_PRO_TORQUE_ENABLE     = 562
ADDR_PRO_GOAL_VELOCITY     = 600
ADDR_PRO_GOAL_POSITION     = 596
ADDR_PRO_PRESENT_POSITION  = 611

ADDR_PRO_RETURN_DELAY_TIME = 9
ADDR_PRO_STATUS_RETURN_LEVEL = 891

class Manip2DxlMatlab:
    def __init__(self):
        # Params
        self.port_name   = rospy.get_param("~port_name", DEFAULT_PORT)
        self.baud_rate   = int(rospy.get_param("~baud_rate", DEFAULT_BAUD))
        self.dxl_ids     = _to_list_ints(rospy.get_param("~dxl_ids", DEFAULT_IDS))
        self.dxl_id_p2   = int(rospy.get_param("~dxl_id_p2", DEFAULT_P2ID))
        self.speed_p1    = int(rospy.get_param("~speed_p1_value", SPEED_P1_VALUE))
        self.speed_p2    = int(rospy.get_param("~speed_p2_value", SPEED_P2_VALUE))
        self.read_rate   = float(rospy.get_param("~read_rate_hz", 60.0))
        self.print_reads = bool(rospy.get_param("~print_reads", True))

        # NEW: toggle verifikasi tulis goal register (default: False untuk performa)
        self.verify_writes = bool(rospy.get_param("~verify_writes", False))

        self.deg_limits   = _to_list2_floats(rospy.get_param("~urdf_limits_deg", DEG_LIMITS))
        self.tick_offsets = _to_list_ints(rospy.get_param("~tick_offsets_nom", TICK_OFFSETS_NOM))
        self.degs_per_tick = _to_list_floats(rospy.get_param("~degs_per_tick_nom", DEGS_PER_TICK_NOM))
        self.rads_per_tick = [math.radians(dpt) for dpt in self.degs_per_tick]

        # State
        self._stop = threading.Event()
        self._port_open = False
        self.initial_ticks = None
        self.initial_deg   = None

        self.agent_enabled = True

        # SDK
        self.port = PortHandler(self.port_name)
        self.ph1  = PacketHandler(PROTOCOL_VERSION1)
        self.ph2  = PacketHandler(PROTOCOL_VERSION2)

        # ---- single bus lock to serialize ALL SDK I/O ----
        self.bus_lock = threading.RLock()

        # Last Ticks stored for Failback
        self.last_ticks = [0]*len(self.dxl_ids)

        # Open + baud
        with self.bus_lock:
            if not self.port.openPort():  raise RuntimeError("openPort failed")
            self._port_open = True
            if not self.port.setBaudRate(self.baud_rate): raise RuntimeError("setBaudRate failed")
        rospy.loginfo("[OK] Port open @ %s, %d baud", self.port_name, self.baud_rate)

        # Set Status Return Level = 1 (hanya PING & READ yang balas)
        rospy.loginfo("Setting status return level to 1...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_STATUS_RETURN_LEVEL, 1)
                with self.bus_lock:
                    p2_srl, rc, er = self.ph2.read1ByteTxRx(self.port, sid, ADDR_PRO_STATUS_RETURN_LEVEL)
                if rc!=0 or er!=0:
                    rospy.logwarn("P2 status return level comm=%s err=%s", self.ph2.getTxRxResult(rc), self.ph2.getRxPacketError(er))
                else:
                    rospy.loginfo("P2 status return level set to %s", p2_srl)
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_STATUS_RETURN_LEVEL, 1)
                with self.bus_lock:
                    p1_srl, rc, er = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_STATUS_RETURN_LEVEL)
                if rc!=0 or er!=0:
                    rospy.logwarn("P1[ID %d] status return level comm=%s err=%s", sid, self.ph1.getTxRxResult(rc), self.ph1.getRxPacketError(er))
                else:
                    rospy.loginfo("P1[ID %d] status return level set to %s", sid, p1_srl)

        # Set Return Delay Time = 0 (minimum latency)
        rospy.loginfo("Setting return delay time to 0...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_RETURN_DELAY_TIME, 0)
                with self.bus_lock:
                    p2_rdt, rc, er = self.ph2.read1ByteTxRx(self.port, sid, ADDR_PRO_RETURN_DELAY_TIME)
                if rc!=0 or er!=0:
                    rospy.logwarn("P2 return delay time val=%s comm=%s err=%s", p2_rdt, self.ph2.getTxRxResult(rc), self.ph2.getRxPacketError(er))
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_RETURN_DELAY_TIME, 0)
                with self.bus_lock:
                    p1_rdt, rc, er = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_RETURN_DELAY_TIME)
                if rc!=0 or er!=0:
                    rospy.logwarn("P1[ID %d] return delay time val=%s comm=%s err=%s", sid, p1_rdt, self.ph1.getTxRxResult(rc), self.ph1.getRxPacketError(er))

        # Ensure P2 is in Position Mode (3) dengan writeTxOnly + verifikasi
        try:
            with self.bus_lock:
                cur_mode, res, err = self.ph2.read1ByteTxRx(self.port, self.dxl_id_p2, ADDR_PRO_OPERATING_MODE)
            if res==0 and err==0:
                rospy.loginfo("P2 OperatingMode before: %d", cur_mode)
                if cur_mode != 3:
                    with self.bus_lock:
                        self.ph2.write1ByteTxOnly(self.port, self.dxl_id_p2, ADDR_PRO_OPERATING_MODE, 3)
                    with self.bus_lock:
                        cur_mode, rc, er = self.ph2.read1ByteTxRx(self.port, self.dxl_id_p2, ADDR_PRO_OPERATING_MODE)
                    if rc!=0 or er!=0:
                        rospy.logwarn("Set P2 mode comm=%s err=%s", self.ph2.getTxRxResult(rc), self.ph2.getRxPacketError(er))
            else:
                rospy.logwarn("Read P2 mode res=%s err=%s", self.ph2.getTxRxResult(res), self.ph2.getRxPacketError(err))
        except Exception as e:
            rospy.logwarn("OperatingMode check failed: %s", e)


        # Speeds
        rospy.loginfo("Setting moving speeds...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write4ByteTxOnly(self.port, sid, ADDR_PRO_GOAL_VELOCITY, self.speed_p2)
                with self.bus_lock:
                    p2_mov_speed, dxl_comm_result, dxl_error = self.ph2.read4ByteTxRx(self.port, sid, ADDR_PRO_GOAL_VELOCITY)
                if dxl_comm_result!=0 or dxl_error!=0:
                    rospy.logwarn("P2 speed set to %s %s / %s", p2_mov_speed, self.ph2.getTxRxResult(dxl_comm_result), self.ph2.getRxPacketError(dxl_error))
            else:
                with self.bus_lock:
                    self.ph1.write2ByteTxOnly(self.port, sid, ADDR_MX_MOVING_SPEED, self.speed_p1)
                with self.bus_lock:
                    p1_mov_speed, dxl_comm_result, dxl_error = self.ph1.read2ByteTxRx(self.port, sid, ADDR_MX_MOVING_SPEED)
                if dxl_comm_result!=0 or dxl_error!=0:
                    rospy.logwarn("P1[ID %d] speed set to %s %s / %s", sid, p1_mov_speed, self.ph1.getTxRxResult(dxl_comm_result), self.ph1.getRxPacketError(dxl_error))

        # Torque ON
        rospy.loginfo("Enabling torque...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
                with self.bus_lock:
                    p2_torque, dxl_comm_result, dxl_error = self.ph2.read1ByteTxRx(self.port, sid, ADDR_PRO_TORQUE_ENABLE)
                if dxl_comm_result!=0 or dxl_error!=0:
                    rospy.logwarn("P2 torque on (val:%s) : %s / %s", p2_torque, self.ph2.getTxRxResult(dxl_comm_result), self.ph2.getRxPacketError(dxl_error))
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                with self.bus_lock:
                    p1_torque, dxl_comm_result, dxl_error = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_TORQUE_ENABLE)
                if dxl_comm_result!=0 or dxl_error!=0:
                    rospy.logwarn("P1[ID %d] torque on (val:%s) : %s / %s", sid, p1_torque, self.ph1.getTxRxResult(dxl_comm_result), self.ph1.getRxPacketError(dxl_error))

        
        self.boot_move_to_initial = bool(rospy.get_param("~boot_move_to_initial", True))
        self.boot_initial_deg     = _to_list_floats(rospy.get_param("~boot_initial_deg", [0, 36, -105.7, 0, 0]))
        self.boot_wait_sec        = float(rospy.get_param("~boot_wait_sec", 6.0))
        self.boot_reach_tol_deg   = float(rospy.get_param("~boot_reach_tol_deg", 0.5))
        self.boot_timeout_sec     = float(rospy.get_param("~boot_timeout_sec", 20.0))

        # # Initial readback (becomes our "initial pose")
        # self.initial_ticks = self.read_all_ticks()
        # self.initial_deg   = self.ticks_to_deg(self.initial_ticks)
        # rospy.loginfo("Initial ticks: %s", self.initial_ticks)
        # rospy.loginfo("Initial deg  : [%.1f %.1f %.1f %.1f %.1f]", *self.initial_deg)

        # --- Boot move to predetermined initial degrees (before agent takes over) ---
        if self.boot_move_to_initial:
            # Ensure agent cannot publish during boot move
            self._preempt_agent("boot_move_to_initial")

            # Clip to URDF limits (existing behavior inside send_degrees will also clip)
            boot_deg = list(self.boot_initial_deg)
            for i in range(5):
                lo, hi = self.deg_limits[i]
                if boot_deg[i] < lo or boot_deg[i] > hi:
                    rospy.logwarn("Boot pose j%d %.1f° outside [%.1f, %.1f]; clipping.",
                                i+1, boot_deg[i], lo, hi)
                    boot_deg[i] = min(max(boot_deg[i], lo), hi)

            rospy.loginfo("Boot-moving to initial deg: [%.1f %.1f %.1f %.1f %.1f]",
                        *[round(x,1) for x in boot_deg])
            self.send_degrees(boot_deg)

            # coarse settle wait
            rospy.sleep(self.boot_wait_sec)

            # fine wait until reached (or timeout)
            readback_deg, reached = self._wait_until_reached(
                boot_deg, tol_deg=self.boot_reach_tol_deg, timeout_sec=self.boot_timeout_sec
            )
            if reached:
                rospy.loginfo("Boot pose reached within %.1f° tolerance.", self.boot_reach_tol_deg)
            else:
                rospy.logwarn("Boot pose NOT fully reached (timeout). Continuing with last readback.")

            # Store as initial
            self.initial_deg = readback_deg
            self.initial_ticks = self.read_all_ticks()
        else:
            # Fallback: keep previous behavior
            self.initial_ticks = self.read_all_ticks()
            self.initial_deg   = self.ticks_to_deg(self.initial_ticks)

        rospy.loginfo("Initial ticks: %s", self.initial_ticks)
        rospy.loginfo("Initial deg  : [%.1f %.1f %.1f %.1f %.1f]", *self.initial_deg)


        # Initialize publishers/subscribers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.feedback_pub = rospy.Publisher('/ros/current_joint_state_deg', Float64MultiArray, queue_size=1)
        self.agent_state_pub = rospy.Publisher('/matlab_perceived/joint_states', JointState, queue_size=1)
        self.joint_cmd_sub = rospy.Subscriber('/matlab/joint_command_deg', Float64MultiArray, self.command_callback)
        self.joint_perceived_sub = rospy.Subscriber('/matlab/perceived_joint_state', Float64MultiArray, self.perceived_callback)

        self.srv_agent_enable  = rospy.Service("/manip2/agent_enable",  Trigger, self._agent_enable)
        self.srv_agent_disable = rospy.Service("/manip2/agent_disable", Trigger, self._agent_disable)

        # Re-enable agent input 
        self.agent_enabled = True

        # self.srv_go_initial = rospy.Service("/manip2/go_initial", Trigger, self.on_go_initial)
        self.srv_direct_initial = rospy.Service("/manip2/direct_initial", Trigger, self.direct_go_initial)
        self.srv_emergency_stop = rospy.Service("/manip2/emergency_stop", Trigger, self.handle_emergency_stop)
        self.srv_go_init_and_quit = rospy.Service("/manip2/go_initial_and_shutdown", Trigger, self.on_go_init_and_quit)

        self.srv_target_reached = rospy.Service("/manip2/target_reached", Trigger, self.on_target_reached)


                
        # Reader thread
        self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.read_thread.start()

        rospy.on_shutdown(self.shutdown)
        signal.signal(signal.SIGINT, self._sigint)

    def _wait_until_reached(self, target_deg, tol_deg=0.5, timeout_sec=20.0, check_rate_hz=50.0):
        r = rospy.Rate(check_rate_hz if check_rate_hz > 0 else 10.0)
        t0 = rospy.Time.now()
        last_deg = None
        while not rospy.is_shutdown():
            ticks = self.read_all_ticks()
            cur_deg = self.ticks_to_deg(ticks)
            last_deg = cur_deg
            diffs = [abs(cur_deg[i] - target_deg[i]) for i in range(5)]
            if all(d <= tol_deg for d in diffs):
                return cur_deg, True
            if (rospy.Time.now() - t0).to_sec() > timeout_sec:
                return cur_deg, False
            r.sleep()
        return last_deg, False


    # Agent Control
    def _agent_enable(self, _):
        self.agent_enabled = True
        return TriggerResponse(True, "Agent control enabled.")

    def _agent_disable(self, _):
        self.agent_enabled = False
        return TriggerResponse(True, "Agent control disabled.")

    # helper to preempt / resume the agent publisher
    def _preempt_agent(self, reason=""):
        if self.agent_enabled:
            rospy.logwarn("Preempting agent (%s). Ignoring /matlab/joint_command_deg.", reason)
        self.agent_enabled = False
        try:
            if self.joint_cmd_sub is not None:
                self.joint_cmd_sub.unregister()
                self.joint_cmd_sub = None
        except Exception:
            pass
        rospy.sleep(0.05)  # let any in-flight callback finish

    def _resume_agent(self):
        if self.joint_cmd_sub is None:
            self.joint_cmd_sub = rospy.Subscriber('/matlab/joint_command_deg',
                                                Float64MultiArray, self.command_callback)
        self.agent_enabled = True

    # == Callbacks ==
    def command_callback(self, msg: Float64MultiArray):
        if not self.agent_enabled or not self._port_open or self._stop.is_set():
            rospy.logwarn_throttle(2.0, "Ignoring agent command (agent disabled or port closed).")
            return
        
        vals = _to_list_floats(msg.data)
        if len(vals) != 5:
            rospy.logwarn("Expected 5 deg values, got %d", len(vals)); return
        for i in range(5):
            lo, hi = self.deg_limits[i]
            if vals[i] < lo or vals[i] > hi:
                rospy.logwarn("Joint j%d %.1f° outside [%.1f, %.1f], clipping.", i+1, vals[i], lo, hi)
                vals[i] = min(max(vals[i], lo), hi)
        self.send_degrees(vals)

    def direct_go_initial(self, _req):
        if self.initial_deg is None:
            return TriggerResponse(success=False, message="No initial pose stored.")
        # PREEMPT agent
        self._preempt_agent("direct_initial")
        try:
            # (optional) briefly wait to let any in-flight callback finish
            rospy.sleep(0.5)
            self.send_degrees(self.initial_deg)
            return TriggerResponse(success=True, message="Agent preempted; moving to initial.")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

        
    # == Main Interfacing ==
    def send_degrees(self, q_deg):
        if not self._port_open or self._stop.is_set():
            rospy.logwarn("send_degrees ignored (port closed or stopping).")
            return
        
        q_rad  = [math.radians(x) for x in q_deg]
        ticks  = [self.tick_offsets[i] + (q_rad[i] / self.rads_per_tick[i]) for i in range(5)]
        ticksr = [int(round(t)) for t in ticks]

        # clamp per-joint
        ticksr[0] = max(0, min(J1_MAX, ticksr[0]))
        idx_p2 = self.dxl_ids.index(self.dxl_id_p2)
        ticksr[idx_p2] = max(P2_MIN_I32, min(P2_MAX_I32, ticksr[idx_p2]))
        ticksr[2] = max(0, min(J3_MAX, ticksr[2]))
        ticksr[3] = max(0, min(J4_MAX, ticksr[3]))
        ticksr[4] = max(0, min(J5_MAX, ticksr[4]))

        # per-servo writes with BUS LOCK
        for i, sid in enumerate(self.dxl_ids):
            if sid == self.dxl_id_p2:
                u = _i32_to_u32(ticksr[i])
                with self.bus_lock:
                    self.ph2.write4ByteTxOnly(self.port, sid, ADDR_PRO_GOAL_POSITION, u)
            else:
                val = ticksr[i] & 0xFFFF
                with self.bus_lock:
                    self.ph1.write2ByteTxOnly(self.port, sid, ADDR_MX_GOAL_POSITION, val)

        # ... bagian verifikasi read goal register tetap seperti di file saat ini ...
        rospy.loginfo("Sent ticks (clamped): %s  (from deg %s)", ticksr, [round(x,1) for x in q_deg])

        # verify (opsional, tergantung param)
        if self.verify_writes:
            ver=[]
            for i, sid in enumerate(self.dxl_ids):
                if sid == self.dxl_id_p2:
                    with self.bus_lock:
                        goal_u, res, err = self.ph2.read4ByteTxRx(self.port, sid, ADDR_PRO_GOAL_POSITION)
                    if res!=0 or err!=0:
                        rospy.logerr("P2[ID %d] ReadGoal: %s / %s",
                                    sid, self.ph2.getTxRxResult(res), self.ph2.getRxPacketError(err))
                    goal_i = _i32_from_u32(goal_u)
                    ver.append(f"ID {sid} goal_reg i32={goal_i} (sent {ticksr[i]})")
                else:
                    with self.bus_lock:
                        goal, res, err = self.ph1.read2ByteTxRx(self.port, sid, ADDR_MX_GOAL_POSITION)
                    if res!=0 or err!=0:
                        rospy.logerr("P1[ID %d] ReadGoal: %s / %s",
                                    sid, self.ph1.getTxRxResult(res), self.ph1.getRxPacketError(err))
                    ver.append(f"ID {sid} goal_reg={goal} (sent {ticksr[i]})")
            rospy.loginfo("Write verify: " + " | ".join(ver))

    # ---------- reads ------------
    def read_all_ticks(self):
        out = list(self.last_ticks)  # start with last good
        for i, sid in enumerate(self.dxl_ids):
            if sid == self.dxl_id_p2:
                # one retry for P2
                ok = False
                for _ in range(2):  # try twice
                    with self.bus_lock:
                        val_u, res_c, res_e = self.ph2.read4ByteTxRx(self.port, sid, ADDR_PRO_PRESENT_POSITION)
                    if res_c==0 and res_e==0:
                        out[i] = _i32_from_u32(val_u)
                        ok = True
                        break
                if not ok:
                    rospy.logwarn_throttle(1.0, "P2 read present failed twice; keeping last=%d", out[i])
            else:
                # one retry for P1 too
                ok = False
                for _ in range(2):
                    with self.bus_lock:
                        val, res_c, res_e = self.ph1.read2ByteTxRx(self.port, sid, ADDR_MX_PRESENT_POSITION)
                    if res_c==0 and res_e==0:
                        out[i] = int(val) & 0xFFFF
                        ok = True
                        break
                if not ok:
                    rospy.logwarn_throttle(1.0, "P1[ID %d] read present failed twice; keeping last=%d", sid, out[i])
        self.last_ticks = out
        return out
        
    # ---------- conversions ----------
    def ticks_to_deg(self, ticks):
        out = []
        for i in range(5):
            q_rad = (ticks[i] - self.tick_offsets[i]) * self.rads_per_tick[i]
            out.append(math.degrees(q_rad))
        return out
    

        
        # ---------- reader thread ----------
    def _reader_loop(self):
        rate = rospy.Rate(self.read_rate if self.read_rate > 0 else 10.0)
        names = ["joint1","joint2","joint3","joint4","joint5"]
        while not rospy.is_shutdown() and not self._stop.is_set():
            try:
                ticks = self.read_all_ticks()              # one bus read
                q_deg = self.ticks_to_deg(ticks)
                q_rad = [math.radians(x) for x in q_deg]

                # RViz
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = names
                js.position = q_rad
                self.joint_state_pub.publish(js)

                # MATLAB feedback in degrees
                fb = Float64MultiArray()
                fb.data = q_deg
                self.feedback_pub.publish(fb)

                if self.print_reads:
                    rospy.loginfo_throttle(0.1,
                        "PRESENT ticks=%s | deg=[%.1f %.1f %.1f %.1f %.1f]",
                        ticks, *[round(x,1) for x in q_deg])
            except Exception as e:
                rospy.logwarn_throttle(2.0, "Read loop warning: %s", str(e))
            rate.sleep()

    # Receive agent's perceived joint state, publish to secondary JointState topic for RViz
    def perceived_callback(self, msg):
        joint_msg = JointState()
        joint_msg.name = ['joint1','joint2','joint3','joint4','joint5']
        joint_msg.position = [math.radians(x) for x in list(msg.data)] # Here, im not sure i have the deg 2 rad, VSC yellow highlighted this deg2rad
        joint_msg.header.stamp = rospy.Time.now()
        self.agent_state_pub.publish(joint_msg)
    
    # # Publish to /joint_states (rviz), /ros/current_joint_state_deg (for MATLAB feedback)
    # def publish_real_joint_state(self):
    #     rate = rospy.Rate(30)
    #     while not rospy.is_shutdown():
    #         with self.bus_lock:
    #             # Read all ticks from Dynamixel, convert to degrees and radians
    #             # Publish to /joint_states (rviz), /ros/current_joint_state_deg (for MATLAB feedback)
    #             joint_msg = JointState()
    #             joint_msg.name = ['joint1','joint2','joint3','joint4','joint5']
    #             joint_msg.position = [math.radians(self.ticks_to_deg(tick)) for tick in self.read_all_ticks()]
    #             joint_msg.header.stamp = rospy.Time.now()
    #             self.joint_state_pub.publish(joint_msg)
    #             feedback = Float64MultiArray()
    #             feedback.data = [self.ticks_to_deg(tick) for tick in self.read_all_ticks()]
    #             self.feedback_pub.publish(feedback)
    #         rate.sleep()

    def handle_emergency_stop(self, req):
        rospy.logwarn("Emergency stop service called: moving to initial pose and shutting down safely.")
        self._preempt_agent("emergency_stop")
        try:
            if self.initial_deg is not None:
                self.send_degrees(self.initial_deg)
                rospy.sleep(6)  # let it move
            # block further traffic and shut down safely
            self._stop.set()
            self.shutdown()
            return TriggerResponse(success=True, message="Emergency stop completed: moved to initial and shutdown")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Emergency stop failed: {e}")
        
    # def handle_target_reached(self, req):
    #     print("\nRobot reached the target. Select next action:")
    #     print("1: Wait for new target")
    #     print("2: Move to initial position")
    #     print("3: Move to initial then shutdown")
    #     choice = input("Enter choice (1/2/3): ").strip()
    #     if choice == '1':
    #         return TriggerResponse(True, "Waiting for new target")
    #     elif choice == '2':
    #         self.send_degrees(self.initial_deg)
    #         return TriggerResponse(True, "Moved to initial position")
    #     elif choice == '3':
    #         self.send_degrees(self.initial_deg)
    #         rospy.sleep(5)
    #         self.shutdown()
    #         rospy.signal_shutdown("Shutdown after moving to initial")
    #         sys.exit(0)
    #     else:
    #         return TriggerResponse(False, "Invalid choice")

    def on_target_reached(self, _req):
        rospy.loginfo("Target reached acknowledged.")
        return TriggerResponse(success=True, message="OK")
    
    def on_go_init_and_quit(self, _req):
        self._preempt_agent("go_initial_and_shutdown")
        try:
            if self.initial_deg is not None:
                self.send_degrees(self.initial_deg)
                rospy.sleep(6)
            self._stop.set()
            self.shutdown()
            return TriggerResponse(True, "Init then shutdown.")
        except Exception as e:
            return TriggerResponse(False, "Failed: %s" % e)



    # ---------- shutdown (disable torque with read-back & retries) ----------
    def shutdown(self):
        if getattr(self, "_shutting_down", False):
            return
        self._shutting_down = True
        self._stop.set()

        try:
            try:
                if hasattr(self, "read_thread") and self.read_thread.is_alive():
                    self.read_thread.join(timeout=1.0)
            except Exception:
                pass

            rospy.loginfo("Disabling torque...")
            for sid in self.dxl_ids:
                if not self._port_open:
                    break
                for attempt in range(3):
                    if sid == self.dxl_id_p2:
                        with self.bus_lock:
                            self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
                        with self.bus_lock:
                            val, r, e = self.ph2.read1ByteTxRx(self.port, sid, ADDR_PRO_TORQUE_ENABLE)
                        if r!=0 or e!=0 or val!=0:
                            rospy.logwarn("P2 Disable Torque Failed! val=%s comm=%s, err=%s", val, r, e)
                        else:
                            break
                    else:
                        with self.bus_lock:
                            self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
                        with self.bus_lock:
                            val, r, e = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_TORQUE_ENABLE)
                        if r!=0 or e!=0 or val!=0:
                            rospy.logwarn("P1 [ID:%s] Disable Torque Failed! val=%s comm=%s, err=%s", sid, val, r, e)
                        else:
                            break
                    rospy.sleep(0.03)
        finally:
            if self._port_open:
                try:
                    rospy.sleep(0.05)  # let bus flush
                    with self.bus_lock:
                        self.port.closePort()
                    self._port_open = False
                    rospy.loginfo("[OK] Port closed.")
                except Exception as e:
                    rospy.logwarn("Close port: %s", str(e))



    def _sigint(self, *_):
        rospy.loginfo("Emergency stop triggered (SIGINT). Moving to initial pose...")
        try:
            self._preempt_agent("USER INTERRUPT")
            # Move to initial if possible
            if self.initial_deg is not None and self._port_open and not self._stop.is_set():
                self.send_degrees(self.initial_deg)
                rospy.sleep(6)  # short pause to start the motion
        except Exception as e:
            rospy.logwarn("Failsafe move-to-initial error: %s", e)
            
        finally:
            # Ensure we stop background loops and close the port cleanly
            self._stop.set()
            try:
                self.shutdown()
            finally:
                rospy.signal_shutdown("SIGINT")
                # don't call sys.exit here; ROS will exit after shutdown


def main():
    rospy.init_node('manip2_dxl_matlab')
    node = Manip2DxlMatlab()
    rospy.loginfo("manip2_dxl_matlab running. Run the agent through MATLAB.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        node.shutdown()

if __name__ == "__main__":
    main()
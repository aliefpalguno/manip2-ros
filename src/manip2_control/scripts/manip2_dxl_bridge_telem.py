#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, threading, signal, sys, ast
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import UInt8MultiArray, String  # new
from manip2_msgs.msg import ManipTelemetry
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupBulkRead

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
SPEED_P1_VALUE = 20
SPEED_P2_VALUE = 250
DEG_LIMITS = [[-90,90],[-30,36],[-105.7,0],[-75,75],[-90,90]]
TICK_OFFSETS_NOM  = [541, -3528, 1715, 506, 504]
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
ADDR_RX_RETURN_DELAY_TIME   = 5
ADDR_RX_CW_ANGLE_LIMIT      = 6
ADDR_RX_CCW_ANGLE_LIMIT     = 8
ADDR_RX_STATUS_RETURN_LEVEL = 16

ADDR_RX_SHUTDOWN            = 18
ADDR_RX_TORQUE_ENABLE       = 24
ADDR_RX_MOVING_SPEED        = 32
ADDR_RX_GOAL_POSITION       = 30
ADDR_RX_PRESENT_POSITION    = 36
ADDR_RX_PRESENT_LOAD        = 40
ADDR_RX_PRESENT_VOLTAGE     = 42
ADDR_RX_PRESENT_TEMPERATURE = 43


# P2 control table (signed int32 positions)
ADDR_PRO_RETURN_DELAY_TIME = 9
ADDR_PRO_OPERATING_MODE    = 11   # 3 = Position mode
ADDR_PRO_STATUS_RETURN_LEVEL = 891
ADDR_PRO_HARDWARE_ERROR_STATUS = 892

ADDR_PRO_SHUTDOWN          = 48
ADDR_PRO_TORQUE_ENABLE     = 562
ADDR_PRO_GOAL_VELOCITY     = 600
ADDR_PRO_GOAL_POSITION     = 596
ADDR_PRO_PRESENT_POSITION  = 611
ADDR_PRO_PRESENT_INPUT_VOLTAGE = 623
ADDR_PRO_PRESENT_CURRENT   = 621
ADDR_PRO_PRESENT_TEMPERATURE = 625

# ---- error decoding helpers ----
# Protocol 1.0 Status Packet Error byte (bits) per e-Manual
_P1_BITS = [
    (0, "Input Voltage"),
    (1, "Angle Limit"),
    (2, "Overheating"),
    (3, "Range"),
    (4, "Checksum"),
    (5, "Overload"),
    (6, "Instruction"),
    # bit7 reserved
]
def decode_p1_error_byte(b):
    labels = [name for (bit, name) in _P1_BITS if (b >> bit) & 0x1]
    return labels
# Protocol 2.0 Hardware Error Status M-42-Pro
_P2_HW_BITS = [
    (0, "Input Voltage"),
    (1, "Motor Hall Sensor"),
    (2, "Overheating"),
    (3, "Motor Encoder"),          # may be unused on some models
    (4, "Electrical Shock"),
    (5, "Overload"),
    # bits 6,7 unused
]
def decode_p2_hwerr_byte(b):
    labels = [name for (bit, name) in _P2_HW_BITS if (b >> bit) & 0x1]
    return labels



class Manip2DxlBridge:
    def __init__(self):
        # Params
        self.port_name   = rospy.get_param("~port_name", DEFAULT_PORT)
        self.baud_rate   = int(rospy.get_param("~baud_rate", DEFAULT_BAUD))
        self.dxl_ids     = _to_list_ints(rospy.get_param("~dxl_ids", DEFAULT_IDS))
        self.dxl_id_p2   = int(rospy.get_param("~dxl_id_p2", DEFAULT_P2ID))
        self.speed_p1    = int(rospy.get_param("~speed_p1_value", SPEED_P1_VALUE))
        self.speed_p2    = int(rospy.get_param("~speed_p2_value", SPEED_P2_VALUE))

        self.read_rate   = float(rospy.get_param("~read_rate_hz", 60.0))
        self.telemetry_rate = float(rospy.get_param("~telemetry_rate_hz", 1.0))
        self.print_reads = bool(rospy.get_param("~print_reads", True))
        self.print_throttle_s = float(rospy.get_param("~print_throttle_s", 0.1))

        self.deg_limits   = _to_list2_floats(rospy.get_param("~urdf_limits_deg", DEG_LIMITS))
        self.tick_offsets = _to_list_ints(rospy.get_param("~tick_offsets_nom", TICK_OFFSETS_NOM))
        self.degs_per_tick = _to_list_floats(rospy.get_param("~degs_per_tick_nom", DEGS_PER_TICK_NOM))
        self.rads_per_tick = [math.radians(dpt) for dpt in self.degs_per_tick]

        self.p2_hw_error_addr = int(rospy.get_param("~p2_hw_error_addr", ADDR_PRO_HARDWARE_ERROR_STATUS))
        self.publish_error_text = bool(rospy.get_param("~publish_error_text", True)) # new

        # State
        self._stop = threading.Event()
        self._port_open = False
        self.initial_ticks = None
        self.initial_deg   = None
        # Error State
        self._last_p1_errors = [0]*len(self.dxl_ids)       # error byte per joint for Protocol 1
        self._last_p2_alert = 0                             # Protocol 2 Alert bit (Status Error bit7)
        self._last_p2_status_err = 0                            # Protocol 2 Error Number (Status Error bits 6..0)
        self._last_p2_hwerr = 0                             # Protocol 2 Hardware Error Status (latched)

        # SDK
        self.port = PortHandler(self.port_name)
        self.ph1  = PacketHandler(PROTOCOL_VERSION1)
        self.ph2  = PacketHandler(PROTOCOL_VERSION2)

        self.syncwrite_p1 = GroupSyncWrite(self.port, self.ph1, ADDR_RX_GOAL_POSITION, 2)
        self.bulkread_p2 = GroupBulkRead(self.port, self.ph2)

        # ---- single bus lock to serialize ALL SDK I/O ----
        self.bus_lock = threading.RLock()

        # Open + baud
        with self.bus_lock:
            if not self.port.openPort():  raise RuntimeError("openPort failed")
            self._port_open = True
            if not self.port.setBaudRate(self.baud_rate): raise RuntimeError("setBaudRate failed")
        rospy.loginfo("[OK] Port open @ %s, %d baud", self.port_name, self.baud_rate)

        # Set Status Return Level to 1 -> Return Status for PING and READ instruction only
        rospy.loginfo("Setting status return level to 1...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write1ByteTxOnly(self.port, self.dxl_id_p2, ADDR_PRO_STATUS_RETURN_LEVEL, 1)
                    p2_srl,  dxl_comm_result, dxl_error = self.ph2.read1ByteTxRx(self.port, self.dxl_id_p2, ADDR_PRO_STATUS_RETURN_LEVEL)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P2 status return level comm=%s err=%s", self.ph2.getTxRxResult(dxl_comm_result), self.ph2.getRxPacketError(dxl_error))
                    else:
                        rospy.loginfo("P2 status return level set to %s", p2_srl)
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_RX_STATUS_RETURN_LEVEL, 1)
                    p1_srl, dxl_comm_result, dxl_error = self.ph1.read1ByteTxRx(self.port, sid, ADDR_RX_STATUS_RETURN_LEVEL)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P1[ID %d] status return level comm=%s err=%s", sid, self.ph1.getTxRxResult(dxl_comm_result), self.ph1.getRxPacketError(dxl_error))
                    else:
                        rospy.loginfo("P1[ID %d] status return level set to %s", sid, p1_srl)

        # Ensure P2 is in Position Mode
        try:
            with self.bus_lock:
                cur_mode, res, err = self.ph2.read1ByteTxRx(self.port, self.dxl_id_p2, ADDR_PRO_OPERATING_MODE)
            if res==0 and err==0:
                rospy.loginfo("P2 OperatingMode before: %d", cur_mode)
                if cur_mode != 3:
                    with self.bus_lock:
                        self.ph2.write1ByteTxOnly(self.port, self.dxl_id_p2, ADDR_PRO_OPERATING_MODE, 3)
                        cur_mode, dxl_comm_result, dxl_error = self.ph2.read1ByteTxRx(self.port, self.dxl_id_p2, ADDR_PRO_OPERATING_MODE)
                        if dxl_comm_result!=0 or dxl_error!=0:
                            rospy.logwarn("Set P2 mode comm=%s err=%s", self.ph2.getTxRxResult(dxl_comm_result), self.ph2.getRxPacketError(dxl_error))
            else:
                rospy.logwarn("Read P2 mode res=%s err=%s", self.ph2.getTxRxResult(res), self.ph2.getRxPacketError(err))
        except Exception as e:
            rospy.logwarn("OperatingMode check failed: %s", e)

        # Return Delay Time
        rospy.loginfo("Setting return delay time to 0...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write1ByteTxOnly(self.port, self.dxl_id_p2, ADDR_PRO_RETURN_DELAY_TIME, 0)
                    p2_rdt, dxl_comm_result, dxl_error = self.ph2.read1ByteTxRx(self.port, self.dxl_id_p2, ADDR_PRO_RETURN_DELAY_TIME)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P2 return delay time val=%s comm=%s err=%s", p2_rdt, self.ph2.getTxRxResult(dxl_comm_result), self.ph2.getRxPacketError(dxl_error))
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_RX_RETURN_DELAY_TIME, 0)
                    p1_rdt, dxl_comm_result, dxl_error = self.ph1.read1ByteTxRx(self.port, sid, ADDR_RX_RETURN_DELAY_TIME)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P1[ID %d] return delay time val=%s comm=%s err=%s", sid, p1_rdt, self.ph1.getTxRxResult(dxl_comm_result), self.ph1.getRxPacketError(dxl_error))

        # Speeds
        rospy.loginfo("Setting moving speeds...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write4ByteTxOnly(self.port, sid, ADDR_PRO_GOAL_VELOCITY, self.speed_p2)
                    p2_mov_speed, dxl_comm_result, dxl_error = self.ph2.read4ByteTxRx(self.port, sid, ADDR_PRO_GOAL_VELOCITY)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P2 speed set to %s  %s / %s", p2_mov_speed,self.ph2.getTxRxResult(dxl_comm_result), self.ph2.getRxPacketError(dxl_error))
            else:
                with self.bus_lock:
                    self.ph1.write2ByteTxOnly(self.port, sid, ADDR_RX_MOVING_SPEED, self.speed_p1)
                    p1_mov_speed, dxl_comm_result, dxl_error = self.ph1.read2ByteTxRx(self.port, sid, ADDR_RX_MOVING_SPEED)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P1[ID %d] speed set to %s  %s / %s", sid, p1_mov_speed,self.ph1.getTxRxResult(dxl_comm_result), self.ph1.getRxPacketError(dxl_error))

        # Torque ON
        rospy.loginfo("Enabling torque...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
                    p2_torque, dxl_comm_result, dxl_error = self.ph2.read1ByteTxRx(self.port, sid, ADDR_PRO_TORQUE_ENABLE)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P2 torque on (val:%s) : %s / %s", p2_torque, self.ph2.getTxRxResult(dxl_comm_result), self.ph2.getRxPacketError(dxl_error))
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_RX_TORQUE_ENABLE, TORQUE_ENABLE)
                    p1_torque, dxl_comm_result, dxl_error = self.ph1.read1ByteTxRx(self.port, sid, ADDR_RX_TORQUE_ENABLE)
                    if dxl_comm_result!=0 or dxl_error!=0:
                        rospy.logwarn("P1[ID %d] torque on (val:%s) : %s / %s", sid, p1_torque, self.ph1.getTxRxResult(dxl_comm_result), self.ph1.getRxPacketError(dxl_error))

        # Initial readback (becomes our "home")
        self.initial_ticks = self.read_all_ticks()
        self.initial_deg   = self.ticks_to_deg(self.initial_ticks)
        rospy.loginfo("Initial ticks: %s", self.initial_ticks)
        rospy.loginfo("Initial deg  : [%.1f %.1f %.1f %.1f %.1f]", *self.initial_deg)

        # ROS I/O
        self.js_pub  = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.cmd_sub = rospy.Subscriber("/manip2/command_deg", Float64MultiArray, self.on_command_deg, queue_size=1)
        self.srv_home = rospy.Service("/manip2/go_home", Trigger, self.on_go_initial)
        self.srv_go_initial = rospy.Service("/manip2/go_initial", Trigger, self.on_go_initial)
        # how long to wait after go_initial before shutdown (for staged shutdown)
        self.shutdown_after_initial_wait_s = float(rospy.get_param("~shutdown_after_initial_wait_s", 6.0))

        # NEW services for safety / supervisor
        self.srv_emergency_shutdown = rospy.Service("/manip2/emergency_shutdown", Trigger, self.on_emergency_shutdown)
        self.srv_go_initial_and_shutdown = rospy.Service("/manip2/go_initial_and_shutdown", Trigger,self.on_go_initial_and_shutdown)

        self.srv_reconnect_port = rospy.Service("/manip2/reconnect_port", Trigger, self.on_reconnect_port)

        self.telem_pub = rospy.Publisher("/manip2/telemetry_raw", ManipTelemetry, queue_size=1)  # add
        self.err_pub_raw = rospy.Publisher("/manip2/error_status_raw", UInt8MultiArray, queue_size=1)  # new
        self.err_pub_text = rospy.Publisher("/manip2/error_status_text", String, queue_size=1)        # new (optional)


        # Reader thread
        # signal.signal(signal.SIGINT, self._sigint)

        self._shutting_down = False  # idempotent shutdown guard
        # ...
        self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.read_thread.start()
        self.telem_thread = threading.Thread(target=self._telem_loop, daemon=True)
        self.telem_thread.start()
        rospy.on_shutdown(self.shutdown)

    
    # ---------- callbacks ----------
    def on_command_deg(self, msg: Float64MultiArray):
        if rospy.get_param("/manip2/lockout", False):
            rospy.logwarn_throttle(1.0, "Safety lockout active; ignoring /manip2/command_deg")
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

    def on_go_initial(self, _req):
        if self.initial_deg is None:
            return TriggerResponse(success=False, message="No initial pose stored.")
        try:
            self.send_degrees(self.initial_deg)
            return TriggerResponse(success=True, message="Sent initial pose")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))
        
    def on_emergency_shutdown(self, _req):
        """
        Direct shutdown, like pressing Ctrl+C:
        - stop loops
        - torque off
        - close port
        BUT keep the node alive so we can /manip2/reconnect_port later.
        """
        def _worker():
            rospy.logwarn("Emergency shutdown requested via /manip2/emergency_shutdown")
            self.shutdown()
            # DO NOT call rospy.signal_shutdown() here

        threading.Thread(target=_worker, daemon=True).start()
        return TriggerResponse(success=True, message="Emergency shutdown started (node stays alive)")


    def on_go_initial_and_shutdown(self, _req):
        """
        1) send initial pose
        2) wait N seconds
        3) shutdown bus
        Node stays alive.
        """
        def _worker():
            ok = False
            if self.initial_deg is not None:
                try:
                    self.send_degrees(self.initial_deg)
                    ok = True
                    rospy.loginfo("go_initial_and_shutdown: initial pose sent.")
                except Exception as e:
                    rospy.logwarn("go_initial_and_shutdown: failed to send initial: %s", str(e))
            else:
                rospy.logwarn("go_initial_and_shutdown: no stored initial pose, skipping move.")

            wait_s = max(0.0, float(self.shutdown_after_initial_wait_s))
            if wait_s > 0:
                rospy.sleep(wait_s)

            rospy.logwarn("go_initial_and_shutdown: proceeding to shutdown...")
            self.shutdown()
            # DO NOT rospy.signal_shutdown()  -> we want to reconnect later

        threading.Thread(target=_worker, daemon=True).start()
        return TriggerResponse(
            success=True,
            message="Go-initial-and-shutdown started (node stays alive)"
        )

    def on_reconnect_port(self, _req):
        """
        Service to recover from an emergency shutdown without restarting ROS.
        - clears the stop flag
        - reopens port
        - restarts reader + telemetry threads
        """
        # if port is already open, just say OK
        if self._port_open:
            return TriggerResponse(success=True, message="Port already open")

        # we may have called shutdown() before, which set _stop and _shutting_down
        self._stop = threading.Event()         # new fresh event for new threads
        self._shutting_down = False            # allow future shutdowns

        try:
            self._reopen_and_reinit()
        except Exception as e:
            rospy.logwarn("Reconnect failed: %s", e)
            return TriggerResponse(success=False, message=str(e))

        # restart threads
        self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.read_thread.start()
        self.telem_thread = threading.Thread(target=self._telem_loop, daemon=True)
        self.telem_thread.start()

        return TriggerResponse(success=True, message="Reconnected and threads restarted")



    def send_degrees(self,q_deg):
        q_rad = [math.radians(x) for x in q_deg]
        ticks = [self.tick_offsets[i] + (q_rad[i] / self.rads_per_tick[i]) for i in range(5)]
        ticksr = [int(round(t)) for t in ticks]

        # clamp per-joint (same as before)
        ticksr[0] = max(0, min(J1_MAX, ticksr[0]))
        idx_p2 = self.dxl_ids.index(self.dxl_id_p2)
        ticksr[idx_p2] = max(P2_MIN_I32, min(P2_MAX_I32, ticksr[idx_p2]))
        ticksr[2] = max(0, min(J3_MAX, ticksr[2]))
        ticksr[3] = max(0, min(J4_MAX, ticksr[3]))
        ticksr[4] = max(0, min(J5_MAX, ticksr[4]))

        # --- P1: GroupSyncWrite broadcast (no Status Packet on broadcast) ---
        # Build parameter list for all Protocol 1 servos that share addr/len
        self.syncwrite_p1.clearParam()
        for i, sid in enumerate(self.dxl_ids):
            if sid == self.dxl_id_p2:
                continue  # skip P2
            val = ticksr[i] & 0xFFFF
            params = bytearray([val & 0xFF, (val >> 8) & 0xFF])
            # addParam doesn't touch the bus; thread-safe to build outside lock
            added = self.syncwrite_p1.addParam(sid, params)
            if not added:
                rospy.logwarn("GroupSyncWrite addParam failed for ID %d", sid)

        # Single broadcast transmit for P1 goals
        with self.bus_lock:
            dxl_comm_result = self.syncwrite_p1.txPacket()
            # No Status Packet expected from broadcast per spec
        self.syncwrite_p1.clearParam()

        # --- P2: single 4B goal write (SRL=1 => no reply on write) ---
        with self.bus_lock:
            self.ph2.write4ByteTxOnly(self.port, self.dxl_id_p2, ADDR_PRO_GOAL_POSITION, _i32_to_u32(ticksr[idx_p2]))
        rospy.loginfo("Sent ticks (clamped): %s (from deg %s)", ticksr, [round(x,1) for x in q_deg])

        # Optional: verify reads can be throttled or disabled via a ROS param
        if rospy.get_param("~verify_writes", False):
            ver=[]
            for i, sid in enumerate(self.dxl_ids):
                if sid == self.dxl_id_p2:
                    with self.bus_lock:
                        goal_u, res, err = self.ph2.read4ByteTxRx(self.port, sid, ADDR_PRO_GOAL_POSITION)
                    if res!=0 or err!=0:
                        rospy.logerr("P2[ID %d] ReadGoal: %s / %s", sid, self.ph2.getTxRxResult(res), self.ph2.getRxPacketError(err))
                    goal_i = _i32_from_u32(goal_u)
                    ver.append(f"ID {sid} goal_reg i32={goal_i} (sent {ticksr[i]})")
                else:
                    with self.bus_lock:
                        goal, res, err = self.ph1.read2ByteTxRx(self.port, sid, ADDR_RX_GOAL_POSITION)
                    if res!=0 or err!=0:
                        rospy.logerr("P1[ID %d] ReadGoal: %s / %s", sid, self.ph1.getTxRxResult(res), self.ph1.getRxPacketError(err))
                    ver.append(f"ID {sid} goal_reg={goal} (sent {ticksr[i]})")
            rospy.loginfo("Write verify: " + " | ".join(ver))


    def read_all_ticks(self):
        res = []
        # P1 block (under one lock)
        with self.bus_lock:
            for i, sid in enumerate(self.dxl_ids):
                if sid == self.dxl_id_p2:
                    continue
                val, rc, re = self.ph1.read2ByteTxRx(self.port, sid, ADDR_RX_PRESENT_POSITION)
                # store Protocol 1 error byte from this READ's Status Packet
                self._last_p1_errors[i] = int(re) & 0xFF
                if rc!=0 or re!=0:
                    rospy.logwarn("P1[ID %d] read pres: %s / %s", sid, self.ph1.getTxRxResult(rc), self.ph1.getRxPacketError(re))
                res.append(int(val) & 0xFFFF)
        # P2 single read (separate lock as separate phase)
        with self.bus_lock:
            val_u, rc, re = self.ph2.read4ByteTxRx(self.port, self.dxl_id_p2, ADDR_PRO_PRESENT_POSITION)
            # store Protocol 2 Status Packet error bits:
            # bit7 = Alert (hardware fault present), bits[0..6] are protocol-level error flags per Protocol 2.0
            self._last_p2_alert = 1 if (int(re) & 0x80) else 0
            self._last_p2_status_err = int(re) & 0x7F
            if rc!=0 or re!=0:
                rospy.logwarn("P2 read pres: %s / %s", self.ph2.getTxRxResult(rc), self.ph2.getRxPacketError(re))
            idx_p2 = self.dxl_ids.index(self.dxl_id_p2)
            res.insert(idx_p2, _i32_from_u32(val_u))
        return res


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
            if not self._port_open:
                rate.sleep()
                continue
            try:
                ticks = self.read_all_ticks()
                q_deg = self.ticks_to_deg(ticks)
                q_rad = [math.radians(x) for x in q_deg]
                js = JointState(); js.header.stamp = rospy.Time.now(); js.name = names; js.position = q_rad
                self.js_pub.publish(js)

                # ---- publish error status snapshot aligned to dxl_ids ----
                raw = UInt8MultiArray()
                raw.data = []
                for i, sid in enumerate(self.dxl_ids):
                    if sid == self.dxl_id_p2:
                        raw.data.append(int(self._last_p2_hwerr) & 0xFF)  # latched cause
                    else:
                        raw.data.append(int(self._last_p1_errors[i]) & 0xFF)
                self.err_pub_raw.publish(raw)

                if self.publish_error_text:
                    parts = []
                    for i, sid in enumerate(self.dxl_ids):
                        if sid == self.dxl_id_p2:
                            labels = decode_p2_hwerr_byte(self._last_p2_hwerr)
                            tag = f"ID{sid}(P2): {'|'.join(labels) if labels else 'OK'}"
                            if self._last_p2_alert:
                                tag += " [ALERT]"  # status Error bit7=1
                        else:
                            labels = decode_p1_error_byte(self._last_p1_errors[i])
                            tag = f"ID{sid}(P1): {'|'.join(labels) if labels else 'OK'}"
                        parts.append(tag)
                    self.err_pub_text.publish(String(data="; ".join(parts)))
                # ---- end error publish ----

                # NEW: only warn if any joint reports a fault; throttle to once per second
                if any(byte != 0 for byte in raw.data):
                    tag = "DXL fault"
                    if getattr(self, "_last_p2_alert", 0):
                        tag += " [P2 ALERT]"
                    rospy.logwarn_throttle(1.0, f"{tag}: " + "; ".join(parts))

                if self.print_reads:
                    if self.print_throttle_s <= 0:
                        rospy.loginfo("PRESENT ticks=%s | deg=[%.1f %.1f %.1f %.1f %.1f]", ticks, *[round(x,1) for x in q_deg])
                    else:
                        rospy.loginfo_throttle(self.print_throttle_s, "PRESENT ticks=%s | deg=[%.1f %.1f %.1f %.1f %.1f]",
                                            ticks, *[round(x,1) for x in q_deg])
            except Exception as e:
                rospy.logwarn_throttle(1.0, "Read loop warning: %s", str(e))
            rate.sleep()

    # -------- NEW TELEM LOOP --------
    def _telem_loop(self):
        rate_hz = self.telemetry_rate if self.telemetry_rate > 0 else 1.0
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown() and not self._stop.is_set():
            if not self._port_open:
                rate.sleep()
                continue
            try:
                # P1: contiguous 4B block (load, voltage, temperature)
                p1_rows = []
                with self.bus_lock:
                    for sid in self.dxl_ids:
                        if sid == self.dxl_id_p2: continue
                        buf, rc, re = self.ph1.readTxRx(self.port, sid, ADDR_RX_PRESENT_LOAD, 4)
                        if rc!=0 or re!=0 or not buf or len(buf)<4:
                            rospy.logwarn("P1[ID %d] telem read: %s / %s", sid, self.ph1.getTxRxResult(rc), self.ph1.getRxPacketError(re))
                            continue
                        load_raw = (buf[1] << 8) | buf[0]; volt_raw = buf[2]; temp_c = buf[3]
                        p1_rows.append((sid, load_raw, volt_raw, temp_c))

                # P2: one GroupBulkRead for current, voltage, temperature
                with self.bus_lock:
                    self.bulkread_p2.clearParam()
                    ok = self.bulkread_p2.addParam(self.dxl_id_p2, ADDR_PRO_PRESENT_CURRENT, 5)  # 621..625
                    rc = self.bulkread_p2.txRxPacket()
                    if rc != 0 or not ok:
                        rospy.logwarn("P2 bulk telem rc=%s ok=%s", self.ph2.getTxRxResult(rc), ok)
                    cur_raw = self.bulkread_p2.getData(self.dxl_id_p2, ADDR_PRO_PRESENT_CURRENT, 2)
                    vin_raw = self.bulkread_p2.getData(self.dxl_id_p2, ADDR_PRO_PRESENT_INPUT_VOLTAGE, 2)
                    t2_raw  = self.bulkread_p2.getData(self.dxl_id_p2, ADDR_PRO_PRESENT_TEMPERATURE, 1)

                # Publish Telem
                msg = ManipTelemetry()
                msg.header.stamp = rospy.Time.now()
                # P1 arrays aligned to P1 IDs (exclude Protocol 2 ID)
                p1_ids = [sid for sid in self.dxl_ids if sid != self.dxl_id_p2]
                msg.ids = p1_ids
                msg.p1_load_raw = [int(load) for (sid, load, v, t) in p1_rows]
                msg.p1_voltage_raw = [int(v) for (sid, load, v, t) in p1_rows]
                msg.p1_temp_c = [int(t) for (sid, load, v, t) in p1_rows]
                # P2 singles (cast current to int16)
                cur = int(cur_raw) & 0xFFFF
                if cur & 0x8000:
                    cur -= 0x10000
                msg.p2_current_raw = int(cur)
                msg.p2_voltage_in_raw = int(vin_raw) & 0xFFFF
                msg.p2_temp_c = int(t2_raw) & 0xFF
                self.telem_pub.publish(msg)
                
                # NEW: read Protocol 2 Hardware Error Status (latched until REBOOT/power cycle)
                with self.bus_lock:
                    hwerr, rc, re = self.ph2.read1ByteTxRx(self.port, self.dxl_id_p2, self.p2_hw_error_addr)
                if rc==0 and hwerr is not None:
                    self._last_p2_hwerr = int(hwerr) & 0xFF
                    if re != 0:
                        rospy.logwarn_throttle(1.0, "P2 HWERR read with status bits set: err=0x%02X", re)
                else:
                    rospy.logwarn_throttle(1.0, "P2 HW Error read rc=%s err=%s", self.ph2.getTxRxResult(rc), self.ph2.getRxPacketError(re))

                # optional throttle log
                # if self.print_reads:
                #     rospy.loginfo_throttle(1.0, f"P2 hwerr=0x{self._last_p2_hwerr:02X} alert={self._last_p2_alert} errnum=0x{self._last_p2_status_err:02X}")

                if p1_rows:
                    rospy.loginfo_throttle(1.0, "P1 telem: " + " | ".join([f"ID{sid} load={load} vol_raw={v} tempC={t}" for (sid, load, v, t) in p1_rows]))
                rospy.loginfo_throttle(1.0, f"P2 telem: ID{self.dxl_id_p2} cur_raw={cur_raw} volin_raw={vin_raw} tempC={t2_raw}")
            except Exception as e:
                rospy.logwarn_throttle(2.0, "Telemetry loop warning: %s", str(e))
            rate.sleep()



    # ---------- shutdown (disable torque with read-back & retries) ----------
    def shutdown(self):
        # make idempotent
        if getattr(self, "_shutting_down", False):
            return
        self._shutting_down = True

        # stop threads and join
        self._stop.set()
        try:
            try:
                if hasattr(self, "read_thread") and self.read_thread.is_alive():
                    self.read_thread.join(timeout=1.0)
            except Exception:
                pass
            try:
                if hasattr(self, "telem_thread") and self.telem_thread.is_alive():
                    self.telem_thread.join(timeout=1.0)
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
                            self.ph1.write1ByteTxOnly(self.port, sid, ADDR_RX_TORQUE_ENABLE, TORQUE_DISABLE)
                        with self.bus_lock:
                            val, r, e = self.ph1.read1ByteTxRx(self.port, sid, ADDR_RX_TORQUE_ENABLE)
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

    # Reopen port after shutdown
    def _reopen_and_reinit(self):
        """
        Re-open serial, set baud, re-enable torque, re-read pose.
        This is a lighter version of what __init__ did the first time.
        """
        # 1) open port
        with self.bus_lock:
            if not self.port.openPort():
                raise RuntimeError("Failed to reopen port")
            if not self.port.setBaudRate(self.baud_rate):
                raise RuntimeError("Failed to set baud on reopen")
        self._port_open = True
        rospy.loginfo("[RECONNECT] Port reopened @ %s baud=%s", self.port_name, self.baud_rate)

        # 2) torque ON again
        rospy.loginfo("[RECONNECT] Enabling torque again...")
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_RX_TORQUE_ENABLE, TORQUE_ENABLE)

        # 3) re-read current pose as the new "present" (don’t overwrite your original initial if you don’t want)
        try:
            ticks = self.read_all_ticks()
            deg   = self.ticks_to_deg(ticks)
            rospy.loginfo("[RECONNECT] Current ticks: %s", ticks)
            rospy.loginfo("[RECONNECT] Current deg  : [%.1f %.1f %.1f %.1f %.1f]", *[round(x,1) for x in deg])
            # you can choose to refresh:
            self.initial_ticks = ticks
            self.initial_deg   = deg
        except Exception as e:
            rospy.logwarn("Re-read after reconnect failed: %s", e)



    # def _sigint(self, *_):
    #     self._stop.set(); self.shutdown()
    #     try: rospy.signal_shutdown("SIGINT")
    #     finally: sys.exit(0)

def main():
    rospy.init_node("manip2_dxl_telem", anonymous=False)
    node = Manip2DxlBridge()
    rospy.loginfo("manip2_dxl_telem running. Publish Float64MultiArray to /manip2/command_deg.")
    rospy.spin()  # rely on rospy.on_shutdown to invoke node.shutdown()

if __name__ == "__main__":
    main()
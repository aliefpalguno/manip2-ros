#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, threading, signal, sys, ast
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
SPEED_P1_VALUE = 20
SPEED_P2_VALUE = 250
DEG_LIMITS = [[-90,90],[-30,36],[-110,0],[-75,75],[-90,90]]
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
ADDR_MX_RETURN_DELAY_TIME   = 5
ADDR_MX_CW_ANGLE_LIMIT      = 6
ADDR_MX_CCW_ANGLE_LIMIT     = 8
ADDR_MX_STATUS_RETURN_LEVEL = 16

ADDR_MX_SHUTDOWN            = 18
ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_MOVING_SPEED        = 32
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_LOAD        = 40
ADDR_MX_PRESENT_VOLTAGE     = 42
ADDR_MX_PRESENT_TEMPERATURE = 43


# P2 control table (signed int32 positions)
ADDR_PRO_RETURN_DELAY_TIME = 9
ADDR_PRO_OPERATING_MODE    = 11   # 3 = Position mode
ADDR_PRO_STATUS_RETURN_LEVEL = 891

ADDR_PRO_SHUTDOWN          = 48
ADDR_PRO_TORQUE_ENABLE     = 562
ADDR_PRO_GOAL_VELOCITY     = 600
ADDR_PRO_GOAL_POSITION     = 596
ADDR_PRO_PRESENT_POSITION  = 611
ADDR_PRO_PRESENT_INPUT_VOLTAGE = 623
ADDR_PRO_PRESENT_CURRENT   = 621
ADDR_PRO_PRESENT_TEMPERATURE = 625


class Manip2DxlBridge:
    def __init__(self):
        # Params
        self.port_name   = rospy.get_param("~port_name", DEFAULT_PORT)
        self.baud_rate   = int(rospy.get_param("~baud_rate", DEFAULT_BAUD))
        self.dxl_ids     = _to_list_ints(rospy.get_param("~dxl_ids", DEFAULT_IDS))
        self.dxl_id_p2   = int(rospy.get_param("~dxl_id_p2", DEFAULT_P2ID))
        self.speed_p1    = int(rospy.get_param("~speed_p1_value", SPEED_P1_VALUE))
        self.speed_p2    = int(rospy.get_param("~speed_p2_value", SPEED_P2_VALUE))
        self.read_rate   = float(rospy.get_param("~read_rate_hz", 30.0))
        self.print_reads = bool(rospy.get_param("~print_reads", True))

        self.deg_limits   = _to_list2_floats(rospy.get_param("~urdf_limits_deg", DEG_LIMITS))
        self.tick_offsets = _to_list_ints(rospy.get_param("~tick_offsets_nom", TICK_OFFSETS_NOM))
        self.degs_per_tick = _to_list_floats(rospy.get_param("~degs_per_tick_nom", DEGS_PER_TICK_NOM))
        self.rads_per_tick = [math.radians(dpt) for dpt in self.degs_per_tick]

        # State
        self._stop = threading.Event()
        self._port_open = False
        self.initial_ticks = None
        self.initial_deg   = None


        # SDK
        self.port = PortHandler(self.port_name)
        self.ph1  = PacketHandler(PROTOCOL_VERSION1)
        self.ph2  = PacketHandler(PROTOCOL_VERSION2)

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
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_STATUS_RETURN_LEVEL, 1)
                    p1_srl, dxl_comm_result, dxl_error = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_STATUS_RETURN_LEVEL)
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
                    self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_RETURN_DELAY_TIME, 0)
                p2_rdt, rc, er = self.ph2.read1ByteTxRx(self.port, sid, ADDR_PRO_RETURN_DELAY_TIME)
                if rc!=0 or er!=0:
                    rospy.logwarn("P2 return delay time val=%s comm=%s err=%s", p2_rdt, self.ph2.getTxRxResult(rc), self.ph2.getRxPacketError(er))
            else:
                with self.bus_lock:
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_RETURN_DELAY_TIME, 0)
                p1_rdt, rc, er = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_RETURN_DELAY_TIME)
                if rc!=0 or er!=0:
                    rospy.logwarn("P1[ID %d] return delay time val=%s comm=%s err=%s", sid, p1_rdt, self.ph1.getTxRxResult(rc), self.ph1.getRxPacketError(er))

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
                    self.ph1.write2ByteTxOnly(self.port, sid, ADDR_MX_MOVING_SPEED, self.speed_p1)
                    p1_mov_speed, dxl_comm_result, dxl_error = self.ph1.read2ByteTxRx(self.port, sid, ADDR_MX_MOVING_SPEED)
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
                    self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                    p1_torque, dxl_comm_result, dxl_error = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_TORQUE_ENABLE)
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

        # Reader thread
        # signal.signal(signal.SIGINT, self._sigint)
        
        self._shutting_down = False
        self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.read_thread.start()
        rospy.on_shutdown(self.shutdown)

    # ---------- callbacks ----------
    def on_command_deg(self, msg: Float64MultiArray):
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

    # ---------- main I/O ----------
    def send_degrees(self, q_deg):
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
            
        rospy.loginfo("Sent ticks (clamped): %s  (from deg %s)", ticksr, [round(x,1) for x in q_deg])

        # verify
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
                    goal, res, err = self.ph1.read2ByteTxRx(self.port, sid, ADDR_MX_GOAL_POSITION)
                if res!=0 or err!=0:
                    rospy.logerr("P1[ID %d] ReadGoal: %s / %s", sid, self.ph1.getTxRxResult(res), self.ph1.getRxPacketError(err))
                ver.append(f"ID {sid} goal_reg={goal} (sent {ticksr[i]})")
        rospy.loginfo("Write verify: " + " | ".join(ver))

    # ---------- reads ----------
    def read_all_ticks(self):
        res = []
        for sid in self.dxl_ids:
            if sid == self.dxl_id_p2:
                with self.bus_lock:
                    val_u, res_c, res_e = self.ph2.read4ByteTxRx(self.port, sid, ADDR_PRO_PRESENT_POSITION)
                if res_c!=0 or res_e!=0:
                    rospy.logwarn("P2 read pres: %s / %s", self.ph2.getTxRxResult(res_c), self.ph2.getRxPacketError(res_e))
                res.append(_i32_from_u32(val_u))
            else:
                with self.bus_lock:
                    val, res_c, res_e = self.ph1.read2ByteTxRx(self.port, sid, ADDR_MX_PRESENT_POSITION)
                if res_c!=0 or res_e!=0:
                    rospy.logwarn("P1[ID %d] read pres: %s / %s", sid, self.ph1.getTxRxResult(res_c), self.ph1.getRxPacketError(res_e))
                res.append(int(val) & 0xFFFF)
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
            if self._stop.is_set() or not self._port_open:
                rate.sleep()
                continue
            try:
                ticks = self.read_all_ticks()
                q_deg = self.ticks_to_deg(ticks)
                q_rad = [math.radians(x) for x in q_deg]

                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = names
                js.position = q_rad
                self.js_pub.publish(js)

                if self.print_reads:
                    rospy.loginfo_throttle(0.1, "PRESENT ticks=%s | deg=[%.1f %.1f %.1f %.1f %.1f]",
                                           ticks, *[round(x,1) for x in q_deg])
            except Exception as e:
                rospy.logwarn_throttle(2.0, "Read loop warning: %s", str(e))
            rate.sleep()

    # ---------- shutdown (disable torque with read-back & retries) ----------
    # def shutdown(self):
    #     try:
    #         rospy.loginfo("Disabling torque...")
    #         for sid in self.dxl_ids:
    #             if not self._port_open: break
    #             for attempt in range(3):
    #                 if sid == self.dxl_id_p2:
    #                     with self.bus_lock:
    #                         self.ph2.write1ByteTxOnly(self.port, sid, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #                         with self.bus_lock:
    #                             val, r, e = self.ph2.read1ByteTxRx(self.port, sid, ADDR_PRO_TORQUE_ENABLE)
    #                         if r!=0 or e!=0 or val!=0:
    #                             rospy.logwarn("P2 Disable Torque Failed! val=%s comm=%s, err=%s", val, r, e)
    #                         else: break
    #                 else:
    #                     with self.bus_lock:
    #                         self.ph1.write1ByteTxOnly(self.port, sid, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    #                         with self.bus_lock:
    #                             val, r, e = self.ph1.read1ByteTxRx(self.port, sid, ADDR_MX_TORQUE_ENABLE)
    #                         if r!=0 or e!=0 or val!=0:
    #                             rospy.logwarn("P1 [ID:%s] Disable Torque Failed! val=%s comm=%s, err=%s", sid, val, r, e)
    #                         else: break
    #                 rospy.sleep(0.03)
    #     finally:
    #         if self._port_open:
    #             try:
    #                 rospy.sleep(0.05)  # let bus flush
    #                 with self.bus_lock:
    #                     self.port.closePort()
    #                     self._port_open = False
    #                 rospy.loginfo("[OK] Port closed.")
    #             except Exception as e:
    #                 rospy.logwarn("Close port: %s", str(e))

    # def _sigint(self, *_):
    #     self._stop.set(); self.shutdown()
    #     try: rospy.signal_shutdown("SIGINT")
    #     finally: sys.exit(0)

    # replace shutdown with this
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
                    rospy.sleep(0.05)
                    with self.bus_lock:
                        self.port.closePort()
                    self._port_open = False
                    rospy.loginfo("[OK] Port closed.")
                except Exception as e:
                    rospy.logwarn("Close port: %s", str(e))

# def main():
#     rospy.init_node("manip2_dxl_bridge", anonymous=False)
#     node = Manip2DxlBridge()
#     rospy.loginfo("manip2_dxl_bridge running. Publish Float64MultiArray to /manip2/command_deg.")
#     try: rospy.spin()
#     except KeyboardInterrupt: pass
#     finally:
#         node._stop.set(); node.shutdown()

def main():
    rospy.init_node("manip2_dxl_bridge", anonymous=False)
    node = Manip2DxlBridge()
    rospy.loginfo("manip2_dxl_bridge running. Publish Float64MultiArray to /manip2/command_deg.")
    rospy.spin()  # rely on rospy.on_shutdown to invoke node.shutdown()

if __name__ == "__main__":
    main()

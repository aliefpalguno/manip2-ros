#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
manip2_command_gateway.py

Gateway to receive command intents from MQTT, log/ack, and actuate ROS.
- Subscribes: <mqtt_base>/cmd/#
- Publishes results: <mqtt_base>/cmd_result
- Publishes to ROS topic: /manip2/command_deg (std_msgs/Float64MultiArray)
- Publishes to ROS topic: /user/target_position (geometry_msgs/Point)  <-- NEW
- Calls services: /manip2/go_initial, /manip2/go_initial_and_shutdown,
                 /manip2/emergency_shutdown, /manip2/reconnect_port,
                 /manip2/safety/reset

Optional clamping: ~deg_limits as [[min,max], ...] (5x2), else no clamp.
"""
import json, time, threading
from typing import List, Tuple, Optional

import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerRequest
# Added Point for coordinate handling
from geometry_msgs.msg import Point

try:
    import paho.mqtt.client as mqtt
except Exception as e:
    mqtt = None

def now_ms() -> int:
    return int(time.time()*1000)

def clamp_vec(vals: List[float], limits: Optional[List[Tuple[float,float]]]) -> List[float]:
    if not limits: return vals
    out = []
    for i, v in enumerate(vals):
        try:
            lo, hi = limits[i]
            out.append(min(max(v, lo), hi))
        except Exception:
            out.append(v)
    return out

class Gateway:
    def __init__(self):
        rospy.init_node("manip2_command_gateway", anonymous=False)
        self.mqtt_host = rospy.get_param("~mqtt_host", "127.0.0.1")
        self.mqtt_port = int(rospy.get_param("~mqtt_port", 1883))
        self.mqtt_base = rospy.get_param("~mqtt_base", "robot/arm")
        self.mqtt_username = rospy.get_param("~mqtt_username", "aliefpal")
        self.mqtt_password = rospy.get_param("~mqtt_password", "Unpad2020")
        
        # optional joint limits in degrees
        self.deg_limits = rospy.get_param("~deg_limits", None)
        if self.deg_limits and len(self.deg_limits) != 5:
            rospy.logwarn("~deg_limits should have 5 pairs; ignoring")
            self.deg_limits = None

        # --- ROS Publishers ---
        # 1. Joint Angle Commands (for bridge/MATLAB bridge)
        self.pub_cmd = rospy.Publisher("/manip2/command_deg", Float64MultiArray, queue_size=1)
        
        # 2. Cartesian Target Commands (for MATLAB RL Agent)
        self.pub_target = rospy.Publisher("/user/target_position", Point, queue_size=1)

        # --- Service Proxies ---
        self.srv_go_initial = rospy.ServiceProxy("/manip2/go_initial", Trigger)
        self.srv_safe_shutdown = rospy.ServiceProxy("/manip2/go_initial_and_shutdown", Trigger)
        self.srv_force_shutdown = rospy.ServiceProxy("/manip2/emergency_shutdown", Trigger)
        self.srv_reconnect = rospy.ServiceProxy("/manip2/reconnect_port", Trigger)
        self.srv_safety_reset = rospy.ServiceProxy("/manip2/safety/reset", Trigger)

        # --- MQTT Setup ---
        if mqtt is None:
            raise RuntimeError("paho-mqtt not available")
        self.cli = mqtt.Client(client_id="cmd_gateway_%d"%now_ms(), clean_session=True)
        if self.mqtt_username:
                    self.cli.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.cli.on_connect = self._on_connect
        self.cli.on_message = self._on_message
        self.cli.connect_async(self.mqtt_host, self.mqtt_port, 60)
        self.cli.loop_start()

        rospy.loginfo("Gateway up: mqtt=%s:%d base=%s", self.mqtt_host, self.mqtt_port, self.mqtt_base)

    # MQTT wiring
    def _on_connect(self, client, userdata, flags, rc):
        topic = f"{self.mqtt_base}/cmd/#"
        client.subscribe(topic, qos=1)
        rospy.loginfo("Subscribed MQTT %s", topic)

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8','ignore')
            data = json.loads(payload) if payload else {}
        except Exception:
            data = {}

        # Normalize topic, e.g. "target/joint_deg", "go_initial", "target/xyz"
        cmd_topic = msg.topic.split(self.mqtt_base + "/cmd/", 1)[-1]

        # Handle dict / list / garbage robustly
        if isinstance(data, dict):
            command_id = data.get("command_id") or data.get("cid") or ""
        else:
            # e.g. plain list [0,30,-100,20,45] → no command_id but still valid for joints
            command_id = ""

        t0 = now_ms()
        status = "error"
        message = ""
        detail = {}

        try:
            # --- 1. Joint Angle Command ---
            if cmd_topic in ("target/joint_deg","target/joints"):
                joints = data if isinstance(data, list) else data.get("joints") or []
                if len(joints) != 5:
                    raise ValueError("need 5 joints in degrees")
                joints = [float(x) for x in joints]
                j_clamped = clamp_vec(joints, self.deg_limits)
                
                msg_ros = Float64MultiArray(data=j_clamped)
                self.pub_cmd.publish(msg_ros)
                
                status = "ok"
                message = "published /manip2/command_deg"
                detail = {"joints": joints, "joints_clamped": j_clamped}
            
            # --- 2. Cartesian Target Command (NEW) ---
            elif cmd_topic == "target/xyz":
                # Node-RED flow sends keys: x_cm, y_cm, z_cm
                x = float(data.get("x_cm", 0.0))
                y = float(data.get("y_cm", 0.0))
                z = float(data.get("z_cm", 0.0))
                
                p = Point()
                p.x = x
                p.y = y
                p.z = z
                self.pub_target.publish(p)
                
                status = "ok"
                message = f"published target /user/target_position: {x}, {y}, {z}"
                detail = {"x": x, "y": y, "z": z}

            # --- 3. System Commands ---
            elif cmd_topic == "go_initial":
                # REVERTED: Do NOT set lockout for normal home command
                resp = self.srv_go_initial(TriggerRequest())
                status = "ok" if resp.success else "fail"
                message = resp.message

            elif cmd_topic == "go_initial_and_shutdown" or cmd_topic == "safe_shutdown":
                # CRITICAL: Set Lockout IMMEDIATELY (Emergency intent)
                rospy.set_param("/manip2/lockout", True)
                resp = self.srv_safe_shutdown(TriggerRequest())
                status = "ok" if resp.success else "fail"
                message = resp.message

            elif cmd_topic == "shutdown" or cmd_topic == "emergency_shutdown":
                # CRITICAL: Set Lockout IMMEDIATELY (Emergency intent)
                rospy.set_param("/manip2/lockout", True)
                resp = self.srv_force_shutdown(TriggerRequest())
                status = "ok" if resp.success else "fail"
                message = resp.message

            elif cmd_topic == "reconnect_port":
                resp = self.srv_reconnect(TriggerRequest())
                status = "ok" if resp.success else "fail"
                message = resp.message
                
            elif cmd_topic == "safety/reset":
                # Reset clears the lockout
                resp = self.srv_safety_reset(TriggerRequest())
                # Also manually clear if service didn't (though supervisor should)
                rospy.set_param("/manip2/lockout", False)
                status = "ok" if resp.success else "fail"
                message = resp.message
            else:
                raise ValueError("unknown command topic: %s"%cmd_topic)
        except Exception as e:
            message = str(e)
            rospy.logwarn(f"Command error ({cmd_topic}): {message}")

        rospy.loginfo("MQTT cmd: topic=%s status=%s", msg.topic, status)

        # Emit result back to MQTT
        out = {
            "ts_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime())+"Z",
            "command_id": command_id,
            "cmd_topic": cmd_topic,
            "status": status,
            "message": message,
            "latency_ms": max(0, now_ms()-t0),
        }
        if detail: out["detail"] = detail
        try:
            self.cli.publish(f"{self.mqtt_base}/cmd_result", json.dumps(out), qos=1, retain=False)
        except Exception as e:
            rospy.logwarn("Failed to publish cmd_result: %s", e)

    def spin(self):
        rospy.spin()
        try:
            self.cli.loop_stop()
        except Exception:
            pass

if __name__ == "__main__":
    g = Gateway()
    g.spin()
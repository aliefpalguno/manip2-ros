#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <manip2_msgs/ManipTelemetry.h> 
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <sstream>

// ---------- Constants & Addresses ----------
// Protocol 1
const int ADDR_RX_RETURN_DELAY_TIME = 5;
const int ADDR_RX_CW_ANGLE_LIMIT = 6;
const int ADDR_RX_CCW_ANGLE_LIMIT = 8;
const int ADDR_RX_STATUS_RETURN_LEVEL = 16;
const int ADDR_RX_TORQUE_ENABLE = 24;
const int ADDR_RX_MOVING_SPEED = 32;
const int ADDR_RX_GOAL_POSITION = 30;
const int ADDR_RX_PRESENT_POSITION = 36;
const int ADDR_RX_PRESENT_LOAD = 40;
// Protocol 2
const int ADDR_PRO_RETURN_DELAY_TIME = 9;
const int ADDR_PRO_OPERATING_MODE = 11;
const int ADDR_PRO_STATUS_RETURN_LEVEL = 891;
const int ADDR_PRO_HARDWARE_ERROR_STATUS = 892;
const int ADDR_PRO_TORQUE_ENABLE = 562;
const int ADDR_PRO_GOAL_VELOCITY = 600;
const int ADDR_PRO_GOAL_POSITION = 596;
const int ADDR_PRO_PRESENT_POSITION = 611;
const int ADDR_PRO_PRESENT_CURRENT = 621;
const int ADDR_PRO_PRESENT_INPUT_VOLTAGE = 623;
const int ADDR_PRO_PRESENT_TEMPERATURE = 625;

const int TORQUE_ENABLE = 1;
const int TORQUE_DISABLE = 0;
const float PROTOCOL_VERSION1 = 1.0;
const float PROTOCOL_VERSION2 = 2.0;

// Limits (matching Python defaults)
const int32_t P2_MIN_I32 = -151875;
const int32_t P2_MAX_I32 = 151875;
const int J1_MAX = 1023;
const int J3_MAX = 4095;
const int J4_MAX = 1023;
const int J5_MAX = 1023;

// ---------- Helper: Parse String Lists from Launch File ----------
// Replaces Python's _ensure_list logic for strings like "[1, 2]"
std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<double> parseVector(std::string input) {
    // Remove brackets
    input.erase(std::remove(input.begin(), input.end(), '['), input.end());
    input.erase(std::remove(input.begin(), input.end(), ']'), input.end());
    std::vector<std::string> parts = split(input, ',');
    std::vector<double> res;
    for (const auto &p : parts) {
        if(!p.empty()) res.push_back(std::stod(p));
    }
    return res;
}

// ---------- Error Decoding ----------
std::string decode_p1_error_byte(uint8_t b) {
    std::vector<std::string> labels;
    if (b & 1) labels.push_back("Input Voltage");
    if (b & 2) labels.push_back("Angle Limit");
    if (b & 4) labels.push_back("Overheating");
    if (b & 8) labels.push_back("Range");
    if (b & 16) labels.push_back("Checksum");
    if (b & 32) labels.push_back("Overload");
    if (b & 64) labels.push_back("Instruction");
    
    if (labels.empty()) return ""; 
    std::string out = "";
    for(size_t i=0; i<labels.size(); i++) {
        out += labels[i] + (i<labels.size()-1 ? "|" : "");
    }
    return out;
}

std::string decode_p2_hwerr_byte(uint8_t b) {
    std::vector<std::string> labels;
    if (b & 1) labels.push_back("Input Voltage");
    if (b & 2) labels.push_back("Motor Hall Sensor");
    if (b & 4) labels.push_back("Overheating");
    if (b & 8) labels.push_back("Motor Encoder");
    if (b & 16) labels.push_back("Electrical Shock");
    if (b & 32) labels.push_back("Overload");

    if (labels.empty()) return "";
    std::string out = "";
    for(size_t i=0; i<labels.size(); i++) {
        out += labels[i] + (i<labels.size()-1 ? "|" : "");
    }
    return out;
}

// ---------- Main Class ----------
class Manip2DxlBridge {
public:
    Manip2DxlBridge(ros::NodeHandle nh, ros::NodeHandle nh_priv) 
        : nh_(nh), nh_priv_(nh_priv), 
          port_open_(false), stop_threads_(false), shutting_down_(false) 
    {
        // 1. Load Parameters (Strings are parsed to vectors to match launch file format)
        std::string s_ids, s_limits, s_offsets, s_degs;
        
        nh_priv_.param<std::string>("port_name", port_name_, "/dev/ttyUSB0");
        nh_priv_.param<int>("baud_rate", baud_rate_, 1000000);
        
        nh_priv_.param<std::string>("dxl_ids", s_ids, "[1,2,3,4,5]");
        std::vector<double> ids_d = parseVector(s_ids);
        for(double d : ids_d) dxl_ids_.push_back((int)d);

        nh_priv_.param<int>("dxl_id_p2", dxl_id_p2_, 2);
        nh_priv_.param<int>("speed_p1_value", speed_p1_, 20);
        nh_priv_.param<int>("speed_p2_value", speed_p2_, 250);

        nh_priv_.param<double>("read_rate_hz", read_rate_hz_, 60.0);
        nh_priv_.param<double>("telemetry_rate_hz", telem_rate_hz_, 2.0); // Updated default
        nh_priv_.param<bool>("print_reads", print_reads_, true);
        nh_priv_.param<double>("print_throttle_s", print_throttle_s_, 0.1);

        nh_priv_.param<std::string>("urdf_limits_deg", s_limits, "[[ -90, 90], ...]"); 
        // Parsing nested list limits [[min,max], [min,max]...]
        std::vector<double> flat_limits = parseVector(s_limits);
        for(size_t i=0; i<flat_limits.size(); i+=2) {
            if(i+1 < flat_limits.size()) 
                deg_limits_.push_back({flat_limits[i], flat_limits[i+1]});
        }

        nh_priv_.param<std::string>("tick_offsets_nom", s_offsets, "...");
        std::vector<double> off_d = parseVector(s_offsets);
        for(double d : off_d) tick_offsets_.push_back((int)d);

        nh_priv_.param<std::string>("degs_per_tick_nom", s_degs, "...");
        degs_per_tick_ = parseVector(s_degs);
        for(double d : degs_per_tick_) rads_per_tick_.push_back(d * M_PI / 180.0);

        nh_priv_.param<int>("p2_hw_error_addr", p2_hw_error_addr_, ADDR_PRO_HARDWARE_ERROR_STATUS);
        nh_priv_.param<bool>("publish_error_text", publish_error_text_, true);
        nh_priv_.param<double>("shutdown_after_initial_wait_s", shutdown_after_initial_wait_s_, 6.0);

        // State Init
        last_p1_errors_.resize(dxl_ids_.size(), 0);
        last_p2_alert_ = 0;
        last_p2_status_err_ = 0;
        last_p2_hwerr_ = 0;

        // SDK Setup
        portHandler_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packetHandler1_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
        packetHandler2_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
        
        groupSyncWriteP1_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler1_, ADDR_RX_GOAL_POSITION, 2);
        groupBulkReadP2_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler2_);

        // Initialize Hardware
        initializePort();

        // ROS Setup
        js_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
        telem_pub_ = nh_.advertise<manip2_msgs::ManipTelemetry>("/manip2/telemetry_raw", 1);
        err_pub_raw_ = nh_.advertise<std_msgs::UInt8MultiArray>("/manip2/error_status_raw", 1);
        err_pub_text_ = nh_.advertise<std_msgs::String>("/manip2/error_status_text", 1);

        cmd_sub_ = nh_.subscribe("/manip2/command_deg", 1, &Manip2DxlBridge::onCommandDeg, this);
        
        srv_home_ = nh_.advertiseService("/manip2/go_home", &Manip2DxlBridge::onGoInitial, this);
        srv_go_init_ = nh_.advertiseService("/manip2/go_initial", &Manip2DxlBridge::onGoInitial, this);
        srv_emerg_ = nh_.advertiseService("/manip2/emergency_shutdown", &Manip2DxlBridge::onEmergencyShutdown, this);
        srv_init_shut_ = nh_.advertiseService("/manip2/go_initial_and_shutdown", &Manip2DxlBridge::onGoInitialAndShutdown, this);
        srv_reconn_ = nh_.advertiseService("/manip2/reconnect_port", &Manip2DxlBridge::onReconnectPort, this);

        // Start threads
        startThreads();
    }

    ~Manip2DxlBridge() {
        shutdownHardware();
        delete groupSyncWriteP1_;
        delete groupBulkReadP2_;
        // PortHandler is handled by SDK, usually need to closePort
    }

    void spin() {
        ros::spin();
        shutdownHardware();
    }

private:
    // ROS & Params
    ros::NodeHandle nh_, nh_priv_;
    std::string port_name_;
    int baud_rate_;
    std::vector<int> dxl_ids_;
    int dxl_id_p2_;
    int speed_p1_, speed_p2_;
    double read_rate_hz_, telem_rate_hz_, print_throttle_s_, shutdown_after_initial_wait_s_;
    bool print_reads_, publish_error_text_;
    std::vector<std::vector<double>> deg_limits_;
    std::vector<int> tick_offsets_;
    std::vector<double> degs_per_tick_, rads_per_tick_;
    int p2_hw_error_addr_;

    // SDK
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler1_;
    dynamixel::PacketHandler *packetHandler2_;
    dynamixel::GroupSyncWrite *groupSyncWriteP1_;
    dynamixel::GroupBulkRead *groupBulkReadP2_;

    // State & Threading
    std::recursive_mutex bus_mutex_; // Replicates RLock
    std::atomic<bool> port_open_;
    std::atomic<bool> stop_threads_;
    bool shutting_down_;
    std::thread read_thread_, telem_thread_;

    std::vector<int32_t> initial_ticks_;
    std::vector<double> initial_deg_;
    
    // Error State
    std::vector<uint8_t> last_p1_errors_;
    uint8_t last_p2_alert_;
    uint8_t last_p2_status_err_;
    uint8_t last_p2_hwerr_;

    // ROS interfaces
    ros::Publisher js_pub_, telem_pub_, err_pub_raw_, err_pub_text_;
    ros::Subscriber cmd_sub_;
    ros::ServiceServer srv_home_, srv_go_init_, srv_emerg_, srv_init_shut_, srv_reconn_;

    // ---------------- Hardware Init ----------------
    void initializePort() {
        std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
        
        if (!portHandler_->openPort()) throw std::runtime_error("openPort failed");
        port_open_ = true;
        if (!portHandler_->setBaudRate(baud_rate_)) throw std::runtime_error("setBaudRate failed");
        ROS_INFO("[OK] Port open @ %s, %d baud", port_name_.c_str(), baud_rate_);

        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;

        // Set Status Return Level = 1
        ROS_INFO("Setting status return level to 1...");
        for (int sid : dxl_ids_) {
            if (sid == dxl_id_p2_) {
                packetHandler2_->write1ByteTxOnly(portHandler_, sid, ADDR_PRO_STATUS_RETURN_LEVEL, 1);
                // Read back verification skipped to keep code concise, matching python structure logic
            } else {
                packetHandler1_->write1ByteTxOnly(portHandler_, sid, ADDR_RX_STATUS_RETURN_LEVEL, 1);
            }
        }

        // Set P2 to Position Mode (3)
        uint8_t cur_mode;
        packetHandler2_->read1ByteTxRx(portHandler_, dxl_id_p2_, ADDR_PRO_OPERATING_MODE, &cur_mode);
        if (cur_mode != 3) {
             packetHandler2_->write1ByteTxOnly(portHandler_, dxl_id_p2_, ADDR_PRO_OPERATING_MODE, 3);
        }

        // Return Delay Time = 0
        ROS_INFO("Setting return delay time to 0...");
        for (int sid : dxl_ids_) {
            if (sid == dxl_id_p2_) packetHandler2_->write1ByteTxOnly(portHandler_, sid, ADDR_PRO_RETURN_DELAY_TIME, 0);
            else packetHandler1_->write1ByteTxOnly(portHandler_, sid, ADDR_RX_RETURN_DELAY_TIME, 0);
        }

        // Speeds
        ROS_INFO("Setting moving speeds...");
        for (int sid : dxl_ids_) {
            if (sid == dxl_id_p2_) packetHandler2_->write4ByteTxOnly(portHandler_, sid, ADDR_PRO_GOAL_VELOCITY, speed_p2_);
            else packetHandler1_->write2ByteTxOnly(portHandler_, sid, ADDR_RX_MOVING_SPEED, speed_p1_);
        }

        // Torque ON
        ROS_INFO("Enabling torque...");
        enableTorque(true);

        // Initial Read
        initial_ticks_ = readAllTicks();
        initial_deg_ = ticksToDeg(initial_ticks_);
        
        ROS_INFO("Initial ticks loaded.");
    }

    void enableTorque(bool enable) {
        int val = enable ? TORQUE_ENABLE : TORQUE_DISABLE;
        for (int sid : dxl_ids_) {
            if (sid == dxl_id_p2_) packetHandler2_->write1ByteTxOnly(portHandler_, sid, ADDR_PRO_TORQUE_ENABLE, val);
            else packetHandler1_->write1ByteTxOnly(portHandler_, sid, ADDR_RX_TORQUE_ENABLE, val);
        }
    }

    void startThreads() {
        stop_threads_ = false;
        read_thread_ = std::thread(&Manip2DxlBridge::readerLoop, this);
        telem_thread_ = std::thread(&Manip2DxlBridge::telemLoop, this);
    }

    // ---------------- Thread Loops ----------------
    void readerLoop() {
        ros::Rate rate(read_rate_hz_ > 0 ? read_rate_hz_ : 10.0);
        while (ros::ok() && !stop_threads_) {
            if (!port_open_) { rate.sleep(); continue; }
            try {
                std::vector<int32_t> ticks = readAllTicks();
                std::vector<double> q_deg = ticksToDeg(ticks);
                
                // Publish JointState
                sensor_msgs::JointState js;
                js.header.stamp = ros::Time::now();
                js.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
                for(double d : q_deg) js.position.push_back(d * M_PI / 180.0);
                js_pub_.publish(js);

                // Error Raw
                std_msgs::UInt8MultiArray raw_err;
                std::vector<std::string> parts;
                for(size_t i=0; i<dxl_ids_.size(); i++) {
                    int sid = dxl_ids_[i];
                    if (sid == dxl_id_p2_) {
                        raw_err.data.push_back(last_p2_hwerr_);
                        if(publish_error_text_) {
                            std::string lab = decode_p2_hwerr_byte(last_p2_hwerr_);
                            std::string tag = "ID" + std::to_string(sid) + "(P2): " + (lab.empty() ? "OK" : lab);
                            if (last_p2_alert_) tag += " [ALERT]";
                            parts.push_back(tag);
                        }
                    } else {
                        raw_err.data.push_back(last_p1_errors_[i]);
                        if(publish_error_text_) {
                            std::string lab = decode_p1_error_byte(last_p1_errors_[i]);
                            std::string tag = "ID" + std::to_string(sid) + "(P1): " + (lab.empty() ? "OK" : lab);
                            parts.push_back(tag);
                        }
                    }
                }
                err_pub_raw_.publish(raw_err);
                
                // Error Text
                if (publish_error_text_) {
                    std_msgs::String txt;
                    std::string joined;
                    for(size_t i=0; i<parts.size(); i++) joined += parts[i] + (i<parts.size()-1?"; ":"");
                    txt.data = joined;
                    err_pub_text_.publish(txt);
                }

                // Throttle Print
                // Note: Implementing precise throttle logic in C++ is verbose, simplifying to basic log
                // If you need exact throttle control similar to python's 0.1s, we can add a timer check.
                if (print_reads_) {
                    ROS_INFO_THROTTLE(print_throttle_s_, "PRESENT ticks=[%d, %d, %d, %d, %d] | deg=[%.1f, %.1f, %.1f, %.1f, %.1f]",
                        ticks[0], ticks[1], ticks[2], ticks[3], ticks[4],
                        q_deg[0], q_deg[1], q_deg[2], q_deg[3], q_deg[4]);
                }

            } catch (const std::exception &e) {
                ROS_WARN_THROTTLE(1.0, "Read loop warning: %s", e.what());
            }
            rate.sleep();
        }
    }

    void telemLoop() {
        ros::Rate rate(telem_rate_hz_ > 0 ? telem_rate_hz_ : 1.0);
        while (ros::ok() && !stop_threads_) {
            if (!port_open_) { rate.sleep(); continue; }
            try {
                manip2_msgs::ManipTelemetry msg;
                msg.header.stamp = ros::Time::now();

                // P1 Reads
                {
                    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
                    for (int sid : dxl_ids_) {
                        if (sid == dxl_id_p2_) continue;
                        uint8_t buf[4];
                        uint8_t dxl_err=0;
                        int res = packetHandler1_->readTxRx(portHandler_, sid, ADDR_RX_PRESENT_LOAD, 4, buf, &dxl_err);
                        if (res == COMM_SUCCESS && dxl_err == 0) {
                            int load = (buf[1] << 8) | buf[0];
                            msg.ids.push_back(sid);
                            msg.p1_load_raw.push_back(load);
                            msg.p1_voltage_raw.push_back(buf[2]);
                            msg.p1_temp_c.push_back(buf[3]);
                        }
                    }
                }

                // P2 Reads (Bulk)
                int32_t cur = 0; 
                uint16_t volt = 0; 
                uint8_t temp = 0;
                {
                    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
                    groupBulkReadP2_->clearParam();
                    groupBulkReadP2_->addParam(dxl_id_p2_, ADDR_PRO_PRESENT_CURRENT, 5); // 2(cur)+2(vol)+1(temp)
                    groupBulkReadP2_->txRxPacket();
                    
                    if (groupBulkReadP2_->isAvailable(dxl_id_p2_, ADDR_PRO_PRESENT_CURRENT, 2))
                        cur = (int16_t)groupBulkReadP2_->getData(dxl_id_p2_, ADDR_PRO_PRESENT_CURRENT, 2);
                    
                    if (groupBulkReadP2_->isAvailable(dxl_id_p2_, ADDR_PRO_PRESENT_INPUT_VOLTAGE, 2))
                        volt = (uint16_t)groupBulkReadP2_->getData(dxl_id_p2_, ADDR_PRO_PRESENT_INPUT_VOLTAGE, 2);

                    if (groupBulkReadP2_->isAvailable(dxl_id_p2_, ADDR_PRO_PRESENT_TEMPERATURE, 1))
                        temp = (uint8_t)groupBulkReadP2_->getData(dxl_id_p2_, ADDR_PRO_PRESENT_TEMPERATURE, 1);
                }
                msg.p2_current_raw = cur;
                msg.p2_voltage_in_raw = volt;
                msg.p2_temp_c = temp;
                telem_pub_.publish(msg);

                // P2 HW Error
                {
                    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
                    uint8_t hwerr = 0;
                    if(packetHandler2_->read1ByteTxRx(portHandler_, dxl_id_p2_, p2_hw_error_addr_, &hwerr) == COMM_SUCCESS) {
                        last_p2_hwerr_ = hwerr;
                    }
                }

                ROS_INFO_THROTTLE(1.0, "P2 telem: ID%d cur=%d vol=%d temp=%d", dxl_id_p2_, cur, volt, temp);

            } catch (const std::exception &e) {
                ROS_WARN_THROTTLE(2.0, "Telem loop warning: %s", e.what());
            }
            rate.sleep();
        }
    }

    // ---------------- Hardware Actions ----------------
    std::vector<int32_t> readAllTicks() {
        std::vector<int32_t> res;
        // P1
        {
            std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
            for(size_t i=0; i<dxl_ids_.size(); i++) {
                int sid = dxl_ids_[i];
                if(sid == dxl_id_p2_) continue;
                
                uint16_t val=0;
                uint8_t dxl_err=0;
                int r = packetHandler1_->read2ByteTxRx(portHandler_, sid, ADDR_RX_PRESENT_POSITION, &val, &dxl_err);
                
                last_p1_errors_[i] = dxl_err;
                if(r != COMM_SUCCESS) ROS_WARN("P1[ID %d] read fail: %d", sid, r);
                res.push_back((int32_t)val);
            }
        }
        // P2
        {
            std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
            uint32_t val_u = 0;
            uint8_t dxl_err = 0;
            int r = packetHandler2_->read4ByteTxRx(portHandler_, dxl_id_p2_, ADDR_PRO_PRESENT_POSITION, &val_u, &dxl_err);
            
            last_p2_alert_ = (dxl_err & 0x80) ? 1 : 0;
            last_p2_status_err_ = (dxl_err & 0x7F);
            
            int32_t val_i = (int32_t)val_u; // Assumes correct casting for P2 signed
            // Helper logic for P2 signed 32-bit conversion if SDK returns raw uint32
            if (val_u >= 0x80000000) val_i = (int32_t)val_u - (int64_t)0x100000000;
            else val_i = (int32_t)val_u;

            // Insert into correct position (index of ID2)
            for(size_t i=0; i<dxl_ids_.size(); i++) {
                if(dxl_ids_[i] == dxl_id_p2_) {
                    res.insert(res.begin() + i, val_i);
                    break;
                }
            }
        }
        return res;
    }

    void sendDegrees(std::vector<double> vals) {
        std::vector<int32_t> ticks;
        for(int i=0; i<5; i++) {
             double rad = vals[i] * M_PI / 180.0;
             double t = tick_offsets_[i] + (rad / rads_per_tick_[i]);
             ticks.push_back((int32_t)std::round(t));
        }
        
        // Clamp
        ticks[0] = std::max(0, std::min(J1_MAX, (int)ticks[0]));
        int p2_idx = -1;
        for(size_t i=0; i<dxl_ids_.size(); i++) if(dxl_ids_[i] == dxl_id_p2_) p2_idx = i;
        if(p2_idx >=0) ticks[p2_idx] = std::max(P2_MIN_I32, std::min(P2_MAX_I32, (int)ticks[p2_idx]));
        ticks[2] = std::max(0, std::min(J3_MAX, (int)ticks[2]));
        ticks[3] = std::max(0, std::min(J4_MAX, (int)ticks[3]));
        ticks[4] = std::max(0, std::min(J5_MAX, (int)ticks[4]));

        // P1 SyncWrite
        groupSyncWriteP1_->clearParam();
        for(size_t i=0; i<dxl_ids_.size(); i++) {
            int sid = dxl_ids_[i];
            if(sid == dxl_id_p2_) continue;
            uint8_t params[2];
            params[0] = DXL_LOBYTE(ticks[i]);
            params[1] = DXL_HIBYTE(ticks[i]);
            groupSyncWriteP1_->addParam(sid, params);
        }
        
        std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
        groupSyncWriteP1_->txPacket();
        
        // P2 Write
        packetHandler2_->write4ByteTxOnly(portHandler_, dxl_id_p2_, ADDR_PRO_GOAL_POSITION, (uint32_t)ticks[p2_idx]);
        
        ROS_INFO("Sent ticks: [%d %d %d %d %d]", ticks[0], ticks[1], ticks[2], ticks[3], ticks[4]);
    }

    std::vector<double> ticksToDeg(std::vector<int32_t> ticks) {
        std::vector<double> out;
        for(int i=0; i<5; i++) {
            double rad = (ticks[i] - tick_offsets_[i]) * rads_per_tick_[i];
            out.push_back(rad * 180.0 / M_PI);
        }
        return out;
    }

    // ---------------- Callbacks ----------------
    void onCommandDeg(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        bool lockout;
        if(nh_.getParam("/manip2/lockout", lockout) && lockout) {
             ROS_WARN_THROTTLE(1.0, "Safety lockout active");
             return;
        }
        if(msg->data.size() != 5) return;
        
        std::vector<double> vals = msg->data;
        for(int i=0; i<5; i++) {
             if(vals[i] < deg_limits_[i][0] || vals[i] > deg_limits_[i][1]) {
                 ROS_WARN("Joint %d clip %.1f to [%.1f, %.1f]", i, vals[i], deg_limits_[i][0], deg_limits_[i][1]);
                 vals[i] = std::max(deg_limits_[i][0], std::min(deg_limits_[i][1], vals[i]));
             }
        }
        sendDegrees(vals);
    }

    bool onGoInitial(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        if(initial_deg_.empty()) { res.success=false; res.message="No initial pose"; return true; }
        sendDegrees(initial_deg_);
        res.success=true; res.message="Sent initial pose";
        return true;
    }

    void shutdownHardware() {
        if (shutting_down_) return;
        shutting_down_ = true;
        stop_threads_ = true;
        
        if(read_thread_.joinable()) read_thread_.join();
        if(telem_thread_.joinable()) telem_thread_.join();

        ROS_INFO("Disabling torque...");
        if(port_open_) {
            std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
            enableTorque(false);
            ros::Duration(0.05).sleep();
            portHandler_->closePort();
            port_open_ = false;
        }
    }

    bool onEmergencyShutdown(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        std::thread([this]() {
            ROS_WARN("Emergency shutdown requested via service");
            this->shutdownHardware();
        }).detach();
        res.success = true;
        res.message = "Emergency shutdown started (node stays alive)";
        return true;
    }

    bool onGoInitialAndShutdown(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        std::thread([this]() {
            if(!initial_deg_.empty()) sendDegrees(initial_deg_);
            ros::Duration(shutdown_after_initial_wait_s_).sleep();
            ROS_WARN("Proceeding to shutdown...");
            this->shutdownHardware();
        }).detach();
        res.success = true;
        res.message = "Go-initial-and-shutdown started";
        return true;
    }

    bool onReconnectPort(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        if(port_open_) { res.success=true; res.message="Already open"; return true; }
        
        shutting_down_ = false;
        try {
            initializePort();
            startThreads();
            res.success=true; res.message="Reconnected";
        } catch(std::exception &e) {
            res.success=false; res.message=e.what();
        }
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "manip2_dxl_telem");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    Manip2DxlBridge node(nh, nh_priv);
    ROS_INFO("manip2_dxl_telem (C++) running.");
    node.spin();
    return 0;
}
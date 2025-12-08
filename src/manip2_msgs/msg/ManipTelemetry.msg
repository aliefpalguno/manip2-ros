# manip2_msgs/msg/ManipTelemetry.msg
std_msgs/Header header
uint8[] ids                 # P1 IDs order
uint16[] p1_load_raw        # per-id, Protocol 1 Present Load (bits 0..9 magnitude, 10 dir)
uint8[]  p1_voltage_raw     # per-id, Protocol 1 Present Voltage (0.1 V/LSB)
uint8[]  p1_temp_c          # per-id, °C
int16    p2_current_raw     # PRO 42 signed current (4.02832 mA/LSB)
uint16   p2_voltage_in_raw  # PRO input voltage (0.1 V/LSB)
uint8    p2_temp_c          # °C

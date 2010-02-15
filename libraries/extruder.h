#ifndef extruder_h
#define extruder_h

void init_tool(uint8_t i);
void select_tool(uint8_t tool);
void wait_for_tool_ready_state(uint8_t tool, int delay_millis, int timeout_seconds);
bool is_tool_ready(uint8_t tool);
bool send_packet();
bool read_tool_response(int timeout);
void print_stats();
void exercise_motors();
int get_tool_temp(uint8_t id);
void set_tool_temp(uint8_t id, uint8_t temp);
void set_motor1_pwm(uint8_t id, uint8_t pwm);
void set_motor2_pwm(uint8_t id, uint8_t pwm);
void toggle_motor1(uint8_t id, bool dir, bool enable);
void toggle_motor2(uint8_t id, bool dir, bool enable);
void toggle_fan(uint8_t id, bool enable);
void toggle_valve(uint8_t id, bool open);
void get_motor1_pwm(uint8_t id);
void get_motor2_pwm(uint8_t id);
void set_servo1_position(uint8_t id, uint8_t pos);
void set_servo2_position(uint8_t id, uint8_t pos);
void get_filament_status(uint8_t id);

#endif
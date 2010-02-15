extern "C" {
#include <avr/io.h>
#include <util/delay.h>
}
#include <HardwareSerial.h>
#include <SimplePacket.h>
#include <extruder.h>

#define PACKET_TIMEOUT 100

// These are our query commands from the host
#define HOST_CMD_VERSION         0
#define HOST_CMD_INIT            1
#define HOST_CMD_GET_BUFFER_SIZE 2
#define HOST_CMD_CLEAR_BUFFER    3
#define HOST_CMD_GET_POSITION    4
#define HOST_CMD_GET_RANGE       5
#define HOST_CMD_SET_RANGE       6
#define HOST_CMD_ABORT           7
#define HOST_CMD_PAUSE           8
#define HOST_CMD_PROBE           9
#define HOST_CMD_TOOL_QUERY     10

// These are our bufferable commands from the host
#define HOST_CMD_QUEUE_POINT_INC   128
#define HOST_CMD_QUEUE_POINT_ABS   129
#define HOST_CMD_SET_POSITION      130
#define HOST_CMD_FIND_AXES_MINIMUM 131
#define HOST_CMD_FIND_AXES_MAXIMUM 132
#define HOST_CMD_DELAY             133
#define HOST_CMD_CHANGE_TOOL       134
#define HOST_CMD_WAIT_FOR_TOOL     135
#define HOST_CMD_TOOL_COMMAND      136

// These are our query commands from the host
#define SLAVE_CMD_VERSION                0
#define SLAVE_CMD_INIT                   1
#define SLAVE_CMD_GET_TEMP               2
#define SLAVE_CMD_SET_TEMP               3
#define SLAVE_CMD_SET_MOTOR_1_PWM        4
#define SLAVE_CMD_SET_MOTOR_2_PWM        5
#define SLAVE_CMD_SET_MOTOR_1_RPM        6
#define SLAVE_CMD_SET_MOTOR_2_RPM        7
#define SLAVE_CMD_SET_MOTOR_1_DIR        8
#define SLAVE_CMD_SET_MOTOR_2_DIR        9
#define SLAVE_CMD_TOGGLE_MOTOR_1        10
#define SLAVE_CMD_TOGGLE_MOTOR_2        11
#define SLAVE_CMD_TOGGLE_FAN            12
#define SLAVE_CMD_TOGGLE_VALVE          13
#define SLAVE_CMD_SET_SERVO_1_POS       14
#define SLAVE_CMD_SET_SERVO_2_POS       15
#define SLAVE_CMD_FILAMENT_STATUS       16
#define SLAVE_CMD_GET_MOTOR_1_PWM       17
#define SLAVE_CMD_GET_MOTOR_2_PWM       18
#define SLAVE_CMD_GET_MOTOR_1_RPM       19
#define SLAVE_CMD_GET_MOTOR_2_RPM       20
#define SLAVE_CMD_SELECT_TOOL           21
#define SLAVE_CMD_IS_TOOL_READY         22


// Setup comms

void serial2_print(uint8_t d) {
  Serial2.print(d, BYTE);
}

SimplePacket slavePacket(serial2_print);

unsigned long rs485_tx_count = 0;
unsigned long rs485_rx_count = 0;
unsigned long rs485_packet_count = 0;
unsigned long rs485_loopback_fails = 0;
unsigned long slave_crc_errors = 0;
unsigned long slave_timeouts = 0;

unsigned long serial_tx_count = 0;
unsigned long serial_rx_count = 0;
unsigned long serial_packet_count = 0;


//initialize a tool to its default state.
void init_tool(uint8_t i)
{
  slavePacket.init();

  slavePacket.add_8(i);
  slavePacket.add_8(SLAVE_CMD_INIT);
  send_packet();
}

//select a tool as our current tool, and let it know.
void select_tool(uint8_t tool)
{
  slavePacket.init();

  slavePacket.add_8(tool);
  slavePacket.add_8(SLAVE_CMD_SELECT_TOOL);
  send_packet();
}

//ping the tool until it tells us its ready
void wait_for_tool_ready_state(uint8_t tool)
{
  uint16_t timeout_counter = 0;
  
  //do it until we hear something, or time out after one minute
  for(timeout_counter = 0; timeout_counter < 600; timeout_counter++)
  {
    //did we hear back from the tool?
    if (is_tool_ready(tool))
      return;

    //try again...
    _delay_ms(100);
  }
}

//is our tool ready for action?
bool is_tool_ready(uint8_t tool)
{
  slavePacket.init();

  slavePacket.add_8(tool);
  slavePacket.add_8(SLAVE_CMD_IS_TOOL_READY);

  //did we get a response?
  if (send_packet())
  {
    //is it true?
    if (slavePacket.get_8(1) == 1)
      return true;
  }

  //bail.
  return false;
}

bool send_packet()
{
  // //take it easy.  no stomping on each other.
  // _delay_ms(50);
  // 
  // digitalWrite(TX_ENABLE_PIN, HIGH); //enable tx
  // 
  // //take it easy.  no stomping on each other.
  // _delay_ms(10);

  slavePacket.sendPacket();
  // 
  // digitalWrite(TX_ENABLE_PIN, LOW); //disable tx
  // 
  rs485_packet_count++;

  return read_tool_response(PACKET_TIMEOUT);
}

bool read_tool_response(int timeout)
{
  long time = 0;

  //keep reading until we got it.
  while (!slavePacket.isFinished())
  {
    //read through our available data
    if (Serial2.available() > 0)
    {
      //grab a uint8_t and process it.
      uint8_t d = Serial2.read();
      slavePacket.process_byte(d);

      rs485_rx_count++;

      // Serial.print("IN:");
      //  Serial.print(d, HEX);
      //  Serial.print("/");
      //  Serial.println(d, BIN);
      // 
      //keep processing while there's data. 
      time = timeout;

      if (slavePacket.getResponseCode() == RC_CRC_MISMATCH)
      {
        slave_crc_errors++;

#ifdef ENABLE_COMMS_DEBUG
        Serial.println("Slave CRC Mismatch");
#endif
        //retransmit?
      }
    }

    //not sure if we need this yet.
    //our timeout guy.
    time -= 10;
    if (time <= 0)
    {
      slave_timeouts++;

#ifdef ENABLE_COMMS_DEBUG
      Serial.println("Slave Timeout");
#endif
      return false;
    }
  }

  return true;
}

void set_tool_pause_state(bool paused)
{
  //TODO: pause/unpause tool.
}

void exercise_heater()
{
  set_tool_temp(1, 100);
  while (1)
  {
    get_tool_temp(1);
    _delay_ms(500);
  }
}

void exercise_motors()
{
  bool dir = true;

  Serial.println("forward");
  Serial.println("up");
  for (int i=0; i<256; i++)
  {
    set_motor1_pwm(0, i);
    toggle_motor1(0, dir, 1);
    set_motor2_pwm(0, i);
    toggle_motor2(0, dir, 1);
	Serial.println(i, DEC);
  }

  Serial.println("down");
  for (int i=255; i>=0; i--)
  {
    set_motor1_pwm(0, i);
    toggle_motor1(0, dir, 1);
    set_motor2_pwm(0, i);
    toggle_motor2(0, dir, 1);
	Serial.println(i, DEC);
  }

  dir = false;

  Serial.println("forward");
  Serial.println("up");
  for (int i=0; i<256; i++)
  {
    set_motor1_pwm(0, i);
    toggle_motor1(0, dir, 1);
    set_motor2_pwm(0, i);
    toggle_motor2(0, dir, 1);
	Serial.println(i, DEC);
  }

  Serial.println("down");
  for (int i=255; i>=0; i--)
  {
    set_motor1_pwm(0, i);
    toggle_motor1(0, dir, 1);
    set_motor2_pwm(0, i);
    toggle_motor2(0, dir, 1);
	Serial.println(i, DEC);
  }
}

void print_stats()
{
  Serial.println("Stats:");
  Serial.print("Slave TX Count:");
  Serial.println(rs485_tx_count, DEC);
  Serial.print("Slave RX Count:");
  Serial.println(rs485_rx_count, DEC);
  Serial.print("Slave Packet Count: ");
  Serial.println(rs485_packet_count, DEC);
  Serial.print("Slave CRC Errors: ");
  Serial.println(slave_crc_errors, DEC);
  Serial.print("Slave timeouts: ");
  Serial.println(slave_timeouts, DEC);
}

void print_tool(uint8_t id)
{
  Serial.print("tool #");
  Serial.print(id, DEC);
  Serial.print(" ");
}

int get_tool_temp(uint8_t id)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_GET_TEMP);
  send_packet();

  int temp = slavePacket.get_16(1);
  return temp;
}

void set_tool_temp(uint8_t id, uint8_t temp)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_SET_TEMP);
  slavePacket.add_16(temp);
  send_packet();

  print_tool(id);
  Serial.print("set temp to: ");
  Serial.println(temp, DEC);
}

void set_motor1_pwm(uint8_t id, uint8_t pwm)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_SET_MOTOR_1_PWM);
  slavePacket.add_8(pwm);
  send_packet();

  //print_tool(id);
  //Serial.print("set motor1 pwm to: ");
  //Serial.println(pwm, DEC);
}

void set_motor2_pwm(uint8_t id, uint8_t pwm)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_SET_MOTOR_2_PWM);
  slavePacket.add_8(pwm);
  send_packet();

  //print_tool(id);
  //Serial.print("set motor2 pwm to: ");
  //Serial.println(pwm, DEC);
}


void set_motor1_rpm(uint8_t id, int rpm)
{

}

void toggle_motor1(uint8_t id, bool dir, bool enable)
{
  uint8_t flags = 0;

  if (enable)
    flags += 1;

  if (dir)
    flags += 2;

  Serial.println(flags, BIN);

  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_TOGGLE_MOTOR_1);
  slavePacket.add_8(flags);
  send_packet();
}

void toggle_motor2(uint8_t id, bool dir, bool enable)
{
  uint8_t flags = 0;

  if (enable)
    flags += 1;

  if (dir)
    flags += 2;

  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_TOGGLE_MOTOR_2);
  slavePacket.add_8(flags);
  send_packet();
}

void toggle_fan(uint8_t id, bool enable)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_TOGGLE_FAN);
  slavePacket.add_8(enable);
  send_packet();
}

void toggle_valve(uint8_t id, bool open)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_TOGGLE_VALVE);
  slavePacket.add_8(open);
  send_packet();
}

void get_motor1_pwm(uint8_t id)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_GET_MOTOR_1_PWM);
  send_packet();

  uint8_t temp = slavePacket.get_8(1);
  print_tool(id);
  Serial.print("m1 pwm: ");
  Serial.println(temp, DEC);
}

void get_motor2_pwm(uint8_t id)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_GET_MOTOR_2_PWM);
  send_packet();

  uint8_t temp = slavePacket.get_8(1);
  print_tool(id);
  Serial.print("m2 pwm: ");
  Serial.println(temp, DEC);
}

void set_servo1_position(uint8_t id, uint8_t pos)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_SET_SERVO_1_POS);
  slavePacket.add_8(pos);
  send_packet();
}

void set_servo2_position(uint8_t id, uint8_t pos)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_SET_SERVO_2_POS);
  slavePacket.add_8(pos);
  send_packet();
}

void get_motor1_rpm(uint8_t id)
{

}

void get_filament_status(uint8_t id)
{
  slavePacket.init();
  slavePacket.add_8(id);
  slavePacket.add_8(SLAVE_CMD_FILAMENT_STATUS);
  send_packet();

  uint8_t temp = slavePacket.get_8(1);
  print_tool(id);
  Serial.print("filament: ");
  Serial.println(temp, DEC);
}



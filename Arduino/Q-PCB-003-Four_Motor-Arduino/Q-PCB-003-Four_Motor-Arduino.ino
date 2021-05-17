// Tim Lubes
// Quantum Integration, 2021

#include <math.h>
#include <SparkFun_Tlc5940.h>
#include "Wire.h"

#define ANALOG_SPEED 1      //1 for Analog Speed; 0 for Full Speed
#define MIN_MOTOR_SPD 50   //Minium Speed(PWM) for motors
#define MAX_MOTOR_SPD 255   //Maximum Speed(PWM) for motors

// uncomment for debug information
// #define DEBUG

#ifdef DEBUG
  #define DEBUG_P(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_P(...)
  #define DEBUG_PLN(...)
#endif

// structure to hold received joystick data
struct {
  uint16_t joystick_1_x;  
  uint16_t joystick_1_y;         
  bool joystick_1_sw;
  uint16_t joystick_2_x;  
  uint16_t joystick_2_y;         
  bool joystick_2_sw;
  bool tactile_sw;         
} joystick_output;          

#define M_STOP      0
#define M_FORWARD   1
#define M_BACKWARD  2

#define CMD_stop              0
#define CMD_moveForward       1
#define CMD_moveBackward      2
#define CMD_moveLeft          3
#define CMD_moveRight         4
#define CMD_moveLeftForward   5
#define CMD_moveRightForward  6
#define CMD_moveLeftBackward  7
#define CMD_moveRightBackward 8
#define CMD_rotateLeft        9
#define CMD_rotateRight       10

#define CMD_TIMEOUT_MS  500

static uint8_t cur_cmd = CMD_stop, last_cmd = CMD_stop, cur_speed = 255, last_speed = 255;
static int cmd_last_update = 0, timeout_cnt = 0;

static bool data_available = false;

typedef enum{
  E_center = 0,
  E_up,
  E_down,
  E_left,
  E_right,
  E_upLeft,
  E_upRight,
  E_downLeft,
  E_downRight
} direction_t;

static direction_t joystick_direction;
#define JOYSTICK_CENTER_X 512       //ADC reading when X is centered
#define JOYSTICK_CENTER_Y 512       //ADC reading when Y is centered
#define JOYSTICK_CENTER_ADC_TOL 100 //ADC reading tolerance for center detection

#define JOYSTICK_MAX_RADIUS  sqrt(2*pow(JOYSTICK_CENTER_X,2))         //Max Radius
#define JOYSTICK_MIN_RADIUS  sqrt(2*pow(JOYSTICK_CENTER_ADC_TOL,2))   //Min Radius

class Motor {
  private:
    uint8_t port_1_p, port_2_p;
  public:
    Motor(uint8_t port_1, uint8_t port_2);
    void set_speed(uint8_t dir, uint8_t spd);
};

Motor::Motor(uint8_t port_1, uint8_t port_2) {
  port_1_p = port_1;
  port_2_p = port_2;
}

void Motor::set_speed (uint8_t dir, uint8_t spd) {
  if(spd<0||spd>255) return;
  uint16_t scaled_speed = 0;
  if(spd == 255){
    scaled_speed = 0xfff;
  }else{
    scaled_speed = spd * 16;
  }
  switch(dir)
  {
    case M_FORWARD:
      Tlc.set(port_1_p, scaled_speed);
      Tlc.set(port_2_p, 0);
      break;
    case M_BACKWARD:
      Tlc.set(port_1_p, 0);
      Tlc.set(port_2_p, scaled_speed);
      break;
   case M_STOP:
   default:
      Tlc.set(port_1_p, 0xfff);
      Tlc.set(port_2_p, 0xfff);
      break;
  }
}

Motor M_frontLeft(0, 1),     //Setup the Pins for four motors
      M_frontRight(2, 3),
      M_rearLeft(4, 5),
      M_rearRight(6, 7);

void receiveEvent(int count){

  if(count != sizeof(joystick_output)){
    DEBUG_PLN(F("WRONG I2C MESSAGE LENGTH"));
  }

  uint8_t *ptr = (uint8_t*) &joystick_output;

  while(Wire.available())
  {
    *ptr = Wire.read(); 
    ptr++;          
  }     

  data_available = true;  
}
      
void setup() {
  Serial.begin(115200);
  Wire.begin(0x22);
  Wire.onReceive(receiveEvent);
  Tlc.init();
  Tlc.clear();
  Tlc.update();
}

void loop() {
  if (data_available)
  {
    DEBUG_P(F("Packet Received"));
    DEBUG_P(F("X= "));
    DEBUG_P(joystick_output.joystick_1_x);
    DEBUG_P(F(" Y= "));
    DEBUG_P(joystick_output.joystick_1_y);
    if (joystick_output.joystick_1_sw == 1)
      DEBUG_P(F(" Switch 1 ON"));
    else
      DEBUG_P(F(" Switch 1 OFF"));
    DEBUG_P(F(" X= "));
    DEBUG_P(joystick_output.joystick_2_x);
    DEBUG_P(F(" Y= "));
    DEBUG_P(joystick_output.joystick_2_y);
    if (joystick_output.joystick_2_sw == 1)
      DEBUG_P(F(" Switch 2 ON"));
    else
      DEBUG_P(F(" Switch 2 OFF"));
    if (joystick_output.tactile_sw == 1)
      DEBUG_P(F(" Switch TAC ON"));
    else
      DEBUG_PLN(F(" Switch TAC OFF"));
 

    joystick_direction = xy_to_dir(joystick_output.joystick_1_x, joystick_output.joystick_1_y);
    if(joystick_direction == E_center){
      if(joystick_output.joystick_2_x > 662){
        cur_cmd = CMD_rotateRight;
        cur_speed = MAX_MOTOR_SPD;
      }else if(joystick_output.joystick_2_x < 362){
        cur_cmd = CMD_rotateLeft;    
        cur_speed = MAX_MOTOR_SPD;
      }else{
        cur_cmd = CMD_stop;
      }
    }else{
      cur_cmd = joystick_direction;
    }  
    cmd_handle();
    data_available = false;
  }
}

// Translate XY coordinate to Joystick Driection
direction_t xy_to_dir(int x_o, int y_o) { 
  long radius;
  float angle;
  int x_i = 1023 - x_o;
  int y_i = 1023 - y_o;
  long x = x_i - JOYSTICK_CENTER_X;
  long y = y_i - JOYSTICK_CENTER_Y;
  x = 0-x;
  if(x == 0 && y == 0 ){
    radius = 0;
    angle = 0;
  }
  else
  {
    radius = sqrt(y*y+x*x);
    angle = atan2(y,x)*180/PI;
  }
  if(angle<0) angle+=360;
  DEBUG_P("x:");
  DEBUG_P(x);
  DEBUG_P(", y:");
  DEBUG_P(y);
  DEBUG_P(" angle:");
  DEBUG_P(angle);
  DEBUG_P(", r:");
  DEBUG_PLN(radius);
  if(radius<sqrt(2*pow(JOYSTICK_CENTER_ADC_TOL,2))) return E_center;
  if(ANALOG_SPEED)
  {
    cur_speed = map(radius,JOYSTICK_MIN_RADIUS,JOYSTICK_MAX_RADIUS,MIN_MOTOR_SPD,255);
    DEBUG_P("speed:");
    DEBUG_PLN(cur_speed);
  }
  if(angle<0+22.5||angle>=360-22.5)
    return E_right;
  if(angle<45+22.5)
    return E_upRight;
  if(angle<90+22.5)
    return E_up;
  if(angle<135+22.5)
    return E_upLeft;
  if(angle<180+22.5)
    return E_left;
  if(angle<225+22.5)
    return E_downLeft;
  if(angle<270+22.5)
    return E_down;
  if(angle<315+22.5)
    return E_downRight;
}


// handle the received command
void cmd_handle(void) {
  if(cur_cmd==last_cmd && cur_speed==last_speed) return;
  switch(cur_cmd)
  {
    case CMD_stop:
      Stop();
      DEBUG_PLN("Stop");
      break;
    case CMD_moveForward:
      moveForward(cur_speed);
      DEBUG_PLN("moveForward");
      break;
    case CMD_moveBackward:
      moveBackward(cur_speed);
      DEBUG_PLN("moveBackward");
      break;
    case CMD_moveLeft:
      moveLeft(cur_speed);
      DEBUG_PLN("moveLeft");
      break;
    case CMD_moveRight:
      moveRight(cur_speed);
      DEBUG_PLN("moveRight");
      break;
    case CMD_moveLeftForward:
      moveLeftForward(cur_speed);
      DEBUG_PLN("moveLeftForward");
      break;
    case CMD_moveRightForward:
      moveRightForward(cur_speed);
      DEBUG_PLN("moveRightForward");
      break;
    case CMD_moveLeftBackward:
      moveLeftBackward(cur_speed);
      DEBUG_PLN("moveLeftBackward");
      break;
    case CMD_moveRightBackward:
      moveRightBackward(cur_speed);
      DEBUG_PLN("moveRightBackward");
      break;
    case CMD_rotateLeft:
      rotateLeft(cur_speed);
      DEBUG_PLN("rotateLeft");
      break;
    case CMD_rotateRight:
      rotateRight(cur_speed);
      DEBUG_PLN("rotateRight");
      break;
  }
  last_cmd = cur_cmd;  
  last_speed = cur_speed;  
}

void tlc_update(void){
  Tlc.update();
  Tlc.update();
}

void Stop(void) {
  M_frontLeft.set_speed(M_STOP,0);
  M_frontRight.set_speed(M_STOP,0);
  M_rearLeft.set_speed(M_STOP,0);
  M_rearRight.set_speed(M_STOP,0);
  tlc_update();
}
void moveForward(uint8_t spd) {
  M_frontLeft.set_speed(M_FORWARD,spd);
  M_frontRight.set_speed(M_FORWARD,spd);
  M_rearLeft.set_speed(M_FORWARD,spd);
  M_rearRight.set_speed(M_FORWARD,spd);
  tlc_update();
}
void moveBackward(uint8_t spd) {
  M_frontLeft.set_speed(M_BACKWARD,spd);
  M_frontRight.set_speed(M_BACKWARD,spd);
  M_rearLeft.set_speed(M_BACKWARD,spd);
  M_rearRight.set_speed(M_BACKWARD,spd);
  tlc_update();
}
void moveLeft(uint8_t spd) {
  M_frontLeft.set_speed(M_BACKWARD,spd);
  M_frontRight.set_speed(M_FORWARD,spd);
  M_rearLeft.set_speed(M_FORWARD,spd);
  M_rearRight.set_speed(M_BACKWARD,spd);
  tlc_update();
}
void moveRight(uint8_t spd) {
  M_frontLeft.set_speed(M_FORWARD,spd);
  M_frontRight.set_speed(M_BACKWARD,spd);
  M_rearLeft.set_speed(M_BACKWARD,spd);
  M_rearRight.set_speed(M_FORWARD,spd);
  tlc_update();
}
void moveLeftForward(uint8_t spd) {
  M_frontLeft.set_speed(M_STOP,0);
  M_frontRight.set_speed(M_FORWARD,spd);
  M_rearLeft.set_speed(M_FORWARD,spd);
  M_rearRight.set_speed(M_STOP,0);
  tlc_update();
}
void moveRightForward(uint8_t spd) {
  M_frontLeft.set_speed(M_FORWARD,spd);
  M_frontRight.set_speed(M_STOP,0);
  M_rearLeft.set_speed(M_STOP,0);
  M_rearRight.set_speed(M_FORWARD,spd);
  tlc_update();
}
void moveLeftBackward(uint8_t spd) {
  M_frontLeft.set_speed(M_BACKWARD,spd);
  M_frontRight.set_speed(M_STOP,0);
  M_rearLeft.set_speed(M_STOP,0);
  M_rearRight.set_speed(M_BACKWARD,spd);
  tlc_update();
}
void moveRightBackward(uint8_t spd) {
  M_frontLeft.set_speed(M_STOP,0);
  M_frontRight.set_speed(M_BACKWARD,spd);
  M_rearLeft.set_speed(M_BACKWARD,spd);
  M_rearRight.set_speed(M_STOP,0);
  tlc_update();
}
void rotateLeft(uint8_t spd) {
  M_frontLeft.set_speed(M_BACKWARD,spd);
  M_frontRight.set_speed(M_FORWARD,spd);
  M_rearLeft.set_speed(M_BACKWARD,spd);
  M_rearRight.set_speed(M_FORWARD,spd);
  tlc_update();
}
void rotateRight(uint8_t spd) {
  M_frontLeft.set_speed(M_FORWARD,spd);
  M_frontRight.set_speed(M_BACKWARD,spd);
  M_rearLeft.set_speed(M_FORWARD,spd);
  M_rearRight.set_speed(M_BACKWARD,spd);
  tlc_update();
}

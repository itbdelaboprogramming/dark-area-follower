/* [SERVO 1] Servo connection */
#define SRV1 3 // "PWM" on "SERVO 1" port

/* [SERVO 2] Servo connection */
#define SRV2 2 // "PWM" on "SERVO 2" port

/* [RC RECEIVER] Interface for 8 channel receiver */
#define RC_PWM1  A8 // "CH1" on "RC RECEIVER" port
#define RC_PWM2  A9 // "CH2" on "RC RECEIVER" port
#define RC_PWM3 A10 // "CH3" on "RC RECEIVER" port
#define RC_PWM4 A11 // "CH4" on "RC RECEIVER" port
#define RC_PWM5 A12 // "CH5" on "RC RECEIVER" port
#define RC_PWM6 A13 // "CH6" on "RC RECEIVER" port
#define RC_PWM7 A14 // "CH7" on "RC RECEIVER" port
#define RC_PWM8 A15 // "CH8" on "RC RECEIVER" port
#define RC_PWM_COUNT 8
#define RC_PWM_USE_COUNT 4

/* [FRONT LAMP] Interface for front lamp on/off */
#define LAMP_ON 27

/* [MOTOR DRIVER] Right motor driver (BTS7960) and encoder */
#define R_PWM1  4 // "RPWM" on "MOTOR DRIVER" port on the right side
#define L_PWM1  5 // "LPWM" on "MOTOR DRIVER" port on the right side
#define EN1     9 // "EN" on "MOTOR DRIVER" port on the right side
#define ENC_RA 13 // "CHA" on "RIGHT ENCODER" port after passed 100kHz LPF
#define ENC_RB 12 // "CHB" on "RIGHT ENCODER" port after passed 100kHz LPF

/* [MOTOR DRIVER] Left motor driver (BTS7960) and encoder */
#define R_PWM2  6 // "RPWM" on "MOTOR DRIVER" port on the left side
#define L_PWM2  7 // "LPWM" on "MOTOR DRIVER" port on the left side
#define EN2     8 // "EN" on "MOTOR DRIVER" port on the left side
#define ENC_LA 11 // "CHA" on "LEFT ENCODER" port after passed 100kHz LPF
#define ENC_LB 10 // "CHB" on "LEFT ENCODER" port after passed 100kHz LPF

/* [UART CH 1] Additional UART port, access with Serial1 */
#define TX1 18 // "TX1" on "UART CH 1" port
#define RX1 19 // "RX1" on "UART CH 1" port

/* [UART CH 2] Additional UART port, access with Serial2 */
#define TX2 16 // "TX2" on "UART CH 2" port
#define RX2 17 // "RX2" on "UART CH 2" port

/* [UART CH 3] Additional UART port, access with Serial3 */
#define TX3 14 // "TX3" on "UART CH 3" port
#define RX3 15 // "TX3" on "UART CH 3" port

/* [GENERAL I2C] General I2C port */
#define SDA 20 // "SDA" on "GENERAL I2C" port
#define SCL 21 // "SCL" on "GENERAL I2C" port

/* [GENERAL SPI] General SPI port */
#define MISO 50 // "MISO" on "GENERAL SPI" port
#define MOSI 51 // "MOSI" on "GENERAL SPI" port
#define SCK  52 // "SCK" on "GENERAL SPI" port
#define SS1  53 // "SS1" on "GENERAL SPI" port
#define SS2  24 // "SS2" on "GENERAL SPI" port
#define SS3  23 // "SS3" on "GENERAL SPI" port
#define SS4  22 // "SS4" on "GENERAL SPI" port

/* [ADC] Interface for ADC */
#define ADC1 A1 // "A1" on "ADC" port
#define ADC2 A2 // "A2" on "ADC" port
#define ADC3 A3 // "A3" on "ADC" port

/* [LED] Internal LED indicator */
#define BLUE_LED 31
#define RED_LED  30

/* [EEPROM AT24C32] Description for AT24C32 IC */
/* 1. AT24C32 communicate with I2C */
/* 2. Address of AT24C32 is 0x50 */
/* 3. Have write protection (WP); HIGH = DISABLE WRITE; LOW = ENABLE WRITE */
#define WP 25

/* [IMU MPU6050] Description for MPU6050 module */
/* 1. MPU6050 communicate with I2C */
/* 2. Address of MPU6050 is 0x69 */

/* [RTC DS3231] Description for DS3231 module */
/* 1. DS3231 communicate with I2C */
/* 2. Address of DS3231 is 0x68 */
/* 3. Module has internal EEPROM with address 0x57 */

/* [COMPASS QMC5883L] Description for QMC5883L module */
/* 1. QMC5883L communicate with I2C */
/* 2. Address of QMC5883L is 0x0D */
/* 3. Usually, IC marking is HMC, but it is actually QMC */

#include <PinChangeInterrupt.h>

#define UWB_SERIAL      Serial2
#define MT_MAX_PWM      200     // saturation PWM for action control (0-255)
#define DISARMED        0x00    // disarmed condition
#define ARMED           0x01    // armed condition

// CMD constant macro
#define STOP            0x01
#define FORWARD         0x02
#define RIGHT           0x03
#define LEFT            0x04
#define ROT_SPD_SCLR    0.80

// UWB transceiver constant macro
#define DISTANCE_THRD_U     1.2     // follower threshold distance in m
#define DISTANCE_THRD_L     0.8     // follower threshold distance in m
#define DEVIATION_THRD_U    0.8     // follower threshold deviation in m to start rotating
#define DEVIATION_THRD_L    0.5     // follower threshold deviation in m to start following
#define DEGREE_THRD         15.0    // follower threshold heading in degree

// FIR Filter Coeff for FS = 10 Hz at FC = 1 Hz
#define UWB_FIR_ORDER   4
const float UWB_FIR_COEFF[UWB_FIR_ORDER + 1] = {0.0284064700150113, 0.237008213590703, 0.469170632788571, 0.237008213590703, 0.0284064700150113};
// const float UWB_FIR_COEFF[UWB_FIR_ORDER + 1] = {1.0, 0.0, 0.0, 0.0, 0.0};

// data processing variables for uwb data filtering
uint8_t rotation_mode = DISARMED;
uint16_t uwb_counter = 0;
float distance_buffer[UWB_FIR_ORDER + 1];
float deviation_buffer[UWB_FIR_ORDER + 1];

// UWB signal variable
float distance = 0.0, deviation = 0.0, prev_deviation = 0.0;
float polar_heading = 0.0, polar_distance = 0.0;

// Receiver signal variables
uint16_t receiver_ch_value[9];    //PIN_CH_1 --> receiver_ch_value[1], and so on.
uint16_t receiver_ch_filtered[9]; //PIN_CH_1 --> receiver_ch_value[1], and so on.
uint32_t rc_timer[10];
uint32_t rc_input[5];
uint32_t uwb_lost_timer = 0;

// control variable
uint8_t  CMD = 0, failsafe = DISARMED;
int16_t  pwm_r = 0, pwm_l = 0;

void setup() {
  UWB_SERIAL.begin(115200);
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  pinMode(LAMP_ON, OUTPUT);
  digitalWrite(LAMP_ON, HIGH);

  init_rc();

  for(uint16_t i = 0; i <= UWB_FIR_ORDER; i++){
    distance_buffer[i] = 0;
    deviation_buffer[i]  = 0;
  }

  pinMode(EN2, OUTPUT);
  pinMode(R_PWM2, OUTPUT);
  pinMode(L_PWM2, OUTPUT);
  
  pinMode(EN1, OUTPUT);
  pinMode(R_PWM1, OUTPUT);
  pinMode(L_PWM1, OUTPUT);
}

void loop() {
  // Get all the necessary data and command
  getReceiverSignal();
  update_uwb_data();

  // Determine the action and then write to motor
  update_failsafe();
  update_cmd();
  write_motor();

  // enable debugging to serial monitor
  debug_all();
}

void init_rc(){
  attachPCINT(digitalPinToPCINT(RC_PWM1), RC1_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(RC_PWM2), RC2_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(RC_PWM3), RC3_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(RC_PWM4), RC4_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(RC_PWM5), RC5_ISR, CHANGE);
}

void getReceiverSignal() {
    // new algorithm
    receiver_ch_value[1] = rc_input[0];
    receiver_ch_value[2] = rc_input[1];
    receiver_ch_value[3] = rc_input[2];
    receiver_ch_value[4] = rc_input[3];
    receiver_ch_value[5] = rc_input[4];
}

void update_uwb_data(){
  if(UWB_SERIAL.available() > 0){
    uwb_lost_timer = millis();
    uint8_t temp = UWB_SERIAL.read();
    if(temp == 'X'){
      distance_buffer[uwb_counter] = UWB_SERIAL.parseFloat(); // filtered distance
      distance = 0.0;
      for(uint16_t i = 0; i <= UWB_FIR_ORDER; i++){
        distance += UWB_FIR_COEFF[i] * distance_buffer[((UWB_FIR_ORDER + 1) + uwb_counter - i) % (UWB_FIR_ORDER + 1)];
      }
      // distance = distance_buffer[uwb_counter];
      temp = UWB_SERIAL.read();
      if(temp == 'Y'){
        deviation_buffer[uwb_counter] = UWB_SERIAL.parseFloat();
        prev_deviation = deviation;
        deviation = 0.0;
        for(uint16_t i = 0; i <= UWB_FIR_ORDER; i++){
          deviation += UWB_FIR_COEFF[i] * deviation_buffer[((UWB_FIR_ORDER + 1) + uwb_counter - i) % (UWB_FIR_ORDER + 1)];
        }
        // deviation = deviation_buffer[uwb_counter];
        polar_distance  = sqrt(pow(distance,2) + pow(deviation,2));
        polar_heading   = atan2(deviation, distance) * 57.295779513;
        uwb_counter = (uwb_counter + 1) % (UWB_FIR_ORDER + 1);
      }
    }
  }
}

void update_failsafe(){
  // It is chosen because if RC is disconnected, the value is 0
  if(receiver_ch_value[4] > 1500){
    failsafe = ARMED;
  } else{
    failsafe = DISARMED;
  }
}

void update_cmd(){
  if(failsafe == DISARMED){ // Disarmed condition
    pwm_r = 0;
    pwm_l = 0;
    CMD = STOP;
  }else{ // Armed condition
    if(receiver_ch_value[3] < 1400){ // RC operation
      int16_t cmd_front_back = map(receiver_ch_value[1], 1000, 2000, -MT_MAX_PWM, MT_MAX_PWM);
      int16_t cmd_right_left = map(receiver_ch_value[2], 1000, 2000, -MT_MAX_PWM, MT_MAX_PWM);
      pwm_r = cmd_right_left - cmd_front_back;
      pwm_l = cmd_right_left + cmd_front_back;
    }else{ // automatic operation
      if(polar_distance < DISTANCE_THRD_L || millis() - uwb_lost_timer > 1000){ // if distance is too close or signal lost, then no movement is not allowed
        pwm_r = 0;
        pwm_l = 0;
        CMD = STOP;
      }else{
        if(rotation_mode == ARMED && (CMD == LEFT || CMD == RIGHT)){ // first phase is rotate to get the correct heading to the object
          // do the right rotation
          if(distance > 0.0){
            if(deviation > DEVIATION_THRD_U){ // turn left condition
              pwm_r = -ROT_SPD_SCLR * MT_MAX_PWM;
              pwm_l = -ROT_SPD_SCLR * MT_MAX_PWM;
              CMD = LEFT;
              rotation_mode = ARMED;
            }else if(fabs(deviation) > DEVIATION_THRD_U){ // turn right condition
              pwm_r = ROT_SPD_SCLR * MT_MAX_PWM;
              pwm_l = ROT_SPD_SCLR * MT_MAX_PWM;
              CMD = RIGHT;
              rotation_mode = ARMED;
            }
          }else{
            pwm_r = ROT_SPD_SCLR * MT_MAX_PWM;
            pwm_l = ROT_SPD_SCLR * MT_MAX_PWM;
            CMD = RIGHT;
          }
          
          // if zero cross is detected, then stop rotation and go to next phase
          if((prev_deviation > 0.0 && deviation <= 0.0) || (prev_deviation < 0.0 && deviation >= 0.0)){
            rotation_mode = DISARMED;
          }
        }else{ // movement phase after rotation phase is done
          if(distance > 0.0){
            if((polar_distance > DISTANCE_THRD_U && fabs(deviation) < DEVIATION_THRD_L) || fabs(polar_heading) < DEGREE_THRD){ // move forward condition
              pwm_r = -MT_MAX_PWM;
              pwm_l =  MT_MAX_PWM;
              CMD = FORWARD;
            }else if(deviation > DEVIATION_THRD_U){ // turn left condition
              pwm_r = -ROT_SPD_SCLR * MT_MAX_PWM;
              pwm_l = -ROT_SPD_SCLR * MT_MAX_PWM;
              CMD = LEFT;
              rotation_mode = ARMED;
            }else if(fabs(deviation) > DEVIATION_THRD_U){
              pwm_r = ROT_SPD_SCLR * MT_MAX_PWM;
              pwm_l = ROT_SPD_SCLR * MT_MAX_PWM;
              CMD = RIGHT;
              rotation_mode = ARMED;
            }
          }else{
            pwm_r = ROT_SPD_SCLR * MT_MAX_PWM;
            pwm_l = ROT_SPD_SCLR * MT_MAX_PWM;
            CMD = RIGHT;
          }
        }
      }
    }
  }
}

void write_motor(){
  // Rotate right motor
  if(pwm_r == 0){
    digitalWrite(EN1, LOW);
  }else if(pwm_r > 0){
    digitalWrite(EN1, HIGH);
    analogWrite(R_PWM1, 0);
    analogWrite(L_PWM1, pwm_r);
  }else{
    digitalWrite(EN1, HIGH);
    analogWrite(L_PWM1, 0);
    analogWrite(R_PWM1, -pwm_r);
  }
  
  // Rotate left motor
  if(pwm_l == 0){
    digitalWrite(EN2, LOW);
  }else if(pwm_l > 0){
    digitalWrite(EN2, HIGH);
    analogWrite(R_PWM2, 0);
    analogWrite(L_PWM2, pwm_l);
  }else{
    digitalWrite(EN2, HIGH);
    analogWrite(L_PWM2, 0);
    analogWrite(R_PWM2, -pwm_l);
  }
}

void debug_all(){
  for(byte i = 1; i <= RC_PWM_USE_COUNT; i++){
    Serial.print("CH ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(receiver_ch_value[i]);
    Serial.print("\t");
  }
  if(failsafe == ARMED){
    Serial.print("Status: ARMED\t");
  }else{
    Serial.print("Status: DISARMED\t");
  }
  Serial.print("RMOTOR: ");
  Serial.print(pwm_r);
  Serial.print("\tLMOTOR: ");
  Serial.print(pwm_l);
  Serial.print("\tDEVIATION:  ");
  Serial.print(deviation);
  Serial.print("\tDISTANCE:  ");
  Serial.print(distance);
  Serial.print("\tRHO: ");
  Serial.print(polar_distance);
  Serial.print("\tTHETA: ");
  Serial.print(polar_heading);
  Serial.print("\tCMD: ");
  if(CMD == 1){
    Serial.print("STOP");
  }else if(CMD == 2){
    Serial.print("FORWARD");
  }else if(CMD == 3){
    Serial.print("RIGHT");
  }else if(CMD == 4){
    Serial.print("LEFT");
  }
  Serial.println();
}

void RC1_ISR(void){
  if(PINK & 0B00000001){
    rc_timer[0] = micros();
  }else{
    rc_timer[1] = micros();
    if(rc_timer[1] > rc_timer[0]) rc_input[0] = rc_timer[1] - rc_timer[0];
  }
}

void RC2_ISR(void){
  if(PINK & 0B00000010){
    rc_timer[2] = micros();
  }else{
    rc_timer[3] = micros();
    if(rc_timer[3] > rc_timer[2]) rc_input[1] = rc_timer[3] - rc_timer[2];
  }
}

void RC3_ISR(void){
  if(PINK & 0B00000100){
    rc_timer[4] = micros();
  }else{
    rc_timer[5] = micros();
    if(rc_timer[5] > rc_timer[4]) rc_input[2] = rc_timer[5] - rc_timer[4];
  }
}

void RC4_ISR(void){
  if(PINK & 0B00001000){
    rc_timer[6] = micros();
  }else{
    rc_timer[7] = micros();
    if(rc_timer[7] > rc_timer[6]) rc_input[3] = rc_timer[7] - rc_timer[6];
  }
}

void RC5_ISR(void){
  if(PINK & 0B00010000){
    rc_timer[8] = micros();
  }else{
    rc_timer[9] = micros();
    if(rc_timer[9] > rc_timer[8]) rc_input[4] = rc_timer[9] - rc_timer[8];
  }
}
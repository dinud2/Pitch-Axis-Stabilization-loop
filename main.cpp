#include <Arduino.h>
#include <Wire.h>

const int MOTOR1_PIN = 25;
const int MOTOR2_PIN = 26;
const int ARM_BTN_PIN = 4; // BUTTON TO GND
const int SDA_PIN = 27, SCL_PIN = 22;

// Set to 1 only once a button is actually wired GPIO4 -> GND.
// With no button, INPUT_PULLUP reads HIGH forever and nothing ever arms.
#define USE_ARM_BUTTON 0

// PWM -?
const int PWM_freq = 20000;
const int PWM_RES = 10;
const int PWM_MAX = (1 << PWM_RES) - 1;
// Lowest duty at which the motors reliably keep turning. Find it with the 'm' raw
// test command and set this a little above where they stall.
const int MOTOR_MIN = 120;

// MPU6050
const uint8_t MPU_ADDR = 0x68;
const float ACC_LSB = 16384.0f;
const float GYRO_LSB = 131.0f;
float gyroBiasY = 0.0f;

// complementary filter
float pitch = 0.0f;
const float ALPHA = 0.98f;
unsigned long lastMicros = 0;

// PID
float Kp = 8.0f, Ki = 0.0f, Kd = 2.0f; // add Kd first to damp, Ki last
float setPoint = 0.0f; // target pitch (deg)
float integral = 0.0f;
int mixSign = 1; // which motor gets +output; flip with 'x-1' if corrections run away
const float I_LIMIT = 150.0f; 
const int OUT_LIMIT = 350; // max correction (duty)
int baseThrottle = 300; // both motors idle
bool armed = false;
bool softArm = false;   // set by serial 'a1' / 'a0'
int  testDuty = -1;     // >=0 bypasses the PID and drives both motors raw ('m' cmd)


void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false); // keeps communication line active (restart msg sent)
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  uint8_t b[14];
  for(int i = 0; i< 14; i++){
    b[i] = Wire.read();
  }
  ax = (int16_t)((b[0] << 8) | b[1]); // combining two bytes into one 16 bith value
  ay = (int16_t)((b[2] << 8) | b[3]); 
  az = (int16_t)((b[4] << 8) | b[5]); 

  gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
}

void calibrateGyro(){
  long sy = 0;
  const int N = 1000;
  int16_t ax, ay, az, gx, gy, gz;
  for(int i = 0; i< N; i++){
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sy += gy;
    delay(2);
  }
  gyroBiasY = sy / (float)N;
}

// Tuning
void applyCommand(char c, float v){
  switch (c) {
    case 'p':
      Kp = v;
      break;
    case 'i':
      Ki = v; 
      integral = 0;
      break;
    case 'd': 
      Kd = v;
      break;
    case 's':
      setPoint = v;
      break;
    case 'b':
      baseThrottle = (int)v;
      break;
    case 'a': // a1 = arm, a0 = disarm
      softArm = (v != 0.0f);
      integral = 0;
      break;
    case 'x': // x1 / x-1 : flip which motor gets +output
      mixSign = (v < 0) ? -1 : 1;
      break;
    case 'm': // m<duty> raw duty on both motors, bypasses PID. m-1 exits test mode.
      testDuty = (int)v;
      if (testDuty > PWM_MAX) testDuty = PWM_MAX;
      Serial.printf("# TEST duty=%d / %d\n", testDuty, PWM_MAX);
      return;
    default:
      return;
  }

  Serial.printf("# Kp=%.2f Ki=%.2f Kd=%.2f set=%.1f base=%d mix=%+d armed=%d\n",
                Kp, Ki, Kd, setPoint, baseThrottle, mixSign, (int)softArm);
}

// Line-buffered, non-blocking. parseFloat() was returning 0 whenever the monitor
// sent characters as they were typed instead of a whole line on Enter.
void handleSerial(){
  static char buf[32];
  static uint8_t n = 0;
  static unsigned long lastChar = 0;

  while(Serial.available()){
    char c = Serial.read();
    lastChar = millis();
    if(c == '\n' || c == '\r'){
      if(n == 0) continue; // swallow the second byte of a CRLF pair
      buf[n] = '\0';
      Serial.printf("# rx \"%s\"\n", buf);
      applyCommand(buf[0], atof(buf + 1));
      n = 0;
    } else if(n < sizeof(buf) - 1){
      buf[n++] = c;
    }
  }

  // Fallback for monitors that send no line ending at all: if input has gone
  // quiet for 200 ms, treat what we have as a complete command.
  if(n > 0 && millis() - lastChar > 200){
    buf[n] = '\0';
    Serial.printf("# rx \"%s\" (timeout)\n", buf);
    applyCommand(buf[0], atof(buf + 1));
    n = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ARM_BTN_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuWrite(0x6b, 0x00); 
  delay(100);
  calibrateGyro();

  bool ok1 = ledcAttach(MOTOR1_PIN, PWM_freq, PWM_RES);
  bool ok2 = ledcAttach(MOTOR2_PIN, PWM_freq, PWM_RES);
  Serial.printf("# ledcAttach m1=%d m2=%d  freq=%d res=%d max=%d\n",
                (int)ok1, (int)ok2, PWM_freq, PWM_RES, PWM_MAX);
  Serial.println("# cmds: a1/a0 arm, m<duty> raw motor test (m-1 off), x1/x-1 mix sign, p/i/d/s/b tuning");
  ledcWrite(MOTOR1_PIN, 0);
  ledcWrite(MOTOR2_PIN, 0);

  lastMicros = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt< 0.002f){
    return;
  }
  lastMicros = now;

#if USE_ARM_BUTTON
  armed = (digitalRead(ARM_BTN_PIN) == LOW) || softArm;
#else
  armed = softArm;
#endif
  // read IMU
  int16_t ax, ay,az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);
  float accY = ay / ACC_LSB;
  float accZ = az / ACC_LSB;

  float pitchAcc = atan2f(accY, accZ) * 57.29578f;
  float gyroRate = (gy - gyroBiasY) / GYRO_LSB;

  pitch = ALPHA * (pitch + gyroRate * dt) + (1.0f - ALPHA) *pitchAcc;

  // PID
  float error = setPoint - pitch;
  integral +=error * dt;
  if(integral > I_LIMIT){
    integral = I_LIMIT;
  }
  if(integral < -I_LIMIT){
    integral = -I_LIMIT;
  }
  // Derivative on measurement: d(error)/dt == -gyroRate for a fixed setpoint, and the
  // raw gyro is far quieter than differencing the filtered pitch at ~500 Hz. Also kills
  // the derivative kick when 's' moves the setpoint.
  float deriv = -gyroRate;
  float output = Kp * error + Ki * integral + Kd * deriv;
  if(output > OUT_LIMIT){
    output = OUT_LIMIT;
  }
  if(output < -OUT_LIMIT){
    output = -OUT_LIMIT;
  }

  // allow both motors to mix
  int m1 =0, m2 = 0;
  if(testDuty >= 0){
    // raw wiring test: PID out of the picture entirely
    m1 = m2 = testDuty;
  } else if(armed){
    // Differential mix: one end of the beam speeds up as the other slows down, which is
    // what actually produces a pitching moment. Clamp the correction to the headroom
    // that exists on both sides so the pair stays symmetric about baseThrottle instead
    // of one motor bottoming out.
    int headroom = min(baseThrottle - MOTOR_MIN, PWM_MAX - baseThrottle);
    int corr = constrain(mixSign * (int)output, -headroom, headroom);
    m1 = constrain(baseThrottle + corr, MOTOR_MIN, PWM_MAX);
    m2 = constrain(baseThrottle - corr, MOTOR_MIN, PWM_MAX);
  } else {
    integral = 0;
  }
  ledcWrite(MOTOR1_PIN, m1);
  ledcWrite(MOTOR2_PIN, m2);

  // telemertry for serial plotter
  static unsigned long lastLog = 0;
  if(millis() - lastLog >= 20){
    lastLog = millis();
    Serial.printf("%.1f\t%.2f\t%.1f\t%d\t%d\t%d\n",
                  setPoint, pitch, output, (int)armed, m1, m2);
  }

  handleSerial();
}
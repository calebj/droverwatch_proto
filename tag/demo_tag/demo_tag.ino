// -*- mode: C++ -*-

#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>

#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
  
/* for feather32u4 */
#define VBATPIN A9
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define RF95_FREQ 915.0

#define USE_SERIAL 0
#define USE_LPF 1

#define SAMPLE_COUNT 10

const PROGMEM uint16_t serial_tag[] = {1, 18};

// Filter and reference parameters
const float LPF_ALPHA = 0.05;
const float COMP_RATIO = 0.07;
const double accel_g = 9.80665;
const double ACCEL_THRESH = 0.15;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton instance of the IMU sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Sensor reading destination variables
sensors_event_t a, m, g, temp;
double ax_f, ay_f, az_f;

// Calculated angles using a complementary filter
double compAngleX, compAngleY;

// packet counter, incremented per transmission
uint32_t packetnum = 0;

// time of last sensor reading, used to calculate dt
uint32_t timer;
uint32_t blink_on;
int ledState = LOW;

typedef struct {
  double roll, pitch, accel_mag;
} roll_pitch_t;

typedef struct {
  double ax, ay, az, wx, wy;
  uint16_t dt_us; // shifted >> 2 for a max of ~0.262s
} imu_sample_t;

// buffer for samples to send
imu_sample_t sample_buf[SAMPLE_COUNT];

typedef uint8_t ORIENTATION_T;
static const ORIENTATION_T OR_UNKNOWN = 0;
static const ORIENTATION_T OR_UPRIGHT = 1;
static const ORIENTATION_T OR_INVERTED = 2;
static const ORIENTATION_T OR_LEFT = 3;
static const ORIENTATION_T OR_RIGHT = 4;
static const ORIENTATION_T OR_TOP = 5;
static const ORIENTATION_T OR_BOTTOM = 6;

// forward declaration
float measureBattery();

class tx_packet_t {
public:
  const uint8_t version = 2;
  const uint16_t serial, tag;
  const uint32_t packet_seq;
  ORIENTATION_T orientation = OR_UNKNOWN;
  uint16_t chain = 0;
  double angle_x, angle_y, displacement;
  const double vbatt;

  const uint8_t sample_count = SAMPLE_COUNT;
  imu_sample_t samples[SAMPLE_COUNT];

  tx_packet_t() :
    serial(pgm_read_word_near(&serial_tag[0])),
    tag(pgm_read_word_near(&serial_tag[1])),
    packet_seq(++packetnum),
    vbatt(measureBattery()) {};
};

uint32_t loop_i = 0;
double v_mag = 0;
double a_last = 0;
double v_last = 0;
double displacement = 0;
uint16_t chain;
ORIENTATION_T orientation, last_orientation = OR_UNKNOWN;

void update_imu() {
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp);
}

inline double hyp2(const double& x, const double& y) {
  return sqrt(x*x + y*y);
}

inline double hyp3(const double& x, const double& y, const double& z) {
  return sqrt(x*x + y*y + z*z);
}

void update_filtered_accel() {
  // y[i] := y[i-1] + a*(y[i] - y[i-1])
#if USE_LPF
  ax_f += LPF_ALPHA * (a.acceleration.x - ax_f);
  ay_f += LPF_ALPHA * (a.acceleration.y - ay_f);
  az_f += LPF_ALPHA * (a.acceleration.z - az_f);
#else
  ax_f = a.acceleration.x;
  ay_f = a.acceleration.y;
  az_f = a.acceleration.z;
#endif
}

roll_pitch_t read_accel_angles() {
  roll_pitch_t ret;

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
  //double xx_zz = ax_f*ax_f + az_f*az_f;
  //double yy_zz = ay_f*ay_f + az_f*az_f;

  // Invert Z axis because the accelerometer is mounted upside down.
  //ret.roll  = atan2(-ax_f, sqrt(yy_zz)) * RAD_TO_DEG;
  //ret.pitch = atan(-ay_f/az_f) * RAD_TO_DEG;
  //ret.roll  = atan2(ay_f, az_f) * RAD_TO_DEG;
  //ret.pitch = atan(-ax_f / sqrt(yy_zz)) * RAD_TO_DEG;
  ret.roll  = atan(ay_f / hyp2(ax_f, az_f)) * RAD_TO_DEG;
  ret.pitch = atan2(-ax_f, az_f) * RAD_TO_DEG;

  ret.accel_mag = hyp3(a.acceleration.x, a.acceleration.y, a.acceleration.z);

  return ret;
}

float measureBattery() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // the pin is connected to a 1/2 voltage divider
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

void send_packet(const tx_packet_t& packet) {
  const uint8_t tx_size = sizeof(tx_packet_t);
  uint8_t tx_buf[tx_size] = {0};
  memcpy(tx_buf, &packet, tx_size);

  if(rf95.mode() == RHGenericDriver::RHMode::RHModeTx) {
    rf95.waitPacketSent();
  }
  delay(8);
  rf95.send((uint8_t *)tx_buf, tx_size);
}

void initializeOrientation() {
  // Read the first set of data and set the time of measurement
  timer = micros();
  update_imu();

  // initialize filtered values with current values
  ax_f = a.acceleration.x;
  ay_f = a.acceleration.y;
  az_f = a.acceleration.z;

  // Compute roll and pitch
  roll_pitch_t rp = read_accel_angles();

  // Set starting angles
  compAngleX = rp.roll;
  compAngleY = rp.pitch;
}

void setup() {
  Serial.begin(115200);

#if USE_SERIAL
  // Wait for serial connection
  while (!Serial) {
    delay(1);
  }
#endif

  //
  // Initialize radio
  //

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // reset the radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  // Defaults are +13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // Both the tag and base configuration must match.

  rf95.setModemConfig(RH_RF95::ModemConfigChoice::Bw500Cr45Sf128);

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  // transmitter power can be from from 5 to 23 dBm, RFO must be false
  rf95.setTxPower(23, false);


  // random seed used for congestion control delay backoff
  randomSeed(analogRead(A0));

  //
  // Initialize IMU
  //

  if(!lsm.begin()) {
      Serial.println("Failed to detect LSM9DS1");
      while(1); // halt
  }

  // Configure IMU resolution
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  initializeOrientation();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  uint32_t dt_us = micros() - timer;
  double dt = (double)dt_us / 1000000; // Calculate delta time
  timer = micros();

  update_imu();
  update_filtered_accel();
  roll_pitch_t rp = read_accel_angles();

  double gyroXrate = g.gyro.x;
  double gyroYrate = g.gyro.y;

  // Resolve the transition problem (when the accelerometer angle jumps between +/-180 deg
  if ((rp.pitch < -90 && compAngleY > 90) || (rp.pitch > 90 && compAngleY < -90)) {
    compAngleY = rp.pitch;
  }

  // Invert rate when the orientation is perpendicular
  if (abs(compAngleY) > 90)
    gyroXrate = -gyroXrate;

  // Calculate the angle using a Complimentary filter
  compAngleX = (1 - COMP_RATIO) * (compAngleX + gyroXrate * dt) + COMP_RATIO * rp.roll;
  compAngleY = (1 - COMP_RATIO) * (compAngleY + gyroYrate * dt) + COMP_RATIO * rp.pitch;

  if (loop_i > 0 && loop_i % SAMPLE_COUNT == 0) {
#if USE_SERIAL
    Serial.print("atan\t"); Serial.print(rp.roll);
    Serial.print("\t"); Serial.print(rp.pitch); Serial.print("\t");  
    Serial.print("comp\t"); Serial.print(compAngleX);
    Serial.print("\t"); Serial.print(compAngleY);
#endif
    last_orientation = orientation;

    if(abs(compAngleX) < 45 && abs(compAngleY) < 45) {
      orientation = OR_INVERTED;
    } else if(abs(compAngleY) > 135 && abs(compAngleX) < 45) {
      orientation = OR_UPRIGHT;
    } else if(abs(compAngleX) < 45 && compAngleY > 45) {
      orientation = OR_BOTTOM;
    } else if(abs(compAngleX) < 45 && compAngleY < -45) {
      orientation = OR_TOP;
    } else if(compAngleX < -45) {
      orientation = OR_RIGHT;
    } else if(compAngleX > 45) {
      orientation = OR_LEFT;
    } else {
      orientation = OR_UNKNOWN;
    }

    if(orientation != last_orientation) chain = 0;
    chain++;
#if USE_SERIAL
    Serial.print("\t");
    Serial.print(orientation);
    Serial.print("\t");
    Serial.print(chain);
    Serial.print("\tact "); Serial.print(displacement * 100);
    Serial.println();
#endif
    tx_packet_t packet;
    packet.orientation = orientation;
    packet.chain = chain;
    packet.angle_x = compAngleX;
    packet.angle_y = compAngleY;
    packet.displacement = displacement * 100;
    memcpy(packet.samples, sample_buf, SAMPLE_COUNT*sizeof(imu_sample_t));

    displacement = 0;
    ledState = HIGH;
    blink_on = timer;
    digitalWrite(LED_BUILTIN, ledState);

    // fire and forget. RH will block if a transmission is active.
    send_packet(packet);
  }

  uint8_t sample_buf_idx = loop_i % SAMPLE_COUNT;
  sample_buf[sample_buf_idx].ax = ax_f;
  sample_buf[sample_buf_idx].ay = ay_f;
  sample_buf[sample_buf_idx].az = az_f;
  sample_buf[sample_buf_idx].wx = gyroXrate;
  sample_buf[sample_buf_idx].wy = gyroYrate;
  sample_buf[sample_buf_idx].dt_us = dt_us >> 2;

  if (timer - blink_on >= 100 && ledState) {
    ledState = LOW;
    digitalWrite(LED_BUILTIN, ledState);
  }

  loop_i++;
  double accel_diff = abs(rp.accel_mag - accel_g);
  double accel, avg_vel, vel = 0;

  // noise margin 
  if(accel_diff > ACCEL_THRESH) {
    accel = (accel_diff + a_last) / 2;
    vel += accel * dt;
    avg_vel = (v_last + vel) / 2;
    v_last = vel;
    
    a_last = accel_diff;
    displacement += avg_vel*dt;
  }
}

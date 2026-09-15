// ---------------------------------------------------------------
// sensor_node_esp32.ino  (fable/sensor_node)
//
// ONE common sensor node for fable/3R, fable/1T and fable/3R1T.
// 3R uses the limb + platform frames, 1T uses the encoder frames,
// 3R1T uses all of them. Flash this once and leave it.
//
// Sensor node for the 3R1T rig: three limb joint angles, the platform
// orientation, and the translation-stage encoder, all on CAN2.
//
// All the sensor maths happens here, not on the MasterMACS. The MACS
// gets finished numbers in the units its kinematics wants, so its 2 ms
// ISR spends its budget on the IK, not on quaternions.
//
// ---------------- IMU mapping (AS WIRED) --------------------------
//   mux ch0 -> LIMB 1     home   0.00 deg
//   mux ch1 -> PLATFORM   XYZ intrinsic Euler
//   mux ch2 -> LIMB 2     home 240.00 deg
//   mux ch3 -> LIMB 3     home 120.00 deg
//
// Confirmed on the bench: moving a limb by hand moves the matching
// theta. An earlier build had ch0 and ch3 the other way round, which
// showed up as "move limb 1, arm 3 responds".
//
// >>> The CAN frame is ARM-ORDERED, not channel-ordered. <<<
// theta1 IS arm 1, theta2 IS arm 2, theta3 IS arm 3. The channel
// shuffle is done here, once, in ARM_CHANNEL below - so the MACS side
// needs no crossover and nothing downstream has to remember the loom.
//
// This differs from the older fable/homing build, where theta1 meant
// mux channel 0 and the MACS undid the shuffle with
// HOMING_SRC_ARM1 = theta3. If you flash this sketch, that programme's
// HOMING_SRC_* defines must be changed to the straight-through
// mapping or arm 1 and arm 3 will be swapped.
//
// ---------------- Wiring (ESP32 DevKit) --------------------------
//   3V3 -> mux VIN, each IMU VCC, transceiver VCC
//   GND -> everything, *and* the MasterMACS CAN GND
//   GPIO21 -> mux SDA        GPIO22 -> mux SCL      (mux 0x70)
//   mux SD0..SD3 / SC0..SC3 -> IMU 1..4  (each AD0 -> GND, all 0x68)
//   GPIO4 -> transceiver TXD     GPIO5 -> transceiver RXD
//   transceiver RS/STB -> GND, CANH/CANL -> MACS CAN2, 120R each end
//   GPIO32 -> encoder A    GPIO33 -> encoder B    GPIO25 -> encoder REF
//
// MSQS output is 5 V RS-422 - bring A/B/REF through an RS-422 receiver
// on 3.3 V. 5 V on these pins will damage the ESP32.
//
// ---------------- Frames (8 bytes, little-endian) ----------------
//   0x6E4  limb joint angles   (UNCHANGED from the homing build)
//     0-1 theta1 uint16 [cdeg 0..35999]   (arm 3)
//     2-3 theta2 uint16                   (arm 2)
//     4-5 theta3 uint16                   (arm 1)
//     6   status bit0/1/2 = IMU 1/2/3 ok, bit3 cal done, bit4 converged
//     7   seq
//
//   0x6E8  platform orientation, XYZ intrinsic Euler
//     0-1 eulerX int16 [cdeg]   rotation about X   (gravity referenced)
//     2-3 eulerY int16 [cdeg]   then about new Y   (gravity referenced)
//     4-5 eulerZ int16 [cdeg]   then about new Z   (SEE THE YAW WARNING)
//     6   status bit0 platform IMU ok, bit1 magnetometer present,
//                bit2 yaw datum set, bit3 converged
//     7   seq
//
//   0x6E9  translation stage, live
//     0-3 position int32 [um from the encoder datum]
//     4-5 velocity int16 [0.1 mm/s, signed: + = positive direction]
//     6   status bit0 REF level now, bit1 ref ever seen,
//                bit2 moving, bit3 direction (1 = positive),
//                bits4-7 rolling count of completed REF passes
//     7   seq
//
//   0x6EA  translation stage, reference-mark edges
//     0-3 position of the last REF rising edge  int32 [um]
//     4-7 position of the last REF falling edge int32 [um]
//     Both are latched by the ESP32 at edge time, so the MACS gets
//     the mark's true extent without having to poll fast enough to
//     catch it. Homing takes the midpoint of these two.
//
// ---------------- YAW WARNING ------------------------------------
// This is a 6-DOF filter: accelerometer + gyro. Roll and pitch are
// referenced to gravity and hold indefinitely. YAW (eulerZ) HAS NO
// ABSOLUTE REFERENCE - it starts wherever the platform happened to be
// and drifts with gyro bias, typically a few degrees per minute.
//
// Two ways to make it usable, in order of preference:
//   1. Zero it at a known pose ('y' command, or the panel button) and
//      re-zero whenever you re-home. Good for minutes, not hours.
//   2. Use the AK8963 magnetometer for an absolute heading. The board
//      reports whether one is present (status bit1). That needs a
//      9-DOF filter and a magnetic survey of the rig - see the
//      MPU-9250/magnet_mapping tooling.
// Until one of those is done, treat eulerZ as relative, not absolute.
//
// ---------------- Commands (115200 8N1) --------------------------
//   h  help              c  recalibrate gyro bias (keep rig still)
//   y  zero the platform yaw datum here
//   z  zero the encoder here      s <um> encoder microns per count
//   f <hz> send rate 1..100       b <bps> bitrate
//   p  1 Hz status print on/off
// ---------------------------------------------------------------

#include <Wire.h>
#include "driver/twai.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

// ---------------- I2C / IMUs -------------------------------------
const int MUX_ADDR = 0x70;
const int MPU_ADDR = 0x68;
const int MAG_ADDR = 0x0C;
const int SDA_PIN  = 21;
const int SCL_PIN  = 22;

const int NUM_IMUS = 4;

// Which mux channel each arm's IMU is on, indexed by ARM-1.
// arm 1 -> ch0,  arm 2 -> ch2,  arm 3 -> ch3.  Platform -> ch1.
//
// >>> IF AN ARM ANGLE FOLLOWS THE WRONG LIMB, EDIT THIS ONE LINE. <<<
// Power the motors OFF, move each limb by hand, and see which arm angle
// moves. That isolates the IMU wiring from the motor wiring completely.
const int ARM_CHANNEL[3] = { 0, 2, 3 };
const int IDX_PLATFORM   = 1;

const uint8_t REG_CONFIG      = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONF  = 0x1C;
const uint8_t REG_SMPLRT_DIV  = 0x19;
const uint8_t REG_INT_PIN_CFG = 0x37;
const uint8_t REG_ACCEL_XOUT  = 0x3B;
const uint8_t REG_PWR_MGMT_1  = 0x6B;
const uint8_t REG_WHO_AM_I    = 0x75;
const uint8_t AK_WIA          = 0x00;

const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE  = 131.0f;

// ---------------- Encoder ----------------------------------------
#define ENC_A_PIN   GPIO_NUM_32
#define ENC_B_PIN   GPIO_NUM_33
#define ENC_REF_PIN GPIO_NUM_25

const int PCNT_HIGH = 16384;
const int PCNT_LOW  = -16384;

pcnt_unit_handle_t    pcntUnit = NULL;
pcnt_channel_handle_t pcntChA = NULL, pcntChB = NULL;
volatile int32_t      pcntAccum = 0;

// MSQS TTL resolution is 0.1 um per count. This was 1.0, which reported
// every position and velocity ten times too large.
float   umPerCount = 0.1f;
int32_t encZero    = 0;

// REF edges are latched in the ISR and resolved in the main loop.
//
// Rising and falling are latched SEPARATELY, each with its own
// timestamp. A single shared slot loses an edge whenever both occur
// between two loop iterations - which is what happens when the stage
// crosses the mark at speed rather than by hand, since loop() can stall
// several ms on the CAN transmits. The rise would then keep a stale
// value from an earlier crossing and the mark would look impossibly
// wide, so the MACS would reject the pass and drive straight past it.
volatile bool     refRisePending = false;
volatile bool     refFallPending = false;
volatile uint32_t refRiseUs = 0, refFallUs = 0;
int32_t  refRiseUm = 0, refFallUm = 0;
bool     refSeen   = false;
bool     refLevel  = false;
// Incremented on the FALLING edge, i.e. once a whole pass across the
// mark is complete and both latched edges are valid. The MACS watches
// this rather than the level: a fast pass can begin and end between two
// CAN frames, so the level alone is missable but a counter never is.
uint32_t refPasses = 0;

// ---------------- CAN --------------------------------------------
#define CAN_TX_PIN    GPIO_NUM_4
#define CAN_RX_PIN    GPIO_NUM_5
#define CAN_ID_THETA  0x6E4
#define CAN_ID_PLAT   0x6E8
#define CAN_ID_TRANS  0x6E9
#define CAN_ID_REF    0x6EA

uint32_t canBitrate = 1000000;
bool     canUp = false;
uint32_t txCount = 0, txFail = 0;

// ---------------- Timing / filter --------------------------------
uint32_t sendHz = 50;
bool     statusPrint = true;

const uint32_t SAMPLE_PERIOD_US = 10000;   // 100 Hz
const float    DT = 0.01f;
const float    BETA_CONVERGE  = 2.0f;
const float    BETA_RUN       = 0.1f;
const int      CONVERGE_LOOPS = 150;
const int      CAL_SAMPLES    = 200;

// Limb rotation axis in each sensor's BODY frame, from the homing
// build. These were measured PER LIMB, so they follow the limb when the
// loom changes - they are indexed by ARM-1 here, not by mux channel.
// NOTE: all three measured axes agree to within about 1.5 degrees of
// each other, so which one is assigned to which arm changes theta by at
// most that much. If you need better than that, re-run the axis
// determination per limb - do not just shuffle these.
const float NHATS[3][3] = {
  { -0.700f, -0.714f, -0.013f },   // arm 1  (mux ch0)
  { -0.717f, -0.697f, -0.001f },   // arm 2  (mux ch2)
  { -0.707f, -0.706f, -0.027f },   // arm 3  (mux ch3)
};
const float THETA_OFFSETS[3] = { 180.0f, 180.0f, 180.0f };

// ---------------- State ------------------------------------------
struct ImuState {
  bool  present, magPresent, ok;
  float accel[3], gyro[3], gyroBias[3];
  float q[4];
  float e1[3], e2[3];
  float theta;                     // limbs only
};
ImuState imu[NUM_IMUS];

float platEuler[3] = {0, 0, 0};    // XYZ intrinsic [deg]
float yawDatum = 0.0f;
bool  yawDatumSet = false;

bool     muxOk = false, calDone = false, converged = false;
uint8_t  seq = 0;
uint32_t loopCount = 0;
uint32_t nextSampleUs = 0, nextSendUs = 0, lastStatusMs = 0;

char    cmdBuf[32];
uint8_t cmdLen = 0;

// =================================================================
// Vector helpers
// =================================================================
static void vecCross(const float a[3], const float b[3], float o[3]) {
  o[0] = a[1] * b[2] - a[2] * b[1];
  o[1] = a[2] * b[0] - a[0] * b[2];
  o[2] = a[0] * b[1] - a[1] * b[0];
}
static float vecDot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static bool vecUnit(float v[3]) {
  float n = sqrtf(vecDot(v, v));
  if (n < 1e-6f) return false;
  v[0] /= n; v[1] /= n; v[2] /= n;
  return true;
}

// =================================================================
// I2C
// =================================================================
static bool muxSelect(int ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}
static bool i2cPresent(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}
static bool regRead(uint8_t a, uint8_t r, uint8_t *o, uint8_t n) {
  Wire.beginTransmission(a);
  Wire.write(r);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) o[i] = Wire.read();
  return true;
}
static bool regWrite(uint8_t a, uint8_t r, uint8_t v) {
  Wire.beginTransmission(a);
  Wire.write(r);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

static void imuInit(int ch) {
  imu[ch].present = false;
  imu[ch].magPresent = false;
  if (!muxSelect(ch)) return;
  uint8_t who = 0;
  if (!regRead(MPU_ADDR, REG_WHO_AM_I, &who, 1)) return;
  imu[ch].present = true;

  regWrite(MPU_ADDR, REG_PWR_MGMT_1, 0x80);
  delay(50);
  regWrite(MPU_ADDR, REG_PWR_MGMT_1, 0x01);
  delay(10);
  regWrite(MPU_ADDR, REG_CONFIG, 0x03);
  regWrite(MPU_ADDR, REG_SMPLRT_DIV, 0x00);
  regWrite(MPU_ADDR, REG_GYRO_CONFIG, 0x00);
  regWrite(MPU_ADDR, REG_ACCEL_CONF, 0x00);
  regWrite(MPU_ADDR, REG_INT_PIN_CFG, 0x02);
  delay(10);
  uint8_t wia = 0;
  if (regRead(MAG_ADDR, AK_WIA, &wia, 1) && wia == 0x48) imu[ch].magPresent = true;

  Serial.printf("#,IMU %d ch%d whoami=0x%02X mag %s%s\n", ch + 1, ch, who,
                imu[ch].magPresent ? "yes" : "no",
                ch == IDX_PLATFORM ? "  (PLATFORM)" : "");
}

static bool imuRead(int ch) {
  if (!imu[ch].present) return false;
  if (!muxSelect(ch)) return false;
  uint8_t b[14];
  if (!regRead(MPU_ADDR, REG_ACCEL_XOUT, b, 14)) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  imu[ch].accel[0] = raw[0] / ACCEL_SCALE;
  imu[ch].accel[1] = raw[1] / ACCEL_SCALE;
  imu[ch].accel[2] = raw[2] / ACCEL_SCALE;
  imu[ch].gyro[0]  = raw[4] / GYRO_SCALE;
  imu[ch].gyro[1]  = raw[5] / GYRO_SCALE;
  imu[ch].gyro[2]  = raw[6] / GYRO_SCALE;
  return true;
}

// =================================================================
// Madgwick 6-DOF, same as the homing build
// =================================================================
static void madgwickUpdate(float q[4], float gx, float gy, float gz,
                           float ax, float ay, float az, float beta) {
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  float qd0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qd1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qd2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qd3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n > 0.0f) {
    ax /= n; ay /= n; az /= n;
    float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;
    float s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
    float s1 =  2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
    float s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
    float s3 =  2.0f * q1 * f1 + 2.0f * q2 * f2;
    n = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (n > 0.0f) {
      qd0 -= beta * s0 / n; qd1 -= beta * s1 / n;
      qd2 -= beta * s2 / n; qd3 -= beta * s3 / n;
    }
  }
  q0 += qd0 * DT; q1 += qd1 * DT; q2 += qd2 * DT; q3 += qd3 * DT;
  n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q[0] = q0 / n; q[1] = q1 / n; q[2] = q2 / n; q[3] = q3 / n;
}

static float computeTheta(const float q[4], const float e1[3],
                          const float e2[3], float offsetDeg) {
  float gB[3] = {
    2.0f * (q[1] * q[3] - q[0] * q[2]),
    2.0f * (q[0] * q[1] + q[2] * q[3]),
    2.0f * (0.5f - q[1] * q[1] - q[2] * q[2]),
  };
  float t = atan2f(vecDot(gB, e2), vecDot(gB, e1)) * 180.0f / PI + offsetDeg;
  t = fmodf(t, 360.0f);
  if (t < 0.0f) t += 360.0f;
  return t;
}

// XYZ INTRINSIC Euler angles from the quaternion.
//
// Intrinsic X-then-Y'-then-Z'' means R = Rx(a) * Ry(b) * Rz(c). Writing
// that product out gives
//     R02 =  sin b
//     R00 =  cos b cos c      R01 = -cos b sin c
//     R12 = -sin a cos b      R22 =  cos a cos b
// so b comes from R02 and the other two from those pairs. Gimbal lock
// is at b = +/-90 deg, where a and c stop being separable; there we
// fold the whole rotation into a and leave c at zero.
static void quatToEulerXYZ(const float q[4], float out[3]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];

  float R00 = 1.0f - 2.0f * (y * y + z * z);
  float R01 = 2.0f * (x * y - w * z);
  float R02 = 2.0f * (x * z + w * y);
  float R12 = 2.0f * (y * z - w * x);
  float R22 = 1.0f - 2.0f * (x * x + y * y);
  float R10 = 2.0f * (x * y + w * z);
  float R11 = 1.0f - 2.0f * (x * x + z * z);

  if (R02 > 1.0f)  R02 = 1.0f;
  if (R02 < -1.0f) R02 = -1.0f;

  float b = asinf(R02);
  float a, c;
  if (fabsf(R02) > 0.99999f) {
    // Gimbal lock. With cos b = 0 the matrix collapses to
    //   b = +90:  R10 = sin(a+c), R11 = cos(a+c)
    //   b = -90:  R10 = sin(c-a), R11 = cos(c-a)
    // so only the sum (or difference) is observable. Fold it all into
    // a and leave c at zero - note the sign flips with the sign of b.
    c = 0.0f;
    if (R02 > 0.0f) a =  atan2f(R10, R11);
    else            a = -atan2f(R10, R11);
  } else {
    a = atan2f(-R12, R22);
    c = atan2f(-R01, R00);
  }
  out[0] = a * 180.0f / PI;
  out[1] = b * 180.0f / PI;
  out[2] = c * 180.0f / PI;
}

static void calibrateGyro() {
  Serial.println("#,calibrating gyro bias (2 s) - keep the rig still...");
  float sums[NUM_IMUS][3] = {};
  int   got[NUM_IMUS] = {};
  for (int n = 0; n < CAL_SAMPLES; n++) {
    for (int i = 0; i < NUM_IMUS; i++) {
      if (imuRead(i)) {
        sums[i][0] += imu[i].gyro[0];
        sums[i][1] += imu[i].gyro[1];
        sums[i][2] += imu[i].gyro[2];
        got[i]++;
      }
    }
    delay(10);
  }
  for (int i = 0; i < NUM_IMUS; i++) {
    if (got[i] > CAL_SAMPLES / 2) {
      for (int k = 0; k < 3; k++) imu[i].gyroBias[k] = sums[i][k] / got[i];
    } else if (imu[i].present) {
      Serial.printf("E!,IMU %d gave only %d/%d calibration samples\n",
                    i + 1, got[i], CAL_SAMPLES);
    }
  }
  calDone = true;
  converged = false;
  loopCount = 0;
  Serial.println("OK,gyro calibration done");
}

// =================================================================
// Encoder
// =================================================================
static inline int IRAM_ATTR fastRead(gpio_num_t pin) {
  if (pin < 32) return (REG_READ(GPIO_IN_REG) >> pin) & 1;
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}

// Both edges, each into its own slot so neither can overwrite the other.
static void IRAM_ATTR refIsr() {
  if (fastRead(ENC_REF_PIN)) {
    refRiseUs = micros();
    refRisePending = true;
  } else {
    refFallUs = micros();
    refFallPending = true;
  }
}

static bool IRAM_ATTR onPcntReach(pcnt_unit_handle_t u,
                                  const pcnt_watch_event_data_t *e, void *ctx) {
  pcntAccum += e->watch_point_value;
  return false;
}

static int32_t encRead() {
  for (int t = 0; t < 3; t++) {
    int32_t before = pcntAccum;
    int raw = 0;
    pcnt_unit_get_count(pcntUnit, &raw);
    if (pcntAccum == before) return before + raw;
  }
  int raw = 0;
  pcnt_unit_get_count(pcntUnit, &raw);
  return pcntAccum + raw;
}

static bool pcntSetup() {
  pcnt_unit_config_t uc = {};
  uc.high_limit = PCNT_HIGH;
  uc.low_limit  = PCNT_LOW;
  if (pcnt_new_unit(&uc, &pcntUnit) != ESP_OK) return false;

  pcnt_glitch_filter_config_t fc = {};
  fc.max_glitch_ns = 1000;
  pcnt_unit_set_glitch_filter(pcntUnit, &fc);

  pcnt_chan_config_t ca = {};
  ca.edge_gpio_num  = ENC_A_PIN;
  ca.level_gpio_num = ENC_B_PIN;
  if (pcnt_new_channel(pcntUnit, &ca, &pcntChA) != ESP_OK) return false;
  pcnt_channel_set_edge_action(pcntChA, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(pcntChA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                         PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_chan_config_t cb = {};
  cb.edge_gpio_num  = ENC_B_PIN;
  cb.level_gpio_num = ENC_A_PIN;
  if (pcnt_new_channel(pcntUnit, &cb, &pcntChB) != ESP_OK) return false;
  pcnt_channel_set_edge_action(pcntChB, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcntChB, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                         PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_add_watch_point(pcntUnit, PCNT_HIGH);
  pcnt_unit_add_watch_point(pcntUnit, PCNT_LOW);
  pcnt_event_callbacks_t cbs = {};
  cbs.on_reach = onPcntReach;
  pcnt_unit_register_event_callbacks(pcntUnit, &cbs, NULL);

  if (pcnt_unit_enable(pcntUnit) != ESP_OK) return false;
  if (pcnt_unit_clear_count(pcntUnit) != ESP_OK) return false;
  if (pcnt_unit_start(pcntUnit) != ESP_OK) return false;

  gpio_set_pull_mode(ENC_A_PIN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(ENC_B_PIN, GPIO_PULLUP_ONLY);
  return true;
}

// =================================================================
// CAN
// =================================================================
static twai_timing_config_t timingFor(uint32_t bps) {
  switch (bps) {
    case 125000: { twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS(); return t; }
    case 250000: { twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS(); return t; }
    case 500000: { twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS(); return t; }
    case 800000: { twai_timing_config_t t = TWAI_TIMING_CONFIG_800KBITS(); return t; }
    default:     { twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();   return t; }
  }
}

static bool canStart() {
  if (canUp) { twai_stop(); twai_driver_uninstall(); canUp = false; }
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN,
                                                        TWAI_MODE_NORMAL);
  g.tx_queue_len = 24;
  g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                     TWAI_ALERT_TX_FAILED;
  twai_timing_config_t t = timingFor(canBitrate);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  canUp = true;
  return true;
}

// Short name for the controller state, for the once-a-second line.
// RUN is healthy. OFF means bus-off: the controller has given up after
// too many failed transmissions, which on CAN means nobody acknowledged
// them. REC is recovering from that.
static const char *canStateName() {
  twai_status_info_t st;
  if (!canUp) return "DOWN";
  if (twai_get_status_info(&st) != ESP_OK) return "????";
  switch (st.state) {
    case TWAI_STATE_RUNNING:    return "RUN";
    case TWAI_STATE_BUS_OFF:    return "OFF";
    case TWAI_STATE_RECOVERING: return "REC";
    case TWAI_STATE_STOPPED:    return "STOP";
  }
  return "????";
}

// 'd' - everything the controller knows about the bus, on demand.
//
// This is the ESP32's half of the diagnosis; SYS_CANERR_RX_1 on the MACS
// is the other half. Read together they separate "nothing is on the wire"
// from "something is on the wire and it is garbled".
//
// tx_err climbing with tx_failed climbing and nothing being received is
// the signature of a transmitter alone on the bus: CAN needs at least one
// other node to acknowledge every frame, so an unacknowledged frame is
// retried until the error counter passes 255 and the controller drops to
// bus-off.
static void canDump() {
  twai_status_info_t st;

  Serial.printf("#,can up=%d bitrate=%lu state=%s\n",
                canUp ? 1 : 0, (unsigned long)canBitrate, canStateName());
  if (!canUp || twai_get_status_info(&st) != ESP_OK) {
    Serial.println("E!,no status available - the driver is not installed");
    return;
  }
  Serial.printf("#,tx_err=%lu rx_err=%lu tx_failed=%lu bus_err=%lu arb_lost=%lu\n",
                (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
                (unsigned long)st.tx_failed_count,  (unsigned long)st.bus_error_count,
                (unsigned long)st.arb_lost_count);
  Serial.printf("#,queued_tx=%lu queued_rx=%lu rx_missed=%lu | sketch tx=%lu fail=%lu\n",
                (unsigned long)st.msgs_to_tx, (unsigned long)st.msgs_to_rx,
                (unsigned long)st.rx_missed_count,
                (unsigned long)txCount, (unsigned long)txFail);
  if (st.state == TWAI_STATE_BUS_OFF || st.tx_error_counter > 127) {
    Serial.println("E!,nothing is acknowledging. Check the MACS is powered and at");
    Serial.println("E!,CANBAUD 88, then CANH/CANL, the 120 ohm ends, the shared GND,");
    Serial.println("E!,and that the transceiver RS/STB pin is tied to GND not floating.");
  }
}

static void canService() {
  if (!canUp) return;
  uint32_t al;
  while (twai_read_alerts(&al, 0) == ESP_OK) {
    if (al & TWAI_ALERT_TX_FAILED) txFail++;
    if (al & TWAI_ALERT_BUS_OFF) {
      Serial.println("E!,bus-off: is the MACS program running, transceiver in CAN2?");
      twai_initiate_recovery();
    }
    if (al & TWAI_ALERT_BUS_RECOVERED) twai_start();
  }
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

static void putI16(uint8_t *d, int o, int32_t v) {
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  d[o] = (uint8_t)(v & 0xFF);
  d[o + 1] = (uint8_t)((v >> 8) & 0xFF);
}
static void putU16(uint8_t *d, int o, uint32_t v) {
  d[o] = (uint8_t)(v & 0xFF);
  d[o + 1] = (uint8_t)((v >> 8) & 0xFF);
}
static void putI32(uint8_t *d, int o, int32_t v) {
  d[o] = (uint8_t)(v & 0xFF);
  d[o + 1] = (uint8_t)((v >> 8) & 0xFF);
  d[o + 2] = (uint8_t)((v >> 16) & 0xFF);
  d[o + 3] = (uint8_t)((v >> 24) & 0xFF);
}

static void canSend(uint32_t id, const uint8_t *d) {
  if (!canUp) return;
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = 8;
  memcpy(m.data, d, 8);
  if (twai_transmit(&m, pdMS_TO_TICKS(2)) == ESP_OK) txCount++;
  else txFail++;
}

static void sendAll(int32_t encCount, float velMmS) {
  uint8_t d[8];

  // ---- 0x6E4 limb angles, ARM-ORDERED: theta1=arm1, 2=arm2, 3=arm3 ----
  for (int a = 0; a < 3; a++) {
    int ch = ARM_CHANNEL[a];
    uint16_t cdeg = (uint16_t)((int32_t)lroundf(imu[ch].theta * 100.0f) % 36000);
    putU16(d, 2 * a, cdeg);
  }
  uint8_t st = 0;
  for (int a = 0; a < 3; a++) if (imu[ARM_CHANNEL[a]].ok) st |= (1 << a);
  if (calDone)   st |= 0x08;
  if (converged) st |= 0x10;
  d[6] = st;
  d[7] = seq;
  canSend(CAN_ID_THETA, d);

  // ---- 0x6E8 platform orientation ----
  float yaw = platEuler[2] - (yawDatumSet ? yawDatum : 0.0f);
  while (yaw > 180.0f)  yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;
  putI16(d, 0, (int32_t)lroundf(platEuler[0] * 100.0f));
  putI16(d, 2, (int32_t)lroundf(platEuler[1] * 100.0f));
  putI16(d, 4, (int32_t)lroundf(yaw * 100.0f));
  st = 0;
  if (imu[IDX_PLATFORM].ok)         st |= 0x01;
  if (imu[IDX_PLATFORM].magPresent) st |= 0x02;
  if (yawDatumSet)                  st |= 0x04;
  if (converged)                    st |= 0x08;
  d[6] = st;
  d[7] = seq;
  canSend(CAN_ID_PLAT, d);

  // ---- 0x6E9 translation stage ----
  putI32(d, 0, (int32_t)lroundf((encCount - encZero) * umPerCount));
  putI16(d, 4, (int32_t)lroundf(velMmS * 10.0f));
  st = 0;
  if (refLevel)              st |= 0x01;
  if (refSeen)               st |= 0x02;
  if (fabsf(velMmS) > 0.01f) st |= 0x04;
  if (velMmS >= 0.0f)        st |= 0x08;
  // bits 4-7: rolling count of completed reference passes. The MACS
  // detects a pass by this nibble CHANGING, so it cannot be missed even
  // if the mark is crossed entirely between two frames.
  st |= (uint8_t)((refPasses & 0x0F) << 4);
  d[6] = st;
  d[7] = seq;
  canSend(CAN_ID_TRANS, d);

  // ---- 0x6EA reference-mark edges ----
  putI32(d, 0, refRiseUm);
  putI32(d, 4, refFallUm);
  canSend(CAN_ID_REF, d);

  seq++;
}

// =================================================================
// Commands
// =================================================================
static void printHelp() {
  Serial.println("#,h help | c gyro cal | y zero yaw datum | z zero encoder");
  Serial.println("#,s <um/count> | f <hz> | b <bitrate> | p status print");
  Serial.println("#,d CAN bus diagnosis - run this when the MACS sees nothing");
}

static void handleCommand(char *line) {
  while (*line == ' ') line++;
  char c = *line;
  char *arg = line + 1;
  switch (c) {
    case 'h': case '?': printHelp(); break;
    case 'c': calibrateGyro(); break;
    case 'y':
      yawDatum = platEuler[2];
      yawDatumSet = true;
      Serial.printf("OK,yaw datum set at %.2f deg - this pose is now yaw 0\n", yawDatum);
      break;
    case 'z':
      encZero = encRead();
      Serial.println("OK,encoder zeroed here");
      break;
    case 'p':
      statusPrint = !statusPrint;
      Serial.printf("OK,status print %s\n", statusPrint ? "on" : "off");
      break;
    case 'd': canDump(); break;
    case 's': {
      float v = atof(arg);
      if (v > 0.0f) { umPerCount = v; Serial.printf("OK,%.4f um/count\n", v); }
      else Serial.println("E!,scale must be > 0");
      break;
    }
    case 'f': {
      long v = atol(arg);
      if (v >= 1 && v <= 100) { sendHz = v; Serial.printf("OK,%ld Hz\n", v); }
      else Serial.println("E!,rate must be 1..100 Hz");
      break;
    }
    case 'b': {
      long v = atol(arg);
      if (v == 125000 || v == 250000 || v == 500000 || v == 800000 || v == 1000000) {
        canBitrate = v;
        if (canStart()) Serial.printf("OK,bitrate %ld\n", v);
        else Serial.println("E!,CAN restart failed");
      } else Serial.println("E!,bad bitrate");
      break;
    }
    case '\0': break;
    default: Serial.printf("E!,unknown command '%c' (h for help)\n", c);
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { cmdBuf[cmdLen] = '\0'; handleCommand(cmdBuf); cmdLen = 0; }
    else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = ch;
  }
}

// =================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("#,sensor_node_esp32 - 3 limb angles + platform Euler + translation encoder");
  Serial.println("#,ch0=LIMB1  ch1=PLATFORM  ch2=LIMB2  ch3=LIMB3");
  Serial.println("#,CAN frame is ARM-ORDERED: theta1=arm1, theta2=arm2, theta3=arm3");

  memset(imu, 0, sizeof(imu));

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  muxOk = i2cPresent(MUX_ADDR);
  if (!muxOk) Serial.println("E!,TCA9548A not answering at 0x70");

  const float zhat[3] = { 0.0f, 0.0f, 1.0f };
  for (int i = 0; i < NUM_IMUS; i++) {
    imu[i].q[0] = 1.0f;
  }
  // Give each arm's basis to the channel that arm is actually wired to.
  for (int a = 0; a < 3; a++) {
    int ch = ARM_CHANNEL[a];
    float nhat[3] = { NHATS[a][0], NHATS[a][1], NHATS[a][2] };
    vecUnit(nhat);
    vecCross(nhat, zhat, imu[ch].e1);
    if (!vecUnit(imu[ch].e1)) {
      Serial.printf("E!,NHAT of arm %d parallel to z - theta%d invalid\n", a + 1, a + 1);
    }
    vecCross(nhat, imu[ch].e1, imu[ch].e2);
    vecUnit(imu[ch].e2);
  }
  for (int i = 0; i < NUM_IMUS; i++) imuInit(i);
  delay(100);

  if (!pcntSetup()) Serial.println("E!,PCNT init failed - encoder unavailable");
  pinMode(ENC_REF_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_REF_PIN), refIsr, CHANGE);
  refLevel = fastRead(ENC_REF_PIN) != 0;
  // If the stage is already sitting ON the mark at power-up there is
  // no rising edge to come, so seed refSeen or the first falling edge
  // would be discarded and that pass would never be counted.
  refSeen  = refLevel;

  if (!canStart()) Serial.println("E!,CAN driver failed to start - check pins 4/5");
  else Serial.printf("#,CAN up at %lu bps\n", (unsigned long)canBitrate);

  calibrateGyro();
  Serial.println("#,NOTE: platform yaw has no absolute reference - press 'y' at a");
  Serial.println("#,      known pose to datum it, and expect slow drift after that.");
  printHelp();

  nextSampleUs = micros();
  nextSendUs   = micros();
  lastStatusMs = millis();
}

// =================================================================
void loop() {
  pollSerial();
  canService();

  static int32_t  lastEnc = 0;
  static uint32_t lastVelUs = 0;
  static float    velMmS = 0.0f;

  int32_t encCount = encRead();

  // Resolve latched REF edges against the live count. Done here rather
  // than in the ISR because the PCNT driver call is not interrupt-safe.
  //
  // BOTH edges are handled in the same iteration if both are pending,
  // and each position is back-dated from its own timestamp using the
  // current speed. Without that, two edges resolved together would land
  // on the same position and the mark would measure as zero wide.
  if (refRisePending || refFallPending) {
    int32_t  umNow = (int32_t)lroundf((encCount - encZero) * umPerCount);
    uint32_t now   = micros();

    if (refRisePending) {
      refRisePending = false;
      // velMmS [mm/s] * age [ms] == microns travelled since the edge
      refRiseUm = umNow - (int32_t)lroundf(velMmS * ((now - refRiseUs) / 1000.0f));
      refSeen   = true;
      refLevel  = true;
    }
    if (refFallPending) {
      refFallPending = false;
      refFallUm = umNow - (int32_t)lroundf(velMmS * ((now - refFallUs) / 1000.0f));
      if (refSeen) refPasses++;      // a complete pass: both edges valid
      refLevel  = false;
    }
  }

  uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;
    loopCount++;
    if (!converged && loopCount > (uint32_t)CONVERGE_LOOPS) converged = true;
    float beta = converged ? BETA_RUN : BETA_CONVERGE;

    for (int i = 0; i < NUM_IMUS; i++) {
      imu[i].ok = imuRead(i);
      if (!imu[i].ok) continue;
      madgwickUpdate(imu[i].q,
                     (imu[i].gyro[0] - imu[i].gyroBias[0]) * PI / 180.0f,
                     (imu[i].gyro[1] - imu[i].gyroBias[1]) * PI / 180.0f,
                     (imu[i].gyro[2] - imu[i].gyroBias[2]) * PI / 180.0f,
                     imu[i].accel[0], imu[i].accel[1], imu[i].accel[2], beta);
      if (i == IDX_PLATFORM) {
        quatToEulerXYZ(imu[i].q, platEuler);
      } else {
        imu[i].theta = computeTheta(imu[i].q, imu[i].e1, imu[i].e2, 180.0f);
      }
    }
  }

  if (lastVelUs && nowUs - lastVelUs >= 20000) {
    velMmS = ((encCount - lastEnc) * umPerCount) * 1000.0f / (float)(nowUs - lastVelUs);
    lastEnc = encCount;
    lastVelUs = nowUs;
  } else if (!lastVelUs) {
    lastEnc = encCount;
    lastVelUs = nowUs;
  }

  uint32_t periodUs = 1000000UL / (sendHz ? sendHz : 1);
  if ((int32_t)(nowUs - nextSendUs) >= 0) {
    nextSendUs += periodUs;
    if ((int32_t)(micros() - nextSendUs) > (int32_t)(10 * periodUs)) {
      nextSendUs = micros();
    }
    sendAll(encCount, velMmS);
  }

  uint32_t ms = millis();
  if (statusPrint && ms - lastStatusMs >= 1000) {
    lastStatusMs = ms;
    // BUGFIX: this had ten format specifiers and eleven arguments -
    // refPasses was passed with nothing to print it, so everything after
    // the %s shifted along by one. "tx=" was showing refPasses, "fail="
    // was showing txCount, and txFail was never printed at all. Anyone
    // reading it while debugging a dead bus was reading the wrong numbers.
    Serial.printf("#,arm1=%.1f arm2=%.1f arm3=%.1f | plat %.1f %.1f %.1f | "
                  "enc %.3f mm %s passes=%lu | can %s tx=%lu fail=%lu\n",
                  imu[ARM_CHANNEL[0]].theta, imu[ARM_CHANNEL[1]].theta, imu[ARM_CHANNEL[2]].theta,
                  platEuler[0], platEuler[1],
                  platEuler[2] - (yawDatumSet ? yawDatum : 0.0f),
                  (encCount - encZero) * umPerCount / 1000.0f,
                  refLevel ? "REF" : "   ", (unsigned long)refPasses,
                  canStateName(),
                  (unsigned long)txCount, (unsigned long)txFail);
  }
}

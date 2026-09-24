#include "Settings.h"

#include <Preferences.h>

#include "Config.h"

void Settings::defaults() {
  d = {};
  d.kp = Config::KP_DEFAULT;
  d.ki = Config::KI_DEFAULT;
  d.kd = Config::KD_DEFAULT;
  d.ticksRev = Config::TICKS_REV_DEFAULT;
  // Calibrado contra multimetro: 11.78V real / 11.935V medido no ADC.
  d.battCal = 0.987f;
  d.maxWheelMps = Config::MAX_WHEEL_MPS;
  d.yawEncoderWeight = Config::YAW_ENCODER_WEIGHT;
  d.staticPwm = Config::STATIC_PWM_DEFAULT;
  d.kvPwm = Config::KV_PWM_DEFAULT;
  d.pwmLimit = Config::PWM_LIMIT_DEFAULT;
  d.maxWzAccel = Config::MAX_WZ_ACCEL;
  d.halfL = Config::HALF_L;
  d.halfW = Config::HALF_W;
  d.wheelDiameter = Config::WHEEL_D_M;
  d.imuYawAxis = Config::IMU_YAW_AXIS_DEFAULT;
  d.imuYawSign = Config::IMU_YAW_SIGN_DEFAULT;
  d.servoTrim = Config::SERVO_TRIM_DEFAULT;
  d.speedProfile = Config::SPEED_PROFILE_DEFAULT;
  dirty = true;
}

void Settings::begin() {
  load();
}

void Settings::load() {
  Preferences p;
  if (!p.begin("robot", true)) {
    defaults();
    return;
  }

  if (!p.getBool("ok", false)) {
    p.end();
    defaults();
    save();
    return;
  }

  d.kp = p.getFloat("kp", Config::KP_DEFAULT);
  d.ki = p.getFloat("ki", Config::KI_DEFAULT);
  d.kd = p.getFloat("kd", Config::KD_DEFAULT);
  d.ticksRev = p.getFloat("ticks", Config::TICKS_REV_DEFAULT);
  d.battCal = p.getFloat("bcal", 0.987f);
  d.maxWheelMps = p.getFloat("maxmps", Config::MAX_WHEEL_MPS);
  d.yawEncoderWeight = p.getFloat(
    "yawweight",
    Config::YAW_ENCODER_WEIGHT
  );
  d.staticPwm = p.getInt("stat", Config::STATIC_PWM_DEFAULT);
  d.kvPwm = p.getFloat("kv", Config::KV_PWM_DEFAULT);
  d.pwmLimit = p.getInt("limit", Config::PWM_LIMIT_DEFAULT);
  d.maxWzAccel = p.getFloat("wzaccel", Config::MAX_WZ_ACCEL);
  d.halfL = p.getFloat("halfl", Config::HALF_L);
  d.halfW = p.getFloat("halfw", Config::HALF_W);
  d.wheelDiameter = p.getFloat("wheeld", Config::WHEEL_D_M);
  d.imuYawAxis = p.getUChar("imuaxis", Config::IMU_YAW_AXIS_DEFAULT);
  d.imuYawSign = p.getChar("imusign", Config::IMU_YAW_SIGN_DEFAULT);
  d.servoTrim = p.getFloat("strim", Config::SERVO_TRIM_DEFAULT);
  d.speedProfile = p.getUChar("vperfil", Config::SPEED_PROFILE_DEFAULT);
  if (d.speedProfile >= Config::SPEED_PROFILE_COUNT) {
    d.speedProfile = Config::SPEED_PROFILE_DEFAULT;
  }

  // MIGRACAO. O flag "ok" so dizia "ja gravei alguma vez" -- sem versao,
  // mudar um default nunca chegava a um robo que ja tinha NVS gravada, e
  // o valor velho vencia em silencio.
  //
  // v1 -> v2: encoders passaram de x1 para x4, entao ticksRev quadruplica.
  // Multiplicar o valor GRAVADO (em vez de forcar o default) preserva
  // qualquer calibracao fina que ja existisse.
  const uint32_t ver = p.getULong("ver", 1);
  p.end();
  if (ver < 2) {
    d.ticksRev *= 4.0f;
    d.imuYawAxis = Config::IMU_YAW_AXIS_DEFAULT;
    d.imuYawSign = Config::IMU_YAW_SIGN_DEFAULT;
  }
  if (ver < 3) {
    // v2 -> v3: feedforward passou de degrau fixo para proporcional a
    // velocidade. Os valores antigos gravados em NVS nao tem como saber
    // disso, entao voltam ao default NOVO -- que foi medido, nao chutado.
    d.staticPwm = Config::STATIC_PWM_DEFAULT;
    d.kvPwm = Config::KV_PWM_DEFAULT;
    d.pwmLimit = Config::PWM_LIMIT_DEFAULT;
  }
  if (ver < 4 && d.maxWheelMps > 1.0f) {
    // v3 -> v4: teto por roda acima de 1 m/s nao protege nada.
    d.maxWheelMps = Config::MAX_WHEEL_MPS;
  }
  if (ver < Config::SETTINGS_VERSION) {
    save();
    return;
  }
  dirty = false;
}

bool Settings::save() {
  Preferences p;
  if (!p.begin("robot", false)) {
    return false;
  }

  p.putBool("ok", true);
  p.putFloat("kp", d.kp);
  p.putFloat("ki", d.ki);
  p.putFloat("kd", d.kd);
  p.putFloat("ticks", d.ticksRev);
  p.putFloat("bcal", d.battCal);
  p.putFloat("maxmps", d.maxWheelMps);
  p.putFloat("yawweight", d.yawEncoderWeight);
  p.putInt("stat", d.staticPwm);
  p.putFloat("kv", d.kvPwm);
  p.putInt("limit", d.pwmLimit);
  p.putFloat("wzaccel", d.maxWzAccel);
  p.putFloat("halfl", d.halfL);
  p.putFloat("halfw", d.halfW);
  p.putFloat("wheeld", d.wheelDiameter);
  p.putUChar("imuaxis", d.imuYawAxis);
  p.putChar("imusign", d.imuYawSign);
  p.putFloat("strim", d.servoTrim);
  p.putUChar("vperfil", d.speedProfile);
  p.putULong("ver", Config::SETTINGS_VERSION);
  p.end();
  dirty = false;
  return true;
}
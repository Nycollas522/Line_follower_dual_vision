  #include <Arduino.h>
  #include <Wire.h>
  #include <driver/pcnt.h>
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>
  #include <math.h>

  // ============================================================
  // ESP32-S3 DevKitC-1 N16R8
  // Ordem lógica / cinemática:
  // M1 = frente-esquerda (FL)
  // M2 = frente-direita  (FR)
  // M3 = traseira-esquerda (RL)
  // M4 = traseira-direita  (RR)
  // USB nativo: GPIO19 / GPIO20 reservados
  // RGB onboard: GPIO38 ou GPIO48 reservados
  // I2C IMU: SDA=GPIO1, SCL=GPIO2
  // ============================================================

  // Pinagem soldada
  constexpr uint8_t MOTOR_IN1[4] = {
    8,   // M1 FL
    5,   // M2 FR
    47,  // M3 RL
    39   // M4 RR
  };

  constexpr uint8_t MOTOR_IN2[4] = {
    9,   // M1 FL
    4,   // M2 FR
    16,  // M3 RL
    40   // M4 RR
  };

  constexpr uint8_t ENC[_A[4] = {
    10,  // M1 FL
    6,   // M2 FR
    21,  // M3 RL
    41   // M4 RR
  };

  constexpr uint8_t ENC_B[4] = {
    11,  // M1 FL
    7,   // M2 FR
    18,  // M3 RL
    42   // M4 RR
  };

  constexpr pcnt_unit_t PCNT_UNIT[4] = {
    PCNT_UNIT_0,
    PCNT_UNIT_1,
    PCNT_UNIT_2,
    PCNT_UNIT_3
  };

  // Ajuste estes sinais após validar cada roda.
  // Em TWIST frente (+vx), todas as rodas devem tracionar para frente
  // e todos os encoders devem aumentar.
  constexpr int8_t MOTOR_SIGN[4] = {1, 1, 1, 1};
  constexpr int8_t ENCODER_SIGN[4] = {1, 1, 1, 1};

  // Geometria e encoder
  constexpr float WHEEL_DIAMETER_M = 0.078f;
  constexpr float ENCODER_TICKS_PER_WHEEL_REV = 330.0f;
  constexpr float WHEEL_CIRCUMFERENCE_M = PI * WHEEL_DIAMETER_M;
  constexpr float TICKS_PER_METER = ENCODER_TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_M;

  // Meça no chassi real e ajuste se necessário.
  constexpr float ROBOT_HALF_LENGTH_M = 0.105f;
  constexpr float ROBOT_HALF_WIDTH_M = 0.090f;
  constexpr float KINEMATIC_K = ROBOT_HALF_LENGTH_M + ROBOT_HALF_WIDTH_M;

  // Controle e telemetria
  constexpr uint32_t CONTROL_PERIOD_US = 10000;   // 100 Hz
  constexpr uint32_t TELEMETRY_PERIOD_MS = 50;    // 20 Hz
  constexpr uint32_t COMMAND_TIMEOUT_MS = 400;

  constexpr int PWM_LIMIT = 180;
  constexpr int PWM_STATIC = 100;                 // ajuste 90..140 para vencer atrito estático
  constexpr float MAX_WHEEL_SPEED_MPS = 0.70f;

  // PID de velocidade: ponto de partida com feedforward estático.
  constexpr float KP = 120.0f;
  constexpr float KI = 100.0f;
  constexpr float KD = 5.0f;
  constexpr float INTEGRAL_LIMIT = 1.2f;

  // IMU e fusão de yaw
  constexpr float YAW_ENCODER_WEIGHT = 0.15f;
  constexpr float ACCEL_FILTER_ALPHA = 0.20f;

  Adafruit_MPU6050 mpu;
  bool imuReady = false;

  int32_t encoderCount[4] = {0, 0, 0, 0};
  int32_t encoderPrevious[4] = {0, 0, 0, 0};
  float wheelSpeedMps[4] = {0, 0, 0, 0};
  float wheelTargetMps[4] = {0, 0, 0, 0};
  float pidIntegral[4] = {0, 0, 0, 0};
  float pidLastError[4] = {0, 0, 0, 0};

  float vxCommand = 0.0f;
  float vyCommand = 0.0f;
  float wzCommand = 0.0f;
  uint32_t lastCommandMs = 0;

  float odomX = 0.0f;
  float odomY = 0.0f;
  float odomYaw = 0.0f;
  float gyroZBias = 0.0f;
  float gyroZRadps = 0.0f;
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;

  uint32_t lastControlUs = 0;
  uint32_t lastTelemetryMs = 0;
  String lineBuffer;

  void setMotor(uint8_t index, int pwm) {
    pwm *= MOTOR_SIGN[index];
    pwm = constrain(pwm, -PWM_LIMIT, PWM_LIMIT);

    if (pwm > 0) {
      analogWrite(MOTOR_IN1[index], pwm);
      analogWrite(MOTOR_IN2[index], 0);
    } else if (pwm < 0) {
      analogWrite(MOTOR_IN1[index], 0);
      analogWrite(MOTOR_IN2[index], -pwm);
    } else {
      analogWrite(MOTOR_IN1[index], 0);
      analogWrite(MOTOR_IN2[index], 0);
    }
  }

  void stopMotors() {
    for (uint8_t i = 0; i < 4; ++i) setMotor(i, 0);
  }

  void setupEncoder(uint8_t index, int pinA, int pinB) {
    pcnt_config_t config = {};
    config.pulse_gpio_num = pinA;
    config.ctrl_gpio_num = pinB;
    config.unit = PCNT_UNIT[index];
    config.channel = PCNT_CHANNEL_0;

    // Quadratura x1: subida em A conta, B define o sentido.
    config.pos_mode = PCNT_COUNT_INC;
    config.neg_mode = PCNT_COUNT_DIS;
    config.lctrl_mode = PCNT_MODE_REVERSE;
    config.hctrl_mode = PCNT_MODE_KEEP;
    config.counter_h_lim = 32767;
    config.counter_l_lim = -32768;

    ESP_ERROR_CHECK(pcnt_unit_config(&config));
    ESP_ERROR_CHECK(pcnt_set_filter_value(PCNT_UNIT[index], 1000));
    ESP_ERROR_CHECK(pcnt_filter_enable(PCNT_UNIT[index]));
    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT[index]));
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT[index]));
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT[index]));
  }

  void readEncoders() {
    for (uint8_t i = 0; i < 4; ++i) {
      int16_t rawCount = 0;
      ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT[i], &rawCount));
      encoderCount[i] = static_cast<int32_t>(rawCount) * ENCODER_SIGN[i];
    }
  }

  void updateImu() {
    if (!imuReady) return;

    sensors_event_t accel, gyro, temperature;
    mpu.getEvent(&accel, &gyro, &temperature);

    gyroZRadps = gyro.gyro.z - gyroZBias;
    accelX += ACCEL_FILTER_ALPHA * (accel.acceleration.x - accelX);
    accelY += ACCEL_FILTER_ALPHA * (accel.acceleration.y - accelY);
    accelZ += ACCEL_FILTER_ALPHA * (accel.acceleration.z - accelZ);
  }

  void calibrateGyro() {
    if (!imuReady) return;

    constexpr int SAMPLES = 500;
    float sumZ = 0.0f;
    sensors_event_t accel, gyro, temperature;

    for (int i = 0; i < SAMPLES; ++i) {
      mpu.getEvent(&accel, &gyro, &temperature);
      sumZ += gyro.gyro.z;
      delay(3);
    }
    gyroZBias = sumZ / static_cast<float>(SAMPLES);
  }

  void setWheelTargetsFromTwist() {
    if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
      vxCommand = 0.0f;
      vyCommand = 0.0f;
      wzCommand = 0.0f;
    }

    // Ordem [FL, FR, RL, RR].
    wheelTargetMps[0] = vxCommand - vyCommand - KINEMATIC_K * wzCommand;
    wheelTargetMps[1] = vxCommand + vyCommand + KINEMATIC_K * wzCommand;
    wheelTargetMps[2] = vxCommand + vyCommand - KINEMATIC_K * wzCommand;
    wheelTargetMps[3] = vxCommand - vyCommand + KINEMATIC_K * wzCommand;

    for (uint8_t i = 0; i < 4; ++i) {
      wheelTargetMps[i] = constrain(
        wheelTargetMps[i], -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS
      );
    }
  }

  void updateOdometry(float dt) {
    float d[4] = {0, 0, 0, 0};

    for (uint8_t i = 0; i < 4; ++i) {
      const int32_t deltaTicks = encoderCount[i] - encoderPrevious[i];
      encoderPrevious[i] = encoderCount[i];
      d[i] = static_cast<float>(deltaTicks) / TICKS_PER_METER;
      wheelSpeedMps[i] = d[i] / dt;
    }

    const float dxBody = (d[0] + d[1] + d[2] + d[3]) * 0.25f;
    const float dyBody = (-d[0] + d[1] + d[2] - d[3]) * 0.25f;
    const float dyawEncoder = (-d[0] + d[1] - d[2] + d[3]) / (4.0f * KINEMATIC_K);
    const float dyawGyro = gyroZRadps * dt;
    const float dyaw = imuReady
      ? ((1.0f - YAW_ENCODER_WEIGHT) * dyawGyro + YAW_ENCODER_WEIGHT * dyawEncoder)
      : dyawEncoder;

    const float yawMid = odomYaw + 0.5f * dyaw;
    odomX += cosf(yawMid) * dxBody - sinf(yawMid) * dyBody;
    odomY += sinf(yawMid) * dxBody + cosf(yawMid) * dyBody;
    odomYaw = atan2f(sinf(odomYaw + dyaw), cosf(odomYaw + dyaw));
  }

  void updatePid(float dt) {
    for (uint8_t i = 0; i < 4; ++i) {
      const float target = wheelTargetMps[i];

      if (fabsf(target) < 0.002f) {
        pidIntegral[i] = 0.0f;
        pidLastError[i] = 0.0f;
        setMotor(i, 0);
        continue;
      }

      const float error = target - wheelSpeedMps[i];
      pidIntegral[i] += error * dt;
      pidIntegral[i] = constrain(pidIntegral[i], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

      const float derivative = (error - pidLastError[i]) / dt;
      pidLastError[i] = error;

      const float pidOutput = KP * error + KI * pidIntegral[i] + KD * derivative;
      const float staticPwm = target > 0.0f ? PWM_STATIC : -PWM_STATIC;
      float output = staticPwm + pidOutput;
      output = constrain(output, static_cast<float>(-PWM_LIMIT), static_cast<float>(PWM_LIMIT));

      setMotor(i, lroundf(output));
    }
  }

  bool parseTwist(const String &line, float &vx, float &vy, float &wz) {
    if (!line.startsWith("TWIST,")) return false;

    const int c1 = line.indexOf(',');
    const int c2 = line.indexOf(',', c1 + 1);
    const int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) return false;

    vx = line.substring(c1 + 1, c2).toFloat();
    vy = line.substring(c2 + 1, c3).toFloat();
    wz = line.substring(c3 + 1).toFloat();
    return true;
  }

  void processCommand(const String &line) {
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;

    if (parseTwist(line, vx, vy, wz)) {
      vxCommand = vx;
      vyCommand = vy;
      wzCommand = wz;
      lastCommandMs = millis();
      return;
    }

    if (line == "STOP") {
      vxCommand = 0.0f;
      vyCommand = 0.0f;
      wzCommand = 0.0f;
      lastCommandMs = 0;
      stopMotors();
      return;
    }

    if (line == "RESET_ODOM") {
      odomX = 0.0f;
      odomY = 0.0f;
      odomYaw = 0.0f;
    }
  }

  void handleSerial() {
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\n' || c == '\r') {
        if (!lineBuffer.isEmpty()) {
          processCommand(lineBuffer);
          lineBuffer = "";
        }
      } else {
        lineBuffer += c;
        if (lineBuffer.length() > 96) lineBuffer = "";
      }
    }
  }

  void publishTelemetry() {
    if (millis() - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
    lastTelemetryMs = millis();

    const float vx = (wheelSpeedMps[0] + wheelSpeedMps[1] + wheelSpeedMps[2] + wheelSpeedMps[3]) * 0.25f;
    const float vy = (-wheelSpeedMps[0] + wheelSpeedMps[1] + wheelSpeedMps[2] - wheelSpeedMps[3]) * 0.25f;
    const float wzEncoder = (-wheelSpeedMps[0] + wheelSpeedMps[1] - wheelSpeedMps[2] + wheelSpeedMps[3]) / (4.0f * KINEMATIC_K);
    const float wz = imuReady ? 0.85f * gyroZRadps + 0.15f * wzEncoder : wzEncoder;

    Serial.printf("ENC,%ld,%ld,%ld,%ld\n",
      static_cast<long>(encoderCount[0]), static_cast<long>(encoderCount[1]),
      static_cast<long>(encoderCount[2]), static_cast<long>(encoderCount[3]));
    Serial.printf("WHEEL,%.4f,%.4f,%.4f,%.4f\n",
      wheelSpeedMps[0], wheelSpeedMps[1], wheelSpeedMps[2], wheelSpeedMps[3]);
    Serial.printf("IMU,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
      accelX, accelY, accelZ, 0.0f, 0.0f, gyroZRadps);
    Serial.printf("ODOM,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
      odomX, odomY, odomYaw, vx, vy, wz);
  }

  void setup() {
    Serial.begin(115200);

    for (uint8_t i = 0; i < 4; ++i) {
      pinMode(MOTOR_IN1[i], OUTPUT);
      pinMode(MOTOR_IN2[i], OUTPUT);
      setupEncoder(i, ENC_A[i], ENC_B[i]);
    }
    stopMotors();

    Wire.begin(1, 2);
    Wire.setClock(400000);
    imuReady = mpu.begin(0x68, &Wire);

    if (imuReady) {
      mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.println("IMU,READY");
      delay(300);
      calibrateGyro();
    } else {
      Serial.println("IMU,NOT_FOUND");
    }

    lastControlUs = micros();
    Serial.println("READY,ESP32S3_MECANUM_PID_STATIC");
  }

  void loop() {
    handleSerial();

    const uint32_t nowUs = micros();
    if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
      const float dt = static_cast<float>(nowUs - lastControlUs) * 1e-6f;
      lastControlUs = nowUs;

      readEncoders();
      updateImu();
      updateOdometry(dt);
      setWheelTargetsFromTwist();
      updatePid(dt);
    }

    publishTelemetry();
  }
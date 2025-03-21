/********************************************************
 * 1) 매크로 설정: 마스터 / 슬레이브 선택
 ********************************************************/
#define IS_MASTER               // (예) 주석 해제 시, 이 보드는 마스터(Commander로 파라미터 입력 + CAN 전송)
// #define SLAVE_1                // 주석 해제 시, 슬레이브1
// #define SLAVE_2                // 슬레이브2
// #define SLAVE_3                // 슬레이브3

// 필요한 보드에 맞춰 위 매크로 중 하나만 활성화해서 빌드한다.
// (ex. 마스터 빌드 시 IS_MASTER만, 슬레이브1 빌드 시 SLAVE_1만)

/********************************************************
 * 2) 헤더 선언
 ********************************************************/
#include <ACAN_ESP32.h>      // CAN 통신 라이브러리
#include <SimpleFOC.h>       // 모터 FOC 라이브러리
#include <math.h>

/********************************************************
 * 3) CAN 통신 관련 상수
 ********************************************************/
// 원하는 CAN 속도 (예시: 500 kbps)
static const uint32_t DESIRED_BIT_RATE = 500UL * 1000UL;

// LED 핀 (ESP32 일부 보드는 2번이 내장 LED)
static const int LED_PIN = 2;

/**
 * (중요) 각 보드별 CAN ID 설정
 * - 마스터 : 0x100
 * - 슬레이브1 : 0x110
 * - 슬레이브2 : 0x120
 * - 슬레이브3 : 0x130
 */
#if defined(IS_MASTER)
static const uint32_t PARAM_CAN_ID = 0x100;  // 마스터는 0x100
#elif defined(SLAVE_1)
static const uint32_t PARAM_CAN_ID = 0x110;
#elif defined(SLAVE_2)
static const uint32_t PARAM_CAN_ID = 0x120;
#elif defined(SLAVE_3)
static const uint32_t PARAM_CAN_ID = 0x130;
#else
// 디폴트: 어떤 매크로도 정의 안 했다면 마스터라고 가정
static const uint32_t PARAM_CAN_ID = 0x100;
#endif

/********************************************************
 * 4) SimpleFOC 모터, 드라이버, 센서 설정
 ********************************************************/
#define INH_A    14     
#define INH_B    27     
#define INH_C    26     
#define EN_GATE  13     
#define M_PWM    33     
#define M_OC     25     
#define OC_ADJ   32     
#define OC_GAIN  5      

#define IOUTA 4
#define IOUTB 12
#define IOUTC 15

BLDCMotor motor(11);                            
BLDCDriver3PWM driver(INH_A, INH_B, INH_C, EN_GATE);
MagneticSensorI2C encoder(AS5600_I2C);
LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

// 임피던스 제어용: 위치 기반 토크 제어
float theta_desired = 0.0f;  
float Kp = 3.0f;  
float Cd = 0.5f; 
float max_speed = 60.0f;
float Kt = 60.0f / (2.0f * PI * 380.0f);  

/********************************************************
 * 5) Commander: 직렬 명령 파싱 (마스터에서만 사용)
 ********************************************************/
#ifdef IS_MASTER
Commander command = Commander(Serial);
void setVelocityP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }
void setVelocityI(char* cmd) { command.scalar(&motor.PID_velocity.I, cmd); }
#endif

/********************************************************
 * 6) 임피던스 제어용 파라미터
 ********************************************************/

/********************************************************
 * 7) 전역 변수 / 자료형
 ********************************************************/
// float ↔ byte 변환용 union
union FloatUnion {
  float f;
  uint8_t b[4];
};

// 파라미터 식별자
static const uint8_t PARAM_ID_REF_ANGLE = 0x01;
static const uint8_t PARAM_ID_KSPRING   = 0x02;
static const uint8_t PARAM_ID_CDAMPER   = 0x03;

/********************************************************
 * 8) 파라미터 변경 시 CAN 전송 함수 (마스터 전용)
 ********************************************************/
#ifdef IS_MASTER
void sendParamOverCAN(uint8_t paramID, float value) {
  CANMessage txFrame;
  txFrame.id = PARAM_CAN_ID;  // 0x100 (마스터 ID) -> 브로드캐스트 용
  txFrame.len = 5; // [1바이트 paramID + 4바이트 float]

  txFrame.data[0] = paramID;

  FloatUnion fu;
  fu.f = value;
  for (int i = 0; i < 4; i++) {
    txFrame.data[i+1] = fu.b[i]; 
  }
  ACAN_ESP32::can.tryToSend(txFrame);
}

/** 
 * 추가: 원하는 슬레이브 ID로 보낼 수 있는 함수
 * paramID : ex) PARAM_ID_REF_ANGLE
 * value   : ex) 10.0f
 */
void sendParamToSlave(uint32_t slaveID, uint8_t paramID, float value) {
  CANMessage txFrame;
  txFrame.id = slaveID; // 여기서 0x110, 0x120, 0x130 등
  txFrame.len = 5;

  txFrame.data[0] = paramID;
  FloatUnion fu;
  fu.f = value;
  for (int i=0; i<4; i++){
    txFrame.data[i+1] = fu.b[i];
  }

  if(ACAN_ESP32::can.tryToSend(txFrame)){
    Serial.print("[MASTER] Sent param ");
    Serial.print(paramID, HEX);
    Serial.print(" to 0x");
    Serial.print(slaveID, HEX);
    Serial.print(" = ");
    Serial.println(value);
  } else {
    Serial.println("[MASTER] Send error!");
  }
}
#endif

/********************************************************
 * 9) Commander 콜백: ref_angle, K_spring, C_damper
 ********************************************************/
#ifdef IS_MASTER
void doRefAngle(char* cmd) {
  command.scalar(&theta_desired, cmd);  
  // 브로드캐스트: 0x100 
  sendParamOverCAN(PARAM_ID_REF_ANGLE, theta_desired);
}
void doKp(char* cmd) {
  command.scalar(&Kp, cmd);
  sendParamOverCAN(PARAM_ID_KSPRING, Kp);
}
void doCd(char* cmd) {
  command.scalar(&Cd, cmd);
  sendParamOverCAN(PARAM_ID_CDAMPER, Cd);
}

/********************************************************
 * 9-1) 새 콜백: 사용자 입력으로 슬레이브 ID와 ref_angle 모두 지정
 * 
 * 예: 시리얼에 
 *  S110 10
 * 라고 치면 -> ID=0x110, theta=10.0 rad
 ********************************************************/
void doSetSlaveAngle(char* cmd) {
  // cmd 예) "110 10" 또는 "120 15.3" 등
  // 첫 번째 토큰: 슬레이브ID(16진)
  // 두 번째 토큰: 목표각(float)

  // 1) 첫 번째 토큰 파싱
  char* token = strtok(cmd, " "); 
  if(token == NULL){
    Serial.println("Usage: S <SlaveID_Hex> <angle>");
    return;
  }
  // 16진 정수 해석
  uint32_t slaveID = (uint32_t) strtol(token, NULL, 16); 
  // ex) "110" -> 0x110

  // 2) 두 번째 토큰 파싱
  token = strtok(NULL, " "); 
  if(token == NULL){
    Serial.println("Usage: S <SlaveID_Hex> <angle>");
    return;
  }
  float angleVal = atof(token); // 문자열 -> float

  // 3) CAN으로 전송
  sendParamToSlave(slaveID, PARAM_ID_REF_ANGLE, angleVal);
  Serial.print("Set slave 0x");
  Serial.print(slaveID, HEX);
  Serial.print(" => ref_angle=");
  Serial.println(angleVal);
}
#endif

/********************************************************
 * 10) setup()
 ********************************************************/
void setup() {
  // 시리얼 초기화 (Commander or 디버그)
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- CAN 설정 ---
  {
    ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
    settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;
    settings.mTxPin = (gpio_num_t)17; // TX pin
    settings.mRxPin = (gpio_num_t)16; // RX pin
    const ACAN_ESP32_Filter filter = ACAN_ESP32_Filter::acceptAll();
    const uint32_t errorCode = ACAN_ESP32::can.begin(settings, filter);
    if (errorCode == 0) {
      Serial.println("CAN configuration OK!");
    } else {
      Serial.print("CAN configuration error: 0x");
      Serial.println(errorCode, HEX);
      while(true){}
    }
  }

  // --- SimpleFOC 설정 ---
  encoder.init();
  motor.linkSensor(&encoder);

  pinMode(M_OC, OUTPUT); digitalWrite(M_OC,LOW);
  pinMode(M_PWM,OUTPUT); digitalWrite(M_PWM,HIGH);
  pinMode(OC_ADJ,OUTPUT);digitalWrite(OC_ADJ,HIGH);
  pinMode(OC_GAIN,OUTPUT);digitalWrite(OC_GAIN,LOW);

  driver.voltage_power_supply = 30;
  driver.pwm_frequency = 15000;
  driver.init();
  motor.linkDriver(&driver);
  cs.linkDriver(&driver);
  motor.voltage_sensor_align = 0.5f;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;  
  motor.motion_downsample = 0.0;

  motor.PID_current_q.P  = 0.05f;
  motor.PID_current_q.I  = 0.6f;
  motor.LPF_current_q.Tf = 0.04f;

  motor.PID_current_d.P  = 0.05f;
  motor.PID_current_d.I  = 0.6f;
  motor.LPF_current_d.Tf = 0.04f;

  motor.PID_velocity.P  = 0.1f;
  motor.PID_velocity.I  = 1.7f;
  motor.LPF_velocity.Tf = 0.001f;
  motor.P_angle.P       = 10.0f;
  motor.LPF_angle.Tf    = 0.0f;

  motor.velocity_limit  = 50.0f;
  motor.voltage_limit   = 20.0f;
  motor.current_limit   = 13.0f;

  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D;
  motor.monitor_downsample = 100;

  motor.init();
  cs.init();
  cs.gain_a *= -1; cs.gain_b *= -1; cs.gain_c *= -1;
  motor.linkCurrentSense(&cs);

  motor.initFOC();

#ifdef IS_MASTER
  // Commander 등록
  command.add('R', doRefAngle, "ref angle [rad]");
  command.add('K', doKp,       "Kp [A/rad]");
  command.add('C', doCd,       "Cd [A/rad]");

  // *** 새로 추가된 커맨드 'S' ***
  command.add('S', doSetSlaveAngle, "S <ID_hex> <angle> : set ref angle for that slave ID");

  Serial.println("[MASTER] FOC Current-based Torque Control + Multi-Slave example");
  Serial.println(" - R: set target angle (broadcast) [rad]");
  Serial.println(" - S: set target angle for a specific slave, ex) 'S110 10' => ID=0x110, angle=10.0");
#else
  Serial.println("[SLAVE] Ready to receive CAN messages for param updates.");
#endif

  Serial.println("Setup complete!");
}

/********************************************************
 * 11) loop()
 ********************************************************/
void loop() {
  // (A) FOC 루프
  motor.loopFOC();

  // 간단 임피던스 식
  float theta  = motor.shaft_angle;
  float dtheta = motor.shaft_velocity;
  float Iq_ref = Kp*(theta_desired - theta) - Cd*dtheta;
  motor.target = Iq_ref;

  // (B) FOC 업데이트
  motor.move();

  // (C) Commander (마스터만)
#ifdef IS_MASTER
  command.run();
#endif

  // (D) CAN 수신
  CANMessage rxFrame;
  while (ACAN_ESP32::can.receive(rxFrame)) {
    if ((rxFrame.id == PARAM_CAN_ID) && (rxFrame.len == 5)) {
      uint8_t paramID = rxFrame.data[0];
      FloatUnion fu;
      fu.b[0] = rxFrame.data[1];
      fu.b[1] = rxFrame.data[2];
      fu.b[2] = rxFrame.data[3];
      fu.b[3] = rxFrame.data[4];
      float newVal = fu.f;
      switch (paramID) {
        case PARAM_ID_REF_ANGLE:
          theta_desired = newVal;
          Serial.print("[CAN] Updated theta_desired: ");
          Serial.println(theta_desired);
          break;
        case PARAM_ID_KSPRING:
          Kp = newVal;
          Serial.print("[CAN] Updated Kp: ");
          Serial.println(Kp);
          break;
        case PARAM_ID_CDAMPER:
          Cd = newVal;
          Serial.print("[CAN] Updated Cd: ");
          Serial.println(Cd);
          break;
        default:
          break;
      }
    } else {
      // 다른 메시지이면 로그만
      Serial.print("Received other CAN frame: ID=0x");
      Serial.print(rxFrame.id, HEX);
      Serial.print(", DLC=");
      Serial.print(rxFrame.len);
      Serial.print(", Data=[");
      for (uint8_t i=0; i<rxFrame.len; i++){
        Serial.print(rxFrame.data[i], HEX);
        if(i<rxFrame.len-1) Serial.print(" ");
      }
      Serial.println("]");
    }
  }
}

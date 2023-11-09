#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" -- dmp를 사용하기 위해 라이브러리 변경
#include "Wire.h"
#include "PID_v1.h"
//라이브러리 설정

#define DEBUG

#ifdef DEBUG
  //#define OUTPUT_READABLE_TIME          //단위 시간 보기
  //#define OUTPUT_READABLE_ACCELGYRO     //가속도, 자이로 값 보기
  //#define OUTPUT_READABLE_QUATERNION    //쿼터니언 값 보기
  //#define OUTPUT_READABLE_RP_Q          //쿼터니언으로 구한 ROLL, PITCH 값 보기
  //#define OUTPUT_READABLE_RP_ACC        //가속도 값으로 구한 ROLL, PITCH 값 보기
  //#define OUTPUT_READABLE_FILTERED_RP   //상보필터를 거친 ROLL, PITCH 값 보기
  //#define OUTPUT_READABLE_ERRORRPY      //ROLL, PITCH 에러 값 보기
  //#define OUTPUT_READABLE_OUTRPY        //OUTPUTROLL, OUTPUTPITCH 값 보기
#endif
//디버깅 설정

#define SERVO_X 9
#define SERVO_Y 10
#define INTERRUPT 2
//핀 설정
MPU6050 accelgyro;

Servo servoX;
Servo servoY;

int16_t _ax, _ay, _az;
int16_t _gx, _gy, _gz;
//센서 값 받아올 변수 선언

double ax, ay, az;
double gx, gy, gz;
//센서 값을 정규화하여 담을 변수

unsigned long lasttime = 0;
//dt를 구하기 위한 변수

double kp = 0.7;
double ki = 0.65;
double kd = 0.4;
//pid기여도상수 => 값 조절 필요함

double Roll = 0.0;
double Pitch = 0.0;
double Yaw = 0.0;
//롤, 피치, 요 각도

Quaternion q;
//쿼터니언 변수

double setpointRoll = 0.0;
double setpointPitch = 0.0;
//원하는 롤, 피치 각도

double outputRoll = 0.0;
double outputPitch = 0.0;
//출력되는 각도

int servoXAngle = 0;
int servoYAngle = 0;
//서보모터의 각도

const float alpha = 0.99;
double accAngleX = 0;
double accAngleY = 0;
double gyroAngleX = 0;
double gyroAngleY = 0;
//상보 필터 변수

PID pidRoll(&Roll, &outputRoll, &setpointRoll, kp, ki, kd, DIRECT);
PID pidPitch(&Pitch, &outputPitch, &setpointPitch, kp, ki, kd, DIRECT);
uint8_t devStatus;
uint8_t fifoBuffer[128];

float euler[3]; // 쿼터니언으로 구한 [YAW, PITCH, ROLL]
VectorFloat gravity;

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    servoX.attach(SERVO_X);
    servoY.attach(SERVO_Y);

    Serial.begin(38400); 
    //디버깅용

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    pinMode(INTERRUPT, INPUT);
    //초기화

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    //센서 연결 확인

    devStatus = accelgyro.dmpInitialize();
    
    if (devStatus == 0)
    {
      accelgyro.CalibrateAccel(3);
      accelgyro.CalibrateGyro(3);
      Serial.println();
      Serial.println("Enabling DMP...");
      accelgyro.setDMPEnabled(true);
      //현재 MPU6050 상태를 기준으로 Calibration
    }

    pidRoll.SetMode(AUTOMATIC);
    pidPitch.SetMode(AUTOMATIC);
    
}

void loop() {
    //1단계: dt 구하기
    unsigned long now = millis();                                   //현재 시간
    double dt = (now - lasttime)*0.001; 

    #ifdef OUTPUT_READABLE_TIME
      Serial.print(dt, 8); Serial.print("\t");                      //현재 시간과 이전 시간 사이의 간격을 dt로 설정
    #endif
    //2단계: 데이터 읽기
    accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer);
    accelgyro.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);       //센서 값 받아오기 => 가속도값 및 자이로스코프 값
    ax = _ax / (16383.0/9.8);
    ay = _ay / (16383.0/9.8);
    az = _az / (16383.0/9.8);
    gx = _gx / 131.0;
    gy = _gy / 131.0;
    gz = _gz / 131.0;

    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(euler, &q, &gravity);
    euler[1] = -euler[1] * 180 / PI;
    euler[2] = euler[2] * 180 / PI;

    #ifdef OUTPUT_READABLE_ACCELGYRO
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.print(gz); Serial.print("\t");                         // 디버깅용 가속도 자이로 값 확인
    #endif
  
    #ifdef OUTPUT_READABLE_QUATERNION
      Serial.print("Quat"); Serial.print("\t");
      Serial.print(q.w); Serial.print("\t");
      Serial.print(q.x); Serial.print("\t");
      Serial.print(q.y); Serial.print("\t");
      Serial.print(q.z); Serial.print("\t");
    #endif

    #ifdef OUTPUT_READABLE_RP_Q
      Serial.print("RP_Q"); Serial.print("\t");
      Serial.print(euler[2]); Serial.print("\t"); // ROLL
      Serial.print(euler[1]); Serial.print("\t"); // PITCH
    #endif

    accAngleX = atan2(ay, sqrt(ax*ax + az*az))*RAD_TO_DEG; //rad -> deg
    accAngleY = atan2(-ax, sqrt(ay*ay + az*az))*RAD_TO_DEG; //rad -> deg         //가속도 센서를 이용한 각도    

    #ifdef OUTPUT_READABLE_RP_ACC
      Serial.print("RP_ACC"); Serial.print("\t");
      Serial.print(accAngleX); Serial.print("\t"); // ROLL
      Serial.print(accAngleY); Serial.print("\t"); // PITCH
    #endif

    //3단계: 상보 필터를 이용한 각도 계산(롤, 피치, 요) => 구하는 방법과 공식에 대한 확인 필요
    
    Roll = (alpha*(euler[2]) + (1-alpha)*accAngleX);               //상보 필터 공식을 이용해 롤 각도 구하기
    Pitch = (alpha*(euler[1]) + (1-alpha)*accAngleY);              //상보 필터 공식을 이용해 피치 각도 구하기
    Yaw += gz * dt;                                                //요 각도 구하기

    /* Reminder: 쿼터니언으로 구한 각 범위는 -180도 ~ 180도 인 반면 가속도로 구한 각 범위는 -90도 ~ 90도임 */
     
    #ifdef OUTPUT_READABLE_FILTERED_RP
      Serial.print("FILTERED_RP"); Serial.print("\t");
      Serial.print(Roll); Serial.print("\t");
      Serial.print(Pitch); Serial.print("\t");
    #endif 

    //4단계: 오차 구하기    
    double errorRoll = setpointRoll - Roll;
    double errorPitch = setpointPitch - Pitch;                             //오차 구하기 -> 원하는 각도(setpoint)와 현재 각도 사이의 차이

    #ifdef OUTPUT_READABLE_ERRORRPY
      Serial.print("eR/eP:  ");
      Serial.print(errorRoll); Serial.print("\t");
      Serial.print(errorPitch); Serial.print("\t");
    #endif

    //5단계: pid 제어

    pidRoll.Compute();
    pidPitch.Compute();

    #ifdef OUTPUT_READABLE_OUTRPY
      Serial.print("r/p:\t");
      Serial.print(outputRoll); Serial.print("\t");
      Serial.print(outputPitch); Serial.print("\t");
    #endif

    //6단계: 액추에이터(서보모터) 작동 및 프로세스
    servoXAngle = map(outputRoll, -90, 90, 0, 180);
    servoYAngle = map(outputPitch, -90, 90, 0, 180);                //서보모터 동작 가용범위로 바꿔주기
    servoX.write(servoXAngle);
    servoY.write(servoYAngle);                                      //서보모터(액추에이터) 작동

    #ifdef DEBUG
      Serial.println();
    #endif

    lasttime = now;                                   
}

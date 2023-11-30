#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
//라이브러리 설정

#define SERVO_X 9
#define SERVO_Y 10
#define INTERRUPT 2
//핀 설정

MPU6050 accelgyro;

Servo servoX;
Servo servoY;

unsigned long lasttime = 0;
//dt를 구하기 위한 변수

Quaternion q;
//쿼터니언 변수

int servoXAngle = 0;
int servoYAngle = 0;
//서보모터의 각도

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

    
}

void loop() {
    //dt 구하기
    unsigned long now = millis();                                   //현재 시간
    double dt = (now - lasttime)*0.001;                             //현재 시간과 이전 시간 사이의 간격을 dt로 설정

    #ifdef OUTPUT_READABLE_TIME
      Serial.print(dt, 8); Serial.print("\t");
    #endif

    //데이터 읽기
    accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer);
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(euler, &q, &gravity);
    euler[1] = -euler[1] * 180 / PI;
    euler[2] = euler[2] * 180 / PI;

    //액추에이터(서보모터) 작동 및 프로세스
    servoXAngle = map(-euler[2], -90, 90, 0, 180);
    servoYAngle = map(-euler[1], -90, 90, 180, 0);                //서보모터 동작 가용범위로 바꿔주기
    servoX.write(servoYAngle);
    servoY.write(servoXAngle);                                      //서보모터(액추에이터) 작동

    lasttime = now;                                   
}

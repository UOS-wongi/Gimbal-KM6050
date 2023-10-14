#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//라이브러리 설정

MPU6050 accelgyro;


Servo servoX;
Servo servoY;
Servo servoZ;


int16_t ax, ay, az;
int16_t gx, gy, gz;
//센서 값 받아올 변수 선언

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

double setpointRoll = 0.0;
double setpointPitch = 0.0;
double setpointYaw = 0.0;
//원하는 롤, 피치, 요 각도

double errorRoll = 0.0;
double errorPitch = 0.0;
double errorYaw = 0.0;
//원하는 각도와 실제 각도 사이의 오차

double lasterrorRoll = 0.0;
double lasterrorPitch = 0.0;
double lasterrorYaw = 0.0;
//적분할 때 필요한 이전 각도 요소

double integralRoll = 0.0;
double integralPitch = 0.0;
double integralYaw = 0.0;
//적분기에 사용되는 각도 요소 -> 오차에 dt를 곱해 구함

double outputRoll = 0.0;
double outputPitch = 0.0;
double outputYaw = 0.0;
//출력되는 각도

int servoXAngle = 0;
int servoYAngle = 0;
int servoZAngle = 0;
//서보모터의 각도

const float alpha = 0.99;
double accAngleX = 0;
double accAngleY = 0;
double accAngleZ = 0;
double gyroAngleX = 0;
double gyroAngleY = 0;
double gyroAngleZ = 0;
//상보 필터 변수

#define OUTPUT_READABLE_ACCELGYRO

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    servoX.attach(9);
    servoY.attach(10);
    servoZ.attach(11);


    Serial.begin(38400); //디버깅용


    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    //초기화

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    //센서 연결 확인

}

void loop() {
    //1단계: dt 구하기
    //unsigned long now = millis();                            //현재 시간
    //double dt = (now - lasttime) / 1000;                     //현재 시간과 이전 시간 사이의 간격을 dt로 설정
    double dt = 0.015;
    //2단계: 데이터 읽기
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);      //센서 값 받아오기 => 가속도값 및 자이로스코프 값
    ax/=(16383.0/90.0);
    ay/=(16383.0/90.0);
    az/=(16383.0/90.0);
    gx/=131.0;
    gy/=131.0;
    gz/=131.0;
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);                                      // 디버깅용 가속도 자이로 값 확인
    

    //3단계: 상보 필터를 이용한 각도 계산(롤, 피치, 요) => 구하는 방법과 공식에 대한 확인 필요
    accAngleX = atan2(ay, sqrt(ax*ax + az*az));
    accAngleY = atan2(-ax, sqrt(ay*ay + az*az));                         //가속도 센서를 이용한 각도
    gyroAngleX = gx*dt;
    gyroAngleY = gy*dt;
    gyroAngleZ = gz*dt;                                                  //자이로 센서를 이용한 각도
    //Roll = (alpha*gyroAngleX + (1-alpha)*accAngleX);          //상보 필터 공식을 이용해 롤 각도 구하기
    //Pitch = (alpha*gyroAngleY + (1-alpha)*accAngleY);         //상보 필터 공식을 이용해 피치 각도 구하기
    //Yaw += gyroAngleZ;                                                    //요 각도 구하기

    Roll = (alpha*(Roll*DEG_TO_RAD + gx*dt) + (1-alpha)*accAngleX)*RAD_TO_DEG;          //상보 필터 공식을 이용해 롤 각도 구하기
    Pitch = (alpha*(Pitch*DEG_TO_RAD + gy*dt) + (1-alpha)*accAngleY)*RAD_TO_DEG;         //상보 필터 공식을 이용해 피치 각도 구하기
    Yaw += gz * dt;                                                                     //요 각도 구하기

    //4단계: 오차 구하기    
    errorRoll = setpointRoll - Roll;
    errorPitch = setpointPitch - Pitch;
    errorYaw = setpointYaw - Yaw;                             //오차 구하기 -> 원하는 각도(setpoint)와 현재 각도 사이의 차이
    Serial.print("eR/eP/eY:");
    Serial.print(errorRoll); Serial.print("\t");
    Serial.print(errorPitch); Serial.print("\t");
    Serial.println(errorYaw); 
    //5단계: pid 제어
    outputRoll = kp*errorRoll + ki*integralRoll + kd*(errorRoll - lasterrorRoll)/dt;
    outputPitch = kp*errorPitch + ki*integralPitch + kd*(errorPitch - lasterrorPitch)/dt;
    outputYaw = kp*errorYaw + ki*integralYaw + kd*(errorYaw - lasterrorYaw)/dt;             //각 각도에 대해 pid 제어 연산을 수행해 출력 값을 찾기
    Serial.print("r/p/y:\t");
    Serial.print(outputRoll); Serial.print("\t");
    Serial.print(outputPitch); Serial.print("\t");
    Serial.println(outputYaw);
    //6단계: 액추에이터(서보모터) 작동 및 프로세스
    servoXAngle = map(outputRoll, -90, 90, 0, 180);
    servoYAngle = map(outputPitch, -90, 90, 0, 180);
    servoZAngle = map(outputYaw, -90, 90, 0, 180);            // 서보모터 동작 가용범위로 바꿔주기
    servoX.write(servoXAngle);
    servoY.write(servoYAngle);
    servoZ.write(servoZAngle);                                //서보모터(액추에이터) 작동
    
    //7단계: 피드백
    lasterrorRoll = errorRoll;
    lasterrorPitch = errorPitch;
    lasterrorYaw = errorYaw;
    integralRoll += errorRoll*dt;
    integralPitch += errorPitch*dt;
    integralYaw += errorYaw*dt;
    delay(50);
    //lasttime = now;                                           //값에 대한 피드백
}
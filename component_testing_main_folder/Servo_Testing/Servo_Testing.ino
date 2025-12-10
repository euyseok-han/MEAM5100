#include "LobotSerialServoControl.h" // 导入库文件(import library file)

// 控制总线舵机转动例程(bus servo rotation control program)

#define SERVO_SERIAL_RX   18
#define SERVO_SERIAL_TX   17
#define receiveEnablePin  13
#define transmitEnablePin 14
HardwareSerial HardwareSerial(2);
LobotSerialServoControl BusServo(HardwareSerial,receiveEnablePin,transmitEnablePin);

int startServoMove = 0;
int lastServoMove = 0;
int pos = 500;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // 设置串口波特率(set baud rate of serial port)
  Serial.println("start...");  // 串口打印"start..."(serial port prints "start...")
  BusServo.OnInit();
  HardwareSerial.begin(115200,SERIAL_8N1,SERVO_SERIAL_RX,SERVO_SERIAL_TX);
  delay(500); // 延时500毫秒(delay for 500ms)

  Serial.println("Starting Position");
  BusServo.LobotSerialServoMove(1,700,1000);
  startServoMove = millis();
  lastServoMove = millis();
}

bool start_en = true;
void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - startServoMove > 1500){
    if(millis() - lastServoMove > 2000){
      if(pos == 0){
        pos = 1000;
      } else {
        pos = 0;
      }
      Serial.print("Moving to ");
      Serial.println(pos);
      BusServo.LobotSerialServoMove(1,pos,1500);  //500 or 900
      lastServoMove = millis();
    }
    


    // Serial.println("Move 1");
    // BusServo.LobotSerialServoMove(1,0,1000); // 设置1号舵机运行到500脉宽位置，运行时间为1000毫秒(Set servo 1 to rotate to the position with pulse width 500 in 1000ms)
    // delay(2000); // 延时2000毫秒(delay for 2000ms)
  
    // Serial.println("Move 2");
    // BusServo.LobotSerialServoMove(1,1000,1000); // 设置1号舵机运行到1000脉宽位置，运行时间为1000毫秒(Set servo 1 to rotate to the position with pulse width 1000 in 1000ms)
    // delay(2000); // 延时2000毫秒(delay for 2000ms)
  
    // Serial.println("Move 3");
    // BusServo.LobotSerialServoMove(1,0,1000); // 设置1号舵机运行到0脉宽位置，运行时间为1000毫秒(Set servo 1 to rotate to the position with pulse width 0 in 1000ms)
    // delay(2000); // 延时2000毫秒(delay for 2000ms)
  
    // Serial.println("Move 4");
    // BusServo.LobotSerialServoMove(1,500,1000); // 设置1号舵机运行到500脉宽位置，运行时间为1000毫秒(Set servo 1 to rotate to the position with pulse width 500 in 1000ms)
    // delay(2000); // 延时2000毫秒(delay for 2000ms)
    // start_en = false;
  }
}

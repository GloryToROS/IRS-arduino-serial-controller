/*
   code write for project:
   https://github.com/Ni3nayka/IRS-1/

   author: Egor Bakay <egor_bakay@inbox.ru> Ni3nayka
   write:  October 2024
   modify: December 2025
*/
 
#include "robot.h"

void setup() {
  setupRobot();
}

void loop() {
  motorSpeedEncUpdate();
  serialDataParser();
  serialROSEnc();
}

void serialDataParser() {
  SerialData data = readSerialData(); // Запрашиваем данные с Serial
  if (data.mode != '\0') { // Если получены данные, обрабатываем их
    if      (data.mode=='m' && data.data_counter==2 && data.data_1==1) Robot.motor_A(data.data_2);
    else if (data.mode=='m' && data.data_counter==2 && data.data_1==2) Robot.motor_B(data.data_2);
    else if (data.mode=='M' && data.data_counter==2) Robot.motors(data.data_1, data.data_2);
    else if (data.mode=='n' && data.data_counter==2) motorCreateTargetSpeedEncRun(data.data_1, data.data_2);
    else if (data.mode=='N' && data.data_counter==2) motorCreateTargetSpeedEncRun2(data.data_1, data.data_2);
    // else if (data.mode=='F' && data.data_counter==1) runGyro(data.data_1);
    // else if (data.mode=='R' && data.data_counter==1) turnGyro(data.data_1);
    // else if (data.mode=='L' && data.data_counter==1) turnGyro(-data.data_1);
    else if (data.mode=='g' && data.data_counter==0) sendDataToSerial();
    else if (data.mode=='C' && data.data_counter==1) motorTargetSpeedEncRun(1,data.data_1);
    // else if (data.mode=='t' && data.data_counter==1) test(); // testErrorEncDistanseRun(data.data_1);
    else Serial.println("ERROR: unknow command");
  }
}
// Файл для функций движения робота
#pragma once

// #define GY25_STRAFE_ANDLE 1
// #define GY25_STRAFE_DT 700
#include "robot.h"

// #define ENC_POROG 50
// #define ENC_TIME 500
// #define ENC_GYRO_TURN_POROG 3
// #define ENC_FORWARD_KP 0.5
// #define ENC_FORWARD_KD 10
// #define ENC_FORWARD_ALIGNMENT_KP 20 // выравнивание колес друг относительно друга 
// #define ENC_TURN_KP 5.0
// #define ENC_TURN_KD 4

// #define ENC_GYRO_FORWARD_KP 2 // 50
// #define ENC_GYRO_FORWARD_KD 60 // 50
// #define ENC_GYRO_FORWARD_UPDATE_DT 20 // 50

// #define GYRO_TURN_KP 5 // 5
// #define GYRO_TURN_KD 80 // 40
#define ENC_ANGLE_TO_PARROT 18.1 // 17
#define ENC_CM_TO_PARROT 122.6 //120
#define ENC_MOTOR_MAX_SPEED 70 // 70
#define ENC_MOTOR_MAX_SPEED_TURN 70
#define ENC_MOTOR_R_BOOST 1 //1.07 // ОН ОТВЕЧАЕТ ЗА ЛЕВЫЙ МОТОР!!!

// 	motorCreateTargetSpeedEncRun2(0,0);
// 	// Motors.run(1, 0);
// 	// Motors.run(2, 0);
// 	Serial.println("END COMMAND: turn - success");
// }
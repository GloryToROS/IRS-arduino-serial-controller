// Файл для функций движения робота
#pragma once

#include <Robot_L298P.h>

#define PARROT_IN_ONE_FULL_ROTATION_WHEEL 2500
#define DIAMETER_WHEEL 6.5

// ========================= VOLTAGE ==============================

const int voltage_counter = 2;
const int voltage_pins[voltage_counter] = {A0, A1};

float getVoltage(int number) {
	if (number>voltage_counter || number<1) return 0.0;
	return analogRead(voltage_pins[number])*0.029325513;
	// int a = 0;
	// const int iteritions = 3;
	// for (int i = 0; i<iteritions; i++) {
	// 	a += analogRead(voltage_pins[number]);
	// 	delay(100);
	// }
	// return float(a)/iteritions*0.029325513;
	
}

// ========================= SERIAL ==============================

#include "serialParser.h"
void sendDataToSerial() {
  Serial.println();
  Serial.println("GY25: 0 0 0 0");
  Serial.print("ENC: ");
  Serial.print(Robot.enc_A);
  Serial.print(" ");
  Serial.println(Robot.enc_B);
  Serial.print("VOLTAGE: ");
  Serial.print(getVoltage(1));
  Serial.print(" ");
  Serial.println(getVoltage(2));
}

long int timer_for_ros_data_send = 0;
void serialROSEnc() { 
  // Выводим данные раз в 100 мс, не мешая вводу
  if (millis() - timer_for_ros_data_send >= 100) {
    timer_for_ros_data_send = millis();
    Serial.print("ENC: ");
    Serial.print(Robot.enc_A);
    Serial.print(" ");
    Serial.print(Robot.enc_B);
    Serial.println();
  }
}

// ========================= CODE ==============================

void setupRobot() {
	Serial.begin(9600);
	Robot.setup();
  Robot.motors(0, 0);
	delay(1000);
	Serial.println("System ready!");
}

int motorTargetRealSpeedEnc[3] = {0, 0, 0};
int motorTargetSpeedEnc[3] = {0, 0, 0};
double motorDifferencial[3] = {0, 0, 0};
// int motorTargetSpeedEnc[3] = {0, 0, 0};

// P-регулятор скорости по энкодерам.
// number: 1 или 2 — номер мотора.
// speed: желаемая скорость в единицах "импульсов энкодера в секунду".
// Функция должна вызываться часто; она самостоятельно поддерживает текущее
// управляющее значение для мотора. Для простоты используется пропорциональный
// коэффициент MOT_KP и ограничение по допустимой мощности мотора.
void motorTargetSpeedEncRun(int number, int speed) {
	if (number != 1 && number != 2) return;

	const double MOT_KP = 0.005; // коэффициент П-регулятора (можно подстроить)
	const double MOT_KD = 0.05;
	const double MOT_KI = 0;

	const int MOTOR_MAX_OUT = 100; // ограничение выхода

	static long lastEnc[3] = {0, 0, 0};
	static unsigned long lastTime[3] = {0, 0, 0};
	static int motorOut[3] = {0, 0, 0};
	// motorTargetSpeedEnc[number] = speed;
	// motorTargetRealSpeedEnc[number] = motorTargetSpeedEnc[number];

	unsigned long now = millis();
	long curEnc = (number == 1) ? Robot.enc_A : Robot.enc_B;

	// Инициализация при первом вызове для данного мотора
	if (lastTime[number] == 0) {
			lastTime[number] = now;
			lastEnc[number] = curEnc;
			// если требуется стартовая установка мощности, используем текущую motorOut (0)
	}

	unsigned long dt = now - lastTime[number];
	if (dt < 20) { // слишком мало времени — ждём накопления импульсов
			return;
	}

	long dEnc = curEnc - lastEnc[number];
	// измеренная скорость в импульсах в секунду
	float measured = (float)dEnc * 1000.0f / (float)dt;
	// Serial.print(measured);
	// Serial.print(" ");

	// ошибка в тех же единицах, что и параметр speed
	double e = (float)speed - measured;
	double d = e - motorDifferencial[number];
	motorDifferencial[number] = e;
	int deltaOut = MOT_KP*e + MOT_KD*d;

	// обновляем выход и ограничиваем
	int out = motorOut[number] + deltaOut;
	out = constrain(out,-MOTOR_MAX_OUT,MOTOR_MAX_OUT);
	if (motorTargetRealSpeedEnc[number]==0 && measured==0) out = 0;

	// Если целевая скорость 0, делать мягкую остановку — уменьшать к нулю
	// if (speed == 0 && abs(out) < 3) out = 0;

	// Применяем к мотору и сохраняем состояние
	// Serial.println(out);
	// Robot.motor(number, out);
	if (number==1) Robot.motor_A(out);
	else if (number==2) Robot.motor_B(out);
	motorOut[number] = out;

	// сохранить для следующего шага
	lastEnc[number] = curEnc;
	lastTime[number] = now;
}

int translateSpeedFronCmSecToParrotSec(long int speed) {
	speed *= PARROT_IN_ONE_FULL_ROTATION_WHEEL;
	return speed/(3.14*DIAMETER_WHEEL); 
}

void motorCreateTargetSpeedEncRun(int number, int speed=0) {
	if (number != 1 && number != 2) return;
	motorTargetSpeedEnc[number] = translateSpeedFronCmSecToParrotSec(speed);
}

void motorCreateTargetSpeedEncRun2(int left, int right) {
	motorTargetSpeedEnc[1] = translateSpeedFronCmSecToParrotSec(left);
	motorTargetSpeedEnc[2] = translateSpeedFronCmSecToParrotSec(right);
}

const int MOTOR_ENC_A = 10; // задержка по времени между ускорениями на единичку по скорости 
const int MOTOR_ENC_DENC = 150; //дельта изменения скорости 

void motorSpeedEncUpdateNumber(int number) {
	if (number != 1 && number != 2) return;
	static unsigned long lastTime2[3] = {0, 0, 0};
	unsigned long now = millis();
	// Инициализация при первом вызове для данного мотора
	if (lastTime2[number] == 0) lastTime2[number] = now;
	if (now-lastTime2[number]>MOTOR_ENC_A) {
		if (abs(motorTargetRealSpeedEnc[number]-motorTargetSpeedEnc[number])>MOTOR_ENC_DENC) {
			if (motorTargetRealSpeedEnc[number]>motorTargetSpeedEnc[number]) motorTargetRealSpeedEnc[number] -= MOTOR_ENC_DENC;
			if (motorTargetRealSpeedEnc[number]<motorTargetSpeedEnc[number]) motorTargetRealSpeedEnc[number] += MOTOR_ENC_DENC;
		}
		else if (abs(motorTargetRealSpeedEnc[number])<MOTOR_ENC_DENC+5) motorTargetRealSpeedEnc[number] = 0;
		lastTime2[number] = now;
	}
	// run motor
	motorTargetSpeedEncRun(number,motorTargetRealSpeedEnc[number]);
}


void motorSpeedEncUpdate() {
	motorSpeedEncUpdateNumber(1);
	motorSpeedEncUpdateNumber(2);
}

void testErrorEncDistanseRun(int a) {
  motorCreateTargetSpeedEncRun2(a,a);
  for (long int t = millis() + 4000; t>millis();) {
    motorSpeedEncUpdate();
  }
  Robot.enc_A = 0;
  Robot.enc_B = 0;
  motorCreateTargetSpeedEncRun2(0,0);
  for (long int t = millis() + 2000; t>millis();) {
    motorSpeedEncUpdate();
  }
  Serial.print(Robot.enc_A);
  Serial.print(" ");
  Serial.println(Robot.enc_B);
}


#include <PWM.h>
#include <LiquidCrystal_I2C.h>
#include <PID.h>

#define SampleCnt		50

#define VOLTAGR_PIN		A0
#define CURRENT1_PIN	A1
#define CURRENT2_PIN	A2
#define TARGET_VOLTAGE  8.0

#define SDA				A4
#define SCL				A5

#define MODE_PIN		2
#define UP_PIN			3
#define DOWN_PIN		4

#define VOLTAGE_PWM		9
#define CURRENT_PWM		10
#define PWM_RES			65535L
#define PWM_FREQ		10000L
#define PWM_MIN			(PWM_RES*0.1)
#define PWM_MAX			(PWM_RES*0.9)

const double VOLTAGE_GAIN = 10.0 / 1024;
const double CURRENT_GAIN = 2.5 / 1024;
double targetRatio = 1.0;

//LCD1602
LiquidCrystal_I2C Lcd(0x3f, 16, 2);

PID voltagePID(1000,20, 0, 50, -50, PWM_RES, -PWM_RES);
PID currentPID(1000,10, 0, 50, -50, PWM_RES, -PWM_RES);

volatile enum { FIXED, AUTO }mode;

bool keyPressed(char pin, char val);
double getRMS(char pin, double gain);
double getAvg(char pin, double gain);
void print2LCD(double current1, double current2, double voltage, double ratio);
void autoMode();
void fixedMode();
void changeMode();

void setup()
{
	InitTimersSafe();
	SetPinFrequency(VOLTAGE_PWM, PWM_FREQ);
	SetPinFrequency(CURRENT_PWM, PWM_FREQ);

	Lcd.init();
	Lcd.backlight();//开启背光
	Lcd.noBlink();//无光标
	Lcd.setCursor(0, 0);

	pinMode(VOLTAGR_PIN, INPUT);
	pinMode(CURRENT1_PIN, INPUT);
	pinMode(CURRENT2_PIN, INPUT);
	pinMode(UP_PIN, INPUT);
	pinMode(DOWN_PIN, INPUT);
	pinMode(MODE_PIN, INPUT);

	attachInterrupt(0, changeMode, LOW);

	mode = FIXED;

	Serial.begin(115200);
}


void loop()
{
	switch(mode)
	{
	case FIXED:
		fixedMode();
		break;
	case AUTO:
		autoMode();
		break;
	}
}

bool keyPressed(char pin, char val = LOW)
{
	if (digitalRead(pin) == val)
	{
		delay(10);
		if (digitalRead(pin) == val)
		{
			return true;
		}
	}
	return false;
}

// 获取均方根测量值，取SampleCnt次测量
double getRMS(char pin, double gain)
{
	double s = 0, t;
	for (uint32_t i = 0; i < SampleCnt; ++i)
	{
		t = analogRead(pin);
		s += t * t;
	}
	return sqrt(s / SampleCnt) * gain;
}

double getAvg(char pin, double gain)
{
	double s = 0, t;
	for (uint32_t i = 0; i < SampleCnt; ++i)
	{
		s += analogRead(pin);
	}
	return s / SampleCnt * gain;
}

void changeMode()
{
	static uint32_t lastTime = millis();
	if (millis() - lastTime < 500)
		return;
	lastTime = millis();

	switch (mode)
	{
	case FIXED:
		mode = AUTO;
	case AUTO:
		mode = FIXED;
	}
}

/*
显示屏样式：
0123456789012345
******************
*1.00A/1.00A=1.00*
*  8.00V   1.00  *
******************
*/
void print2LCD(double current1, double current2, double voltage, double ratio)
{
	static uint32_t last = millis();
	if (millis() - last < 500)
		return;
	last = millis();

	String lcdStr = String(current1, 2) + "A/" + String(current2, 2) + "A=" + String(current1 / current2, 2);
	Lcd.setCursor(0, 0);
	Lcd.print(lcdStr);
	lcdStr = "  " + String(voltage, 2) + "V   " + String(ratio, 2) + "  ";
	Lcd.setCursor(0, 1);
	Lcd.print(lcdStr);
}

void fixedMode()
{
	uint8_t loopCnt = 0;
	uint32_t voltagePos = PWM_RES * 0.3;
	uint32_t currentPos = PWM_RES * 0.3;
	while(mode == FIXED)
	{
		++loopCnt;

		double voltage = getAvg(VOLTAGR_PIN, VOLTAGE_GAIN);
		voltagePos += voltagePID.update(TARGET_VOLTAGE - voltage, voltage);;
		voltagePos = constrain(voltagePos, PWM_MIN, PWM_MAX);
		pwmWriteHR(VOLTAGE_PWM, voltagePos);

		if (loopCnt >= 10)
		{
			loopCnt = 0;

			double current1 = getAvg(CURRENT1_PIN, CURRENT_GAIN);
			double current2 = getAvg(CURRENT2_PIN, CURRENT_GAIN);
			double currentRatio = current1 / current2;

			currentPos += currentPID.update(targetRatio - currentRatio, currentRatio);
			currentPos = constrain(currentPos, PWM_MIN, PWM_MAX);
			pwmWriteHR(CURRENT_PWM, currentPos);

			print2LCD(current1, current2, voltage, targetRatio);
		}

		if(keyPressed(UP_PIN))
		{
			targetRatio = constrain(targetRatio + 0.1, 0.5, 2.0);
		}
		else if(keyPressed(DOWN_PIN))
		{
			targetRatio = constrain(targetRatio - 0.1, 0.5, 2.0);
		}
	}
}

void autoMode()
{
	uint8_t loopCnt = 0;
	uint32_t voltagePos = PWM_RES * 0.3;
	uint32_t currentPos = PWM_RES * 0.3;
	while (mode == FIXED)
	{
		++loopCnt;

		double voltage = getRMS(VOLTAGR_PIN, VOLTAGE_GAIN);
		voltagePos += voltagePID.update(TARGET_VOLTAGE - voltage, voltage);
		voltagePos = constrain(voltagePos, PWM_MIN, PWM_MAX);
		pwmWriteHR(VOLTAGE_PWM, voltagePos);

		if (loopCnt >= 10)
		{
			loopCnt = 0;

			double current1 = getRMS(CURRENT1_PIN, CURRENT_GAIN);
			double current2 = getRMS(CURRENT2_PIN, CURRENT_GAIN);
			double currentRatio = current1 / current2;
			double currentSum = current1 + current2;

			if(abs(1.5 - currentSum) < 0.1)
			{
				targetRatio = 1.5;
			}
			else
			{
				targetRatio = 1.0;
			}

			currentPos += currentPID.update(targetRatio - currentRatio, currentRatio);
			currentPos = constrain(currentPos, PWM_MIN, PWM_MAX);
			pwmWriteHR(CURRENT_PWM, currentPos);

			print2LCD(current1, current2, voltage, targetRatio);
		}
	}
}

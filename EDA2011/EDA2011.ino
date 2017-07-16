#include <PWM.h>
#include <LiquidCrystal_I2C.h>
#include <PID.h>

#define SampleCnt		100

#define VOLTAGR_PIN		A0
#define CURRENT1_PIN	A1
#define CURRENT2_PIN	A2
#define TARGET_VOLTAGE  8.0

#define SDA				A4
#define SCL				A5

#define UP_PIN			2
#define DOWN_PIN		3

#define VOLTAGE_PWM		9
#define CURRENT_PWM		10
#define PWM_RES			65535L
#define PWM_FREQ		40000L
#define PWM_MIN			(PWM_RES*0.1)
#define PWM_MAX			(PWM_RES*0.9)

const double VOLTAGE_GAIN = 5.0 / 255 * 10;
const double CURRENT_GAIN = 5.0 / 255 * 5;
double targetRatio = 1.0;

//LCD1602
LiquidCrystal_I2C Lcd(0x3f, 16, 2);

PID voltagePID(2000, 50, 0, 50, -50, PWM_RES, -PWM_RES);
PID currentPID(1000, 50, 0, 50, -50, PWM_RES, -PWM_RES);

bool keyPressed(char pin, char val);
double getRMS(char pin, double gain);
void upRatio();
void downRatio();
void print2LCD(double current1, double current2, double voltage, double ratio);

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

	attachInterrupt(0, upRatio, LOW);
	attachInterrupt(1, downRatio, LOW);

	Serial.begin(115200);
}


void loop()
{
	static uint8_t loopCnt = 0;
	static uint32_t voltagePos = PWM_RES * 0.3;
	static uint32_t currentPos = PWM_RES * 0.3;

	loopCnt += 1;

	double voltage = getRMS(VOLTAGR_PIN, VOLTAGE_GAIN);
	voltagePos += voltagePID.update(TARGET_VOLTAGE - voltage, voltage);
	voltagePos = constrain(voltagePos, PWM_MAX, PWM_MAX);
	pwmWriteHR(VOLTAGE_PWM, voltagePos);

	if(loopCnt >= 10)
	{
		loopCnt = 0;

		double current1 = getRMS(CURRENT1_PIN, CURRENT_GAIN);
		double current2 = getRMS(CURRENT2_PIN, CURRENT_GAIN);
		double currentRatio = current1 / current2;

		currentPos += currentPID.update(targetRatio - currentRatio, currentRatio);
		currentPos = constrain(currentPos, PWM_MAX, PWM_MIN);
		pwmWriteHR(CURRENT_PWM, currentPos);
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
	for (int i = 0; i < SampleCnt; ++i)
	{
		t = analogRead(pin) * gain;
		s += t * t;
	}
	return sqrt(s / SampleCnt);
}

void upRatio()
{
	static uint32_t lastTime = millis();
	if (millis() - lastTime < 500)
		return;
	lastTime = millis();

	targetRatio = constrain(targetRatio + 0.1, 0.5, 2.0);
}

void downRatio()
{
	static uint32_t lastTime = millis();
	if (millis() - lastTime < 500)
		return;
	lastTime = millis();

	targetRatio = constrain(targetRatio - 0.1, 0.5, 2.0);
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
	String lcdStr = String(current1, 2) + "/" + String(current2, 2) + "=" + String(current1 / current2, 2);
	Lcd.setCursor(0, 0);
	Lcd.print(lcdStr);
	lcdStr = "  " + String(voltage, 2) + "V   " + String(ratio, 2) + "  ";
	Lcd.setCursor(0, 1);
	Lcd.print(lcdStr);
}

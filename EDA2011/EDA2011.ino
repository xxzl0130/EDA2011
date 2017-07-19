#include <PWM.h>
#include <LiquidCrystal_I2C.h>
#include <PID.h>

#define SampleCnt		100L

#define VOLTAGR_PIN		A0
#define CURRENT1_PIN	A1
#define CURRENT2_PIN	A3
#define TARGET_VOLTAGE  8.0

#define SDA				A4
#define SCL				A5

#define MODE_PIN		2
#define UP_PIN			3
#define DOWN_PIN		4

// 9 10 for uno
#define VOLTAGE_PWM		11
#define CURRENT_PWM		12
#define PWM_RES			65536L
#define PWM_FREQ		1000L
#define PWM_MIN			(PWM_RES*0.02)
#define PWM_MAX			(PWM_RES*0.9)

#define CURRENT_CYCLE	5

const double VOLTAGE_GAIN = 10.0 / 1024;
const double CURRENT1_GAIN = (5.0 / 2.0812) / 1024;
const double CURRENT2_GAIN = (5.0 / 2.0134) / 1024;
const double CURRENT1_OFFSET = -0.0118;
const double CURRENT2_OFFSET = -0.0002;
double targetRatio = 1.0;

//LCD1602
LiquidCrystal_I2C Lcd(0x3f, 16, 2);

PID voltagePID(1000, 30, -200, 5, -5, PWM_RES*0.05, -PWM_RES*0.05);
PID currentPID(-500, -60, 50, 1, -1, PWM_RES*0.02, -PWM_RES*0.02);

volatile enum { FIXED, AUTO, STOP }mode;

bool keyPressed(char pin, char val = LOW);
double getRMS(char pin, double gain);
void print2LCD(double current1, double current2, double voltage, double ratio);
void changeMode();

void setup()
{
	Lcd.init();
	Lcd.backlight();//开启背光
	Lcd.noBlink();//无光标
	Lcd.setCursor(0, 0);

	pinMode(VOLTAGR_PIN, INPUT);
	pinMode(CURRENT1_PIN, INPUT);
	pinMode(CURRENT2_PIN, INPUT);
	pinMode(UP_PIN, INPUT_PULLUP);
	pinMode(DOWN_PIN, INPUT_PULLUP);
	pinMode(MODE_PIN, INPUT_PULLUP);
	pinMode(13, OUTPUT);

	InitTimersSafe();
	SetPinFrequency(VOLTAGE_PWM, PWM_FREQ);
	SetPinFrequency(CURRENT_PWM, PWM_FREQ);

	//attachInterrupt(MODE_PIN, changeMode, LOW);

	mode = FIXED;

	Serial.begin(115200);

	pwmWriteHR(VOLTAGE_PWM, PWM_RES * 0.15);
	pwmWriteHR(CURRENT_PWM, PWM_RES * 0.15);
}


void loop()
{
	static uint8_t loopCnt = 0;
	static int32_t voltagePos = PWM_RES * 0.17;
	static int32_t currentPos = PWM_RES * 0.13;
	static int32_t lastKeyTime = 0, det;
	static uint32_t currentTime, overloadCnt = 0;
	static double current1, current2, currentRatio, currentSum, voltage;

	++loopCnt;

	currentTime = millis();

	// 检查模式和比例设定
	if (keyPressed(MODE_PIN))
	{
		if (currentTime - lastKeyTime > 500)
		{
			changeMode();
			lastKeyTime = currentTime;
		}
	}
	if (mode == FIXED)
	{
		if (currentTime - lastKeyTime > 500)
		{
			if (keyPressed(UP_PIN))
			{
				targetRatio = constrain(targetRatio + 0.1, 0.5, 2.0);
				lastKeyTime = currentTime;
			}
			else if (keyPressed(DOWN_PIN))
			{
				targetRatio = constrain(targetRatio - 0.1, 0.5, 2.0);
				lastKeyTime = currentTime;
			}
		}
	}
	else if (mode == AUTO)
	{
		if (abs(1.5 - currentSum) < 0.1)
		{
			targetRatio = 0.5;
		}
		else
		{
			targetRatio = 1.0;
		}
	}
	else if (mode == STOP)
	{
		pwmWriteHR(VOLTAGE_PWM, 0);
		pwmWriteHR(CURRENT_PWM, 0);
		print2LCD(0, 0, 0, 0);
		return;
	}

	// 电压调节
	voltage = getRMS(VOLTAGR_PIN, VOLTAGE_GAIN);

	//if (abs(TARGET_VOLTAGE - voltage) > 0.05)
	{
		det = voltagePID.update(TARGET_VOLTAGE - voltage, voltage);
		voltagePos += det;
		voltagePos = constrain(voltagePos, PWM_MIN, PWM_MAX);
		pwmWriteHR(VOLTAGE_PWM, voltagePos);
		//Serial.println(String(TARGET_VOLTAGE - voltage) + "\t" + String(det) + "\t" + String(voltagePos));
	}

	// 电流环
	if (loopCnt >= CURRENT_CYCLE)
	{
		loopCnt = 0;

		current1 = getRMS(CURRENT1_PIN, CURRENT2_GAIN) + CURRENT1_OFFSET;
		current2 = getRMS(CURRENT2_PIN, CURRENT2_GAIN) + CURRENT2_OFFSET;
		current1 = constrain(current1, 0, 10);
		current2 = constrain(current2, 0, 10);
		currentRatio = current1 / current2;
		currentSum = current1 + current2;

		if (current1 + current2 > 4.5)
		{// 过流保护
			++overloadCnt;
			Serial.println(String(current1) + "\t" + String(current2) + "\t" + String(currentSum));
			if(overloadCnt > 10)
			{
				mode = STOP;
				print2LCD(0, 0, 0, 0);
				return;
			}
		}
		else
		{
			overloadCnt = 0;
		}

		//if (abs(currentRatio - targetRatio) > 0.01)
		{
			det = currentPID.update(targetRatio - currentRatio, currentRatio);
			currentPos += det;
			currentPos = constrain(currentPos, PWM_MIN, PWM_MAX);
			// 同时调节两路pwm
			//voltagePos -= det;
			//voltagePos = constrain(voltagePos, PWM_MIN, PWM_MAX);
			pwmWriteHR(CURRENT_PWM, currentPos);
			//pwmWriteHR(VOLTAGE_PWM, voltagePos);
		}
		print2LCD(current1, current2, voltage, targetRatio);
	}
}

bool keyPressed(char pin, char val)
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

void changeMode()
{
	static uint32_t lastTime = millis();
	static char led = 0;
	if (millis() - lastTime < 500)
		return;
	lastTime = millis();

	digitalWrite(13, led ^= 1);

	switch (mode)
	{
	case FIXED:
		mode = AUTO;
		break;
	case AUTO:
		mode = STOP;
		break;
	case STOP:
		mode = FIXED;
		break;
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
	lcdStr = "  " + String(voltage, 2) + "V   " + String(ratio, 2) + " ";
	switch (mode)
	{
	case FIXED:
		lcdStr += "F";
	case AUTO:
		lcdStr += "A";
	case STOP:
		lcdStr += "S";
	}
	Lcd.setCursor(0, 1);
	Lcd.print(lcdStr);
}

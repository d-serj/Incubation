#include <LiquidCrystal_I2C.h>
#include <i2c_SI7021.h>
#include <OneWire.h>
#include <PID\PID_v1.h>
#include <Wire.h>
#include <Bounce2\Bounce2.h>
#include <Encoder\Encoder.h>
#include <EEPROM2\EEPROM2.h>
#include "RTClib.h"

//*** BEGIN PINS ***
#define DS18B20_PIN    10										// Пин датчика температуры DS18B20
#define ENCODER_BUTTON 4										// Пин кнопки энкодера
#define ENC_PIN1       5
#define ENC_PIN2       6
//*** END PINS ***

// TODO: another defines
#define SENSORS_DELAY         900								// ms
#define EEPROM_WRITE_DELAY    1800								// Задержка записи EEPROM в секундах

uint8_t currentPeriod = 0;										// Текущий преиод инкубации, зависит от parameter[].endDay дня инкубации
float tempDS18B20;												// Температура с датчика DS18B20
float tempSI7021;												// Температура с датчика Si7021
float humiSI7021;												// Влажность с датчика Si7021
float currentNeedTemp;											// Нужная в текущий момент температура
uint8_t currentNeedHum;											// Нужная в текущий момент влажность

/**** PID ****/
double Setpoint;												// Необходимая температура
double Input;													// Входные температуры
double Output;													// Выход от 0 до 1000
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/*************/

LiquidCrystal_I2C lcd(0x27, 20, 4);
SI7021 si7021;
OneWire dsTemp(DS18B20_PIN);
DS3231 RTC;
Bounce encoderButton = Bounce();							// Антидребезг кнопка энкодера
Encoder myEnc(ENC_PIN1, ENC_PIN2);


struct parameters
{
	//uint8_t period;										// Номер периода
	uint8_t endDay;											// День окончания периода
	float neededTemp;										// Необходимая температура
	uint8_t neededHum;										// Необходимая влажность
	uint8_t eggTurns;										// Нужен ли поворот яиц?
	uint8_t aeration;										// Нужно ли проветривание?
	uint8_t aerationTime;									// Время проветривания, мин
};

// Reading byte for PROGMEM : pgm_read_byte_near(&parameter[1].endDay)
// Reading float for PROGMEM: pgm_read_float_near(&parameter[1].needTemp)
// Structures of parameters of incubation
const parameters parameter[4] PROGMEM =
{
/* endDay |  temp |  hum% | turn | aerat | aeratTime */
	{11,     37.9,    66,     4,     0,     0},
	{17,     37.3,    53,     4,     2,     5},
	{19,     37.3,    47,     4,     2,     20},
	{21,     37.0,    66,     0,     2,     5}
};

// Streaming
template<class T> inline Print &operator << (Print &obj, T arg) 
{ 
	obj.print(arg);
	return obj; 
}


void setup(void)
{
	Serial.begin(9600);
	initLCD();
	initSI7021();
	myPID.SetMode(AUTOMATIC);
	Wire.begin();
	RTC.begin();
	
	pinMode(ENCODER_BUTTON, INPUT);
	encoderButton.attach(ENCODER_BUTTON);									// Антидребезг кнопка энкодера
	encoderButton.interval(5);												// 5 ms антидребезг кнопки
	
	// TEST EEPROM
	/*
	EEPROM_write(1, (float)parameter[0].needTemp);
	delay(100);
	EEPROM_read(1, temp);
	lcd.setCursor(2, 1);
	lcd << temp;
	delay(100000);
	*/
}


void loop(void)
{
	DateTime now = RTC.now();
	static uint32_t startTimeUnix = now.unixtime() / 86400L;					// Дней с 1970
	static uint16_t currentDay;
	uint8_t currentDay_EEPROM;
	uint8_t currentPeriod_EEPROM;
	static uint32_t delayTime = now.unixtime();
	static bool encoderButtonPress = false;

	/*******************ОТВЕТСТВЕННЫЙ БЛОК ТРЕБУЮЩИЙ ПРОВЕРКИ****************/
	currentDay = startTimeUnix - now.unixtime() / 86400L;
	
	if (now.unixtime() - delayTime > EEPROM_WRITE_DELAY)
	{
		EEPROM_write(1, currentDay);
		// TODO
		delayTime = now.unixtime();
	}

	EEPROM_read(1, currentDay_EEPROM);
	EEPROM_read(5, currentPeriod);
	if (currentDay_EEPROM == (uint8_t)parameter[currentPeriod].endDay)
	{
		currentNeedTemp = parameter[currentPeriod].neededTemp;
		currentNeedHum  = parameter[currentPeriod].neededHum;
		
		// Проблема: при исчезновении питания и после перезагрузки currentPeriod будет сбиваться
		currentPeriod++;

		if (currentPeriod > 3)
			currentPeriod = 0;

		EEPROM_write(5, (uint8_t)currentPeriod);
	}
	/********************КОНЕЦ ОТВЕТСТВЕННОГО БЛОКА ТРЕБУЮЩЕГО ПРОВЕРКИ*********/


	lcd.setCursor(3, 1);
	lcd << now.format("hh:mm:ss\0");

	getSensors();
	thermostat();

	/*
	encoderButton.update();
	if (encoderButton.rose())
	{
		encoderButtonPress = !encoderButtonPress;
	}

	if (encoderButtonPress)
	{
		mainScreenProcessing(now, startTimeUnix);
	}
	else
	{
		menuScreenProcessing();
	}
	*/
}


/* Initialize LCD and show first message on screen */
void initLCD(void)
{
	lcd.init();
	//lcd.backlight();

	/*lcd.setCursor(5, 1);
	lcd.print("HOME");
	lcd.setCursor(5, 0);
	lcd.print("INCUBATOR");*/

	//delay(5000);
	lcd.clear();
}


/* Initialize Si7021 */
void initSI7021(void)
{
	if (si7021.initialize())
		Serial.println("Sensor found!");
	else
	{
		Serial.println("Sensor si7021 is missing");
		delay(1000);
	}
}


void getSensors(void)
{
	static unsigned long previousMillis = 0;
	unsigned long currentMillis = millis();

	if (currentMillis - previousMillis >= SENSORS_DELAY)
	{
		getTempDS18B20();													// interval 2000 ms
		getSi7021();														// interval 1000 ms
		previousMillis = currentMillis;
	}
}

/* Получаем температуру с датчика DS18B20 с задержкой 2000 миллисекунд при SENSORS_DELAY 900 */
void getTempDS18B20()
{
	byte buff[9];
	static bool requestTemp = TRUE;												// Get off delay 750ms to read data from ds18b20

	if (requestTemp)
	{
		dsTemp.reset();
		dsTemp.write(0xCC);
		dsTemp.write(0x44);

		requestTemp = !requestTemp;

		return;
	}

	else
	{
		dsTemp.reset();
		dsTemp.write(0xCC);
		dsTemp.write(0xBE);
		dsTemp.read_bytes(buff, 9);

		requestTemp = !requestTemp;

		if (OneWire::crc8(buff, 8) == buff[8])
		{
			// Конвертация сырых данных
			tempDS18B20 = (float)((int)buff[0] | (((int)buff[1]) << 8)) * 0.0625 + 0.03125;
			//Serial.println(f_temp1, DEC);

			return;
		}

		else
		{
			tempDS18B20 = 0;
			//Serial.println("Error reading data DS18B20");
		}
	}
}


void getSi7021()
{
	si7021.getHumidity(humiSI7021);
	si7021.getTemperature(tempSI7021);
	si7021.triggerMeasurement();

	if (tempSI7021 <= 0 || tempSI7021 > 128.9)
	{
		Serial.println("Error reding data SI7021");
	}
}


uint8_t thermostat()
{
	Input = (tempDS18B20 + tempSI7021) / 2;
	Setpoint = 100;
	myPID.Compute();
	//Serial << "Input is: " << Input << " and Output is: " << map(Output, 0, 255, 0, 1000) << '\n';

	return 0;
}


/* Главный экран */
void mainScreenProcessing(DateTime time, unsigned long _startUnixTime)
{
	lcd.setCursor(0, 0);
	lcd << "T1   : " << tempDS18B20 << "C";
	lcd.setCursor(0, 1);
	lcd << "T2   : " << tempSI7021 << "C";
	lcd.setCursor(0, 2);
	lcd << "H    : " << humiSI7021 << "%";
	lcd.setCursor(0, 3);
	lcd << "TIME : " <<  time.hour() << ":" << time.minute() << ":" << time.second();        // Время работы в секундах
	/* time.unixtime() - _startUnixTime  сколько времени прошло */
}


/* Меню */
void menuScreenProcessing(void)
{
	static int32_t f_encPosition;
	static uint16_t lcdClearTime = millis();
	static int8_t menuPosition = 0;

	readEncoder(f_encPosition);
	//delay(10);
	menuPosition += f_encPosition;

	if (menuPosition < 0)
		menuPosition = 3;
	else if (menuPosition > 3)
		menuPosition = 0;
	
	Serial << menuPosition << "\n";

	if (lcdClearTime - millis() > 200)
	{
		lcd.clear();
		lcdClearTime = millis();
	}

	switch (menuPosition)
	{
	case 0:
		lcd.setCursor(0, 0);
		lcd << "1";
	case 1:
		lcd.setCursor(0, 0);
		lcd << "2";
	case 2:
		lcd.setCursor(0, 0);
		lcd << "3";
	case 3:
		lcd.setCursor(0, 0);
		lcd << "4";
	}
}


/* Читаем энкодер */
void readEncoder(int32_t &encoderPosition)
{
	static int32_t encOldPosition;
	int32_t currentEncPosition = myEnc.read();

	if (encOldPosition != currentEncPosition)
	{
		if (currentEncPosition >= encOldPosition + 2)
		{
			encoderPosition = 1;
		}

		else if (currentEncPosition <= encOldPosition - 2)
		{
			encoderPosition = -1;
		}

		encOldPosition = currentEncPosition;
	}

	else
		encoderPosition = 0;
}

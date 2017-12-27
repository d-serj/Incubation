#include <LiquidCrystal_I2C.h>
#include <i2c_SI7021.h>
#include <OneWire.h>
#include <PID\PID_v1.h>
#include <Wire.h>
#include "RTClib.h"

//*** BEGIN PINS ***
#define DS18B20_PIN 10								// Пин датчика температуры DS18B20
//*** END PINS ***

// TODO: another defines
#define SENSORS_DELAY 900							// ms
// END

// Streaming
template<class T> inline Print &operator << (Print &obj, T arg) { obj.print(arg); return obj; }

LiquidCrystal_I2C lcd(0x27, 20, 4);
SI7021 si7021;
OneWire dsTemp(DS18B20_PIN);
DS3231 RTC;

double Setpoint;									// Необходимая температура
double Input;										// Входные температуры
double Output;										// Выход от 0 до 1000
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

struct parameters
{
	uint8_t period;									// Номер периода
	uint8_t endDay;									// День окончания периода
	float needTemp;									// Необходимая температура
	uint8_t needHum;								// Необходимая влажность
	uint8_t eggTurns;								// Нужен ли поворот яиц?
	uint8_t aeration;								// Нужно ли проветривание?
	uint8_t aerationTime;							// Время проветривания, мин
};

// Reading byte for PROGMEM : pgm_read_byte_near(&parameter[1].endDay)
// Reading float for PROGMEM: pgm_read_float_near(&parameter[1].needTemp)
// Structures of parameters of incubation
const parameters parameter[4] PROGMEM  = 
{ 
/*  per | endDay |  temp |  hum% | turn | aerat | aeratTime */  
	{1,     11,     37.9,    66,     4,     0,     0},
	{2,     17,     37.3,    53,     4,     2,     5},
	{3,     19,     37.3,    47,     4,     2,     20},
	{4,     21,     37.0,    66,     0,     2,     5}
};

void setup(void)
{
	Serial.begin(9600);
	initLCD();
	initSI7021();
	myPID.SetMode(AUTOMATIC);
	Wire.begin();
	RTC.begin();

	
}

void loop(void)
{
	DateTime now = RTC.now();

	static unsigned long startTimeUnix = now.unixtime();

	static float temp1, temp2, humi;
	static unsigned long previousMillis = 0;
	unsigned long currentMillis = millis();

	if (currentMillis - previousMillis >= SENSORS_DELAY)
	{
		getTempDS18B20(temp1);								// interval 2000 ms
		getSi7021(temp2, humi);								// interval 1000 ms
		previousMillis = currentMillis;
	}

	thermostat(temp1, temp2);

	lcd.setCursor(0, 0);
	lcd << "TEMP 1   : " << temp1 << "C";
	lcd.setCursor(0, 1);
	lcd << "TEMP 2   : " << temp2 << "C";
	lcd.setCursor(0, 2);
	lcd << "HUMI     : " << humi << "%";
	lcd.setCursor(0, 3);
	lcd << "WORK TIME: " << now.unixtime() - startTimeUnix << " s";        // Время работы в секундах
}

/* Initialize LCD and show first message on screen */
void initLCD(void)
{
	lcd.init();
	//lcd.backlight();

	lcd.setCursor(5, 1);
	lcd.print("HOME");
	lcd.setCursor(5, 0);
	lcd.print("INCUBATOR");

	delay(5000);
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


/* Получаем температуру с датчика DS18B20 с задержкой 2000 миллисекунд при SENSORS_DELAY 900 */
void getTempDS18B20(float &f_temp1)
{
	byte buff[9];
	static bool requestTemp = TRUE;						// Get off delay 750ms to read data from ds18b20

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
			f_temp1 = (float)((int)buff[0] | (((int)buff[1]) << 8)) * 0.0625 + 0.03125;
			Serial.println(f_temp1, DEC);

			return;
		}

		else
		{
			f_temp1 = -100;
			Serial.println("Error reading data DS18B20");
		}
	}
}

void getSi7021(float &f_temp2, float &f_humi)
{
	si7021.getHumidity(f_humi);
	si7021.getTemperature(f_temp2);
	si7021.triggerMeasurement();

	if (f_temp2 <= 0 || f_temp2 > 128.9)
	{
		Serial.println("Error reding data SI7021");
	}
}

uint8_t thermostat(float &temp1, float &temp2)
{
	Input = (temp1 + temp2) / 2;
	Setpoint = 100;
	myPID.Compute();
	//Serial << "Input is: " << Input << " and Output is: " << map(Output, 0, 255, 0, 1000) << '\n';

	return 0;
}

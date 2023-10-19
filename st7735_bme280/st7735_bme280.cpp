/*!
	@file st7735_bme280.cpp
	@author Gavin Lyons (TFT Display Driver), Darren Horrocks (BME 280 Driver), Lumir Vanek (Integration)
	@brief Two ST7735_TFT displays + BME280 sensor.
*/

// Section ::  libraries 
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "./st7735/ST7735_TFT.hpp"

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#include "bme280.h"


// Section :: Defines   
//  Test timing related defines 
#define TEST_DELAY1 1000
#define TEST_DELAY2 2000
#define TEST_DELAY5 5000
#define CLOCK_DISPLAY_TIME 20

// Section :: Globals 
ST7735_TFT myTFT1;
ST7735_TFT myTFT2;

bool bTestFPS = true; /**< turn on frame rate per second test , set true for ON */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


 /* Two ST7735 TFT displays, BME280 sensor and Raspberry PI Pico

    NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
    GPIO (and therefore I2C) cannot be used at 5v.

    You will need to use a level shifter on the I2C lines if you want to run the
    board at 5v.

    Connections on Raspberry Pi Pico board, other boards may vary.

    GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP12 (pin 16)) -> SDA on BME280
    board
    GPIO PICO_DEFAULT_I2C_SCK_PIN (on Pico this is GP13 (pin 17)) -> SCL on BME280
    BMP280 board
    3.3v (pin 36) -> VCC on BME280 board
    GND (pin 38)  -> GND on BME280 board
 */

#define BME280_ADDRESS1 _u(0x76) 
#define BME280_ADDRESS2 _u(0x77) 

// Overtake default setting
#define PICO_I2C_SDA_PIN 12
#define PICO_I2C_SCL_PIN 13

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  Section ::  Function Headers 

void SetupTFT1(void);  // setup + user options
void SetupTFT2(void);  // setup + user options
void Test1(ST7735_TFT);  // Print out  fonts 1-6
void EndTests(ST7735_TFT);

//  Section ::  MAIN loop

int main(void) 
{
	SetupTFT1();
	SetupTFT2();
	Test1(myTFT1);
	Test1(myTFT2);
	TFT_MILLISEC_DELAY(TEST_DELAY5);

	stdio_init_all();

//#if !defined(i2c_default) || !defined(PICO_I2C_SDA_PIN) || !defined(PICO_I2C_SCL_PIN)
//    #warning i2c / bmp280_i2c example requires a board with I2C pins
//        puts("Default I2C pins were not defined");
//#else
    // useful information for picotool
    bi_decl(bi_2pins_with_func(PICO_I2C_SDA_PIN, PICO_I2C_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("BMP280 I2C"));

    printf("Hello, BMP280! Reading temperaure and pressure values from sensor...\n");

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C_SDA_PIN);
    gpio_pull_up(PICO_I2C_SCL_PIN);

    // configure BME280
	BME280 sensor0 = BME280(0, BME280_ADDRESS1);
    bool connected0 = sensor0.begin();
	BME280 sensor1 = BME280(0, BME280_ADDRESS2);
    bool connected1 = sensor1.begin();

	char strTemp[] = "Temp. dgC";
	char strHum[] = "Humid. Pct";
	char strPres[] = "Press. hPa";
	
	char strTempValue1[10];
	char strTempValue2[10];

	char strHumValue1[10];
	char strHumValue2[10];

	char strPresValue1[10];
	char strPresValue2[10];
	
	while (true) 
	{
		if (connected0) 
		{
			float temperature = sensor0.readTemperature();
			float humidity = sensor0.readHumidity();
			float pressure = sensor0.readPressure() / 100.0f;
			float altitude = sensor0.readAltitude();
			float seaLevelPressure = sensor0.seaLevelForAltitude(altitude, pressure);

			printf("Temperature1 = %.2f C\n", temperature);
			printf("Pressure1 uncorrected = %.3f hPa\n", pressure);
			printf("Pressure1 = %.3f hPa\n", seaLevelPressure);
			printf("Humidity1 = %.1f %\n", humidity);

			sprintf(strTempValue1, "%.1f", temperature);
			sprintf(strHumValue1, "%.1f", humidity);
			sprintf(strPresValue1, "%.1f", seaLevelPressure);

			uint16_t colorTemp = ST7735_WHITE;
			if(temperature < 0)
			{
				colorTemp = ST7735_CYAN;
			}
			else if(temperature < 10)
			{
				colorTemp = ST7735_BLUE;
			}
			else if(temperature < 20)
			{
				colorTemp = ST7735_YELLOW;
			}
			else if(temperature < 30)
			{
				colorTemp = ST7735_MAGENTA;
			}
			else
			{
				colorTemp = ST7735_RED;
			}

			myTFT1.TFTfillScreen(ST7735_BLACK);
			myTFT1.TFTFontNum(myTFT1.TFTFont_Default);

			myTFT1.TFTdrawText(0, 10, strTemp, ST7735_WHITE, ST7735_BLACK, 2);
			myTFT1.TFTdrawText(0, 65, strHum, ST7735_WHITE, ST7735_BLACK, 2);
			myTFT1.TFTdrawText(0, 120, strPres, ST7735_WHITE, ST7735_BLACK, 2);
			
			myTFT1.TFTFontNum(myTFT1.TFTFont_Wide);
			
			myTFT1.TFTdrawText(19, 35, strTempValue1, colorTemp, ST7735_BLACK, 2);
			myTFT1.TFTdrawText(19, 88, strHumValue1, ST7735_GREEN, ST7735_BLACK, 2);
			myTFT1.TFTdrawText(5, 144, strPresValue1, ST7735_CYAN, ST7735_BLACK, 2);
			
			myTFT1.TFTFontNum(myTFT1.TFTFont_Default);
			
			myTFT1.TFTdrawFastHLine(0, 55, 128, ST7735_YELLOW);
			myTFT1.TFTdrawFastHLine(0, 112, 128, ST7735_YELLOW);
		}

		if (connected1) 
		{
			float temperature = sensor1.readTemperature();
			float humidity = sensor1.readHumidity();
			float pressure = sensor1.readPressure() / 100.0f;
			float altitude = sensor1.readAltitude();
			float seaLevelPressure = sensor1.seaLevelForAltitude(altitude, pressure);

			printf("Temperature2 = %.2f C\n", temperature);
			printf("Pressure2 uncorrected = %.3f hPa\n", pressure);
			printf("Pressure2 = %.3f hPa\n", seaLevelPressure);
			printf("Humidity2 = %.1f %\n", humidity);

			sprintf(strTempValue2, "%.1f", temperature);
			sprintf(strHumValue2, "%.1f", humidity);
			sprintf(strPresValue2, "%.1f", seaLevelPressure);

			uint16_t colorTemp = ST7735_WHITE;
			if(temperature < 0)
			{
				colorTemp = ST7735_CYAN;
			}
			else if(temperature < 10)
			{
				colorTemp = ST7735_BLUE;
			}
			else if(temperature < 20)
			{
				colorTemp = ST7735_YELLOW;
			}
			else if(temperature < 30)
			{
				colorTemp = ST7735_MAGENTA;
			}
			else
			{
				colorTemp = ST7735_RED;
			}

			myTFT2.TFTfillScreen(ST7735_BLACK);
			myTFT2.TFTFontNum(myTFT2.TFTFont_Default);

			myTFT2.TFTdrawText(0, 10, strTemp, ST7735_WHITE, ST7735_BLACK, 2);
			myTFT2.TFTdrawText(0, 65, strHum, ST7735_WHITE, ST7735_BLACK, 2);
			myTFT2.TFTdrawText(0, 120, strPres, ST7735_WHITE, ST7735_BLACK, 2);
			
			myTFT2.TFTFontNum(myTFT2.TFTFont_Wide);
			
			myTFT2.TFTdrawText(19, 35, strTempValue2, colorTemp, ST7735_BLACK, 2);
			myTFT2.TFTdrawText(19, 88, strHumValue2, ST7735_GREEN, ST7735_BLACK, 2);
			myTFT2.TFTdrawText(5, 144, strPresValue2, ST7735_CYAN, ST7735_BLACK, 2);
			
			myTFT2.TFTFontNum(myTFT2.TFTFont_Default);
			
			myTFT2.TFTdrawFastHLine(0, 55, 128, ST7735_YELLOW);
			myTFT2.TFTdrawFastHLine(0, 112, 128, ST7735_YELLOW);
		}

		TFT_MILLISEC_DELAY(TEST_DELAY5);
    }

	EndTests(myTFT1);
	EndTests(myTFT2);
	return 0;
}
// *** End OF MAIN **

//  Section ::  Function Space 

/*!
	@brief setup + user options
*/
void SetupTFT1(void)
{

	TFT_MILLISEC_DELAY(TEST_DELAY1);
	
//*************** USER OPTION 0 SPI_SPEED + TYPE ***********
	bool bhardwareSPI = true; // true for hardware spi, 
	
	if (bhardwareSPI == true) { // hw spi
		uint32_t TFT_SCLK_FREQ =  8000 ; // Spi freq in KiloHertz , 1000 = 1Mhz
		myTFT1.TFTInitSPIType(TFT_SCLK_FREQ, spi0); 
	} else { // sw spi
		myTFT1.TFTInitSPIType(); 
	}
//**********************************************************

// ******** USER OPTION 1 GPIO *********
// NOTE if using Hardware SPI clock and data pins will be tied to 
// the chosen interface eg Spi0 CLK=18 DIN=19)
	int8_t SDIN_TFT = 19; 
	int8_t SCLK_TFT = 18; 
	int8_t DC_TFT = 3; // RS just means Register-Select. Some people call it DC for Data-Command. 
	int8_t CS_TFT = 17; //2 ;  
	int8_t RST_TFT = 15; //17;
	myTFT1.TFTSetupGPIO(RST_TFT, DC_TFT, CS_TFT, SCLK_TFT, SDIN_TFT);
//**********************************************************

// ****** USER OPTION 2 Screen Setup ****** 
	uint8_t OFFSET_COL = 0;  // 2, These offsets can be adjusted for any issues->
	uint8_t OFFSET_ROW = 0; // 3, with screen manufacture tolerance/defects
	uint16_t TFT_WIDTH = 128;// Screen width in pixels
	uint16_t TFT_HEIGHT = 160; // Screen height in pixels
	myTFT1.TFTInitScreenSize(OFFSET_COL, OFFSET_ROW , TFT_WIDTH , TFT_HEIGHT);
// ******************************************

// ******** USER OPTION 3 PCB_TYPE  **************************
	myTFT1.TFTInitPCBType(myTFT1.TFT_ST7735R_Red); // pass enum,4 choices,see README
//**********************************************************
}

/*!
	@brief setup + user options
*/
void SetupTFT2(void)
{

	TFT_MILLISEC_DELAY(TEST_DELAY1);
	
//*************** USER OPTION 0 SPI_SPEED + TYPE ***********
	bool bhardwareSPI = true; // true for hardware spi, 
	
	if (bhardwareSPI == true) { // hw spi
		uint32_t TFT_SCLK_FREQ =  8000 ; // Spi freq in KiloHertz , 1000 = 1Mhz
		myTFT2.TFTInitSPIType(TFT_SCLK_FREQ, spi1);
	} else { // sw spi
		myTFT2.TFTInitSPIType(); 
	}
//**********************************************************

// ******** USER OPTION 1 GPIO *********
// NOTE if using Hardware SPI clock and data pins will be tied to 
// the chosen interface eg Spi1 CLK=10 DIN=11)
	int8_t SDIN_TFT = 11; 
	int8_t SCLK_TFT = 10; 
	int8_t DC_TFT = 4; // RS just means Register-Select. Some people call it DC for Data-Command. 
	int8_t CS_TFT = 9; //2 ;  
	int8_t RST_TFT = 16; //17;
	myTFT2.TFTSetupGPIO(RST_TFT, DC_TFT, CS_TFT, SCLK_TFT, SDIN_TFT);
//**********************************************************

// ****** USER OPTION 2 Screen Setup ****** 
	uint8_t OFFSET_COL = 0;  // 2, These offsets can be adjusted for any issues->
	uint8_t OFFSET_ROW = 0; // 3, with screen manufacture tolerance/defects
	uint16_t TFT_WIDTH = 128;// Screen width in pixels
	uint16_t TFT_HEIGHT = 160; // Screen height in pixels
	myTFT2.TFTInitScreenSize(OFFSET_COL, OFFSET_ROW , TFT_WIDTH , TFT_HEIGHT);
// ******************************************

// ******** USER OPTION 3 PCB_TYPE  **************************
	myTFT2.TFTInitPCBType(myTFT2.TFT_ST7735R_Red); // pass enum,4 choices,see README
//**********************************************************
}


/*!
	@brief print out fonts 1-6
*/
void Test1(ST7735_TFT myTFT) 
{

	char teststr1[] = "Default 1";
	char teststr2[] = "THICK 2";
	char teststr3[] = "Seven 3";
	char teststr4[] = "WIDE 4";
	char teststr5[] = "Tiny 5";
	char teststr6[] = "Home 6";
	
	myTFT.TFTfillScreen(ST7735_BLACK);
	
	myTFT.TFTFontNum(myTFT.TFTFont_Default);
	myTFT.TFTdrawText(5, 5, teststr1, ST7735_WHITE, ST7735_BLACK, 2);
	myTFT.TFTFontNum(myTFT.TFTFont_Thick);
	myTFT.TFTdrawText(5, 25, teststr2, ST7735_GREEN, ST7735_BLACK, 2);
	myTFT.TFTFontNum(myTFT.TFTFont_Seven_Seg);
	myTFT.TFTdrawText(5, 45, teststr3, ST7735_BLUE, ST7735_BLACK, 2);
	myTFT.TFTFontNum(myTFT.TFTFont_Wide);
	myTFT.TFTdrawText(5, 65, teststr4, ST7735_CYAN, ST7735_BLACK, 2);
	myTFT.TFTFontNum(myTFT.TFTFont_Tiny);
	myTFT.TFTdrawText(5, 85, teststr5, ST7735_RED, ST7735_BLACK, 2);
	myTFT.TFTFontNum(myTFT.TFTFont_HomeSpun);
	myTFT.TFTdrawText(5, 105, teststr6, ST7735_YELLOW, ST7735_BLACK, 2);
	
	//myTFT.TFTfillScreen(ST7735_BLACK);
	myTFT.TFTFontNum(myTFT.TFTFont_Default);
}

// *************** EOF ****************

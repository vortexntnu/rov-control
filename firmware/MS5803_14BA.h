// MS5803_14BA.h

#ifndef _MS5803_14BA_h
#define _MS5803_14BA_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class MS5803_14BA
{
	public:
		MS5803_14BA();// default constructor with Resolution 512
		MS5803_14BA(uint16_t resolution);
		boolean initialize(boolean Verbose);
		void reset();
		void read();
		
		float getTemperature();
		float getPreassure();
		
		uint32_t getD1();
		uint32_t getD2();
		
		
	private:
		float mbar_;
		float tempC_;
		
		uint32_t	D1_;
		uint32_t	D2_;
		
		int32_t mbarInt_;
		
		uint16_t Resolution_;
		static uint16_t sensorCoefficients_[8];
		
		unsigned char MS5803_CRC(uint16_t n_prom[]);
		uint32_t MS5803_ADC(int8_t commandAdc);
		

		
	
};


// enums:
// i2C_address might be 0x77 if R6 is added and R5 is removed from the circuit board.
enum MS5803_14BAI2CAddress{
	MS5803_I2C_ADDRESS	= 0x76,
	PROM_ADDRESS_START	= 0xA0
};

//comands for different operations, the PROM Read comand is not included.
enum MS5803_14BACommands {
	COMMAND_RESET		= 0x1E,
	COMMAND_ADC_READ	= 0x00,
	COMMAND_D1_256		= 0x40,
	COMMAND_D1_512		= 0x42,
	COMMAND_D1_1024		= 0x44,
	COMMAND_D1_2048		= 0x46,
	COMMAND_D1_4096		= 0x48,
	COMMAND_D2_256		= 0x50,
	COMMAND_D2_512		= 0x52,
	COMMAND_D2_1024		= 0x54,
	COMMAND_D2_2048		= 0x56,
	COMMAND_D2_4096		= 0x58
	};



#endif


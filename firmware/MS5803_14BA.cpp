// 
// 
//
//TODO: should change to static_cast<...> instead of (...)
// NULL should be removed and inserted 0 instead.
//
// For member variable access.
// http://geosoft.no/development/cpppractice.html
// http://geosoft.no/development/cppstyle.html#Variables
// http://stackoverflow.com/questions/10198046/c-member-variables

#include "MS5803_14BA.h"
#include <Wire.h>

uint16_t MS5803_14BA::sensorCoefficients_[] = {};
	
MS5803_14BA::MS5803_14BA(){
	Resolution_ = 512;
}

MS5803_14BA::MS5803_14BA(uint16_t resolution){
	Resolution_ = resolution;
}

void MS5803_14BA::reset(){
	Wire.beginTransmission(MS5803_I2C_ADDRESS);
	Wire.write(COMMAND_RESET);
	Wire.endTransmission();
	delay(10);
}

boolean MS5803_14BA::initialize(boolean Verbose){
	reset();
	
	if (Verbose){
		if (Resolution_ == 256 | Resolution_ == 512 | Resolution_ == 1024 | Resolution_ == 2048 | Resolution_ == 4096){
			Serial.print("Oversampling setting: ");
			Serial.println(Resolution_);
		}
		else
		{
			Serial.println("*******************************************");
			Serial.println("Error: specify a valid oversampling value.");
			Serial.println("Choices are: 256, 512, 1024, 2048, 4096.");
			Serial.println("*******************************************");
		}
	}
	
	// Read calibration constants form PROM
	byte HighByte,MiddleByte,LowByte; // to hold the result from PROM and other byte data in initialize
	for(int i = 0; i < 8; i++){
		Wire.beginTransmission(MS5803_I2C_ADDRESS);
		Wire.write(PROM_ADDRESS_START +(i * 2));
		Wire.endTransmission();
		Wire.requestFrom(MS5803_I2C_ADDRESS, 2);
		while(Wire.available()){
			HighByte = Wire.read();
			LowByte = Wire.read();
		}
		sensorCoefficients_[i] = ((static_cast<uint16_t>(HighByte) << 8) + LowByte);
		if(Verbose){
			Serial.print("C");
			Serial.print(i);
			Serial.print(" = ");
			Serial.println(sensorCoefficients_[i]);
			delay(10);
		}
	}
	
	// CRC error checking
	uint8_t p_crc = sensorCoefficients_[7];
	uint8_t n_crc = MS5803_CRC(sensorCoefficients_);
	
	if (Verbose){
		Serial.print("p_crc: ");
		Serial.println(p_crc);
		Serial.print("n_crc: ");
		Serial.println(n_crc);
	}
	
	if (p_crc != n_crc){
		return false;
	}
	
	return true;
	

}

void MS5803_14BA::read(){
	
	// casting is used extensively in this code to reduce risk of overflow 
	if (Resolution_ == 256){
		D1_ = MS5803_ADC(COMMAND_D1_256);
		D2_ = MS5803_ADC(COMMAND_D2_256);	
	}
	else if (Resolution_ == 512){
		D1_ = MS5803_ADC(COMMAND_D1_512);
		D2_ = MS5803_ADC(COMMAND_D2_512);
	}
	else if (Resolution_ == 1024){
		D1_ = MS5803_ADC(COMMAND_D1_1024);
		D2_ = MS5803_ADC(COMMAND_D2_1024);		
	}
	else if (Resolution_ == 2048){
		D1_ = MS5803_ADC(COMMAND_D1_2048);
		D2_ = MS5803_ADC(COMMAND_D2_2048);		
	}
	else if(Resolution_ == 4096){
		D1_ = MS5803_ADC(COMMAND_D1_4096);
		D2_ = MS5803_ADC(COMMAND_D2_4096);		
	}
	
	// calculate temp difference between actual and ref temp 
	int32_t dT = static_cast<int32_t>(D2_) - ( static_cast<int32_t>(sensorCoefficients_[5])* pow(2,8));
	
	// calculate actual temp
	int32_t TEMP = 2000 +static_cast<int64_t>(dT) * (sensorCoefficients_[6] / pow(2,23));
	
	TEMP = static_cast<int32_t>(TEMP);
	
	
	int64_t OFF1 = static_cast<int64_t>(sensorCoefficients_[2]) * pow(2,16) + (sensorCoefficients_[4] * static_cast<int64_t>(dT) / pow(2,7));
	int64_t SENS1 = static_cast<int64_t>(sensorCoefficients_[1]) * pow(2,15) +(sensorCoefficients_[3] *static_cast<int64_t>(dT) / pow(2,8));
	
	int64_t T2 = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
		
	if (TEMP < 2000){
		T2 = 3 * (static_cast<int64_t>(dT) *dT / pow(2,33) ) ;
		OFF2 = 5 * ((TEMP - 2000)*(TEMP - 2000)) / 2 ;
		SENS2 = 5 *	((TEMP - 2000)*(TEMP - 2000)) /8 ;
	}
	else { // temp > 20.0C (2000) 
		T2 = 7 * static_cast<uint64_t>(dT) *dT / pow(2,37);
		OFF2 = 1 * ((TEMP - 2000)*(TEMP - 2000)) / 16 ;
		SENS2 = 0;
	}
	if (TEMP < -1500){// extra low temp compensation < -15C
		OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
		SENS2 = SENS2 + 4 * ((TEMP + 1500) *(TEMP + 1500));
	}
	
	OFF1 = OFF1 - OFF2;
	SENS1 = SENS1 - SENS2;
	
	mbarInt_ = ((this->D1_ * SENS1)/ pow(2,21) - OFF1) / pow(2,15) ;
	mbar_ = (float)mbarInt_ / 10 ;
	
	tempC_ = (float)TEMP / 100;
	 
}

float MS5803_14BA::getTemperature(){
	return tempC_;
}

float MS5803_14BA::getPreassure(){
	return mbar_;
}

uint32_t MS5803_14BA::getD1(){
	return D1_;
}

uint32_t MS5803_14BA::getD2(){
	return D2_;
}

unsigned long MS5803_14BA::MS5803_ADC(int8_t commandADC) {
	// D1 and D2 will come back as 24-bit values, and so they must be stored in
	// a long integer on 8-bit Arduinos.
	long result = 0;
	// Send the command to do the ADC conversion on the chip
	Wire.beginTransmission(MS5803_I2C_ADDRESS);
	Wire.write(commandADC);
	Wire.endTransmission();
	// Wait a specified period of time for the ADC conversion to happen
	// See table on page 1 of the MS5803 data sheet showing response times of
	// 0.5, 1.1, 2.1, 4.1, 8.22 ms for each accuracy level.
	switch (commandADC)
	{
		case COMMAND_D1_256 :
		delay(1); // 1 ms
		break;
		case COMMAND_D2_256 :
		delay(1); // 1 ms
		break;
		case COMMAND_D1_512 :
		delay(3); // 3 ms
		break;
		case COMMAND_D2_512 :
		delay(3); // 3 ms
		break;
		case COMMAND_D1_1024 :
		delay(4);
		break;
		case COMMAND_D2_1024 :
		delay(4);
		break;
		case COMMAND_D1_2048 :
		delay(6);
		break;
		case COMMAND_D2_2048 :
		delay(6);
		break;
		case COMMAND_D1_4096 :
		delay(10);
		break;
		case COMMAND_D2_4096 :
		delay(10);
		break;
	}
	// Now send the read command to the MS5803
	Wire.beginTransmission(MS5803_I2C_ADDRESS);
	Wire.write((byte)COMMAND_ADC_READ);
	Wire.endTransmission();
	// Then request the results. This should be a 24-bit result (3 bytes)
	Wire.requestFrom(MS5803_I2C_ADDRESS, 3);
	byte HighByte, MidByte, LowByte;
	while(Wire.available()) {
		HighByte = Wire.read();
		MidByte = Wire.read();
		LowByte = Wire.read();
	}
	// Combine the bytes into one integer
	result = ((long)HighByte << 16) + ((long)MidByte << 8) + (long)LowByte;
	return result;
}

 
//------------------------------------------------------------------
// Function to check the CRC value provided by the sensor against the
// calculated CRC value from the rest of the coefficients.
// Based on code from Measurement Specialties application note AN520
// http://www.meas-spec.com/downloads/C-Code_Example_for_MS56xx,_MS57xx_%28except_analog_sensor%29_and_MS58xx_Series_Pressure_Sensors.pdf
//
unsigned char MS5803_14BA::MS5803_CRC(uint16_t n_prom[]) {
	int cnt;				// simple counter
	unsigned int n_rem;		// crc reminder
	unsigned int crc_read;	// original value of the CRC
	unsigned char  n_bit;
	n_rem = 0x00;
	crc_read = MS5803_14BA::sensorCoefficients_[7];		// save CRC value read form register
	MS5803_14BA::sensorCoefficients_[7] = (0xFF00 & (MS5803_14BA::sensorCoefficients_[7])); // CRC byte replaced with 0
	for (cnt = 0; cnt < 16; cnt++)
	{ // choose LSB or MSB
		if (cnt%2 == 1) {
			n_rem ^= static_cast<unsigned short>(((MS5803_14BA::sensorCoefficients_[cnt>>1]) & 0x00FF));
		}
		else {
			n_rem ^= static_cast<unsigned short>((MS5803_14BA::sensorCoefficients_[cnt>>1] >> 8));
		}
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else {
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem = (0x000F & (n_rem >> 12));// // final 4-bit reminder is CRC code
	sensorCoefficients_[7] = crc_read; // restore the crc_read to its original place
	// Return n_rem so it can be compared to the sensor's CRC value
	return (n_rem ^ 0x00);
}




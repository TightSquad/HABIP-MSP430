/******************************************************************************
*  Filename:       pres_ms5607.h
*  Revised:        
*  Revision:       
*
*  Description:    Interface to the I2C pressure sensor TE Connectivity part
*  				number MS560702BA03-50. Digi-key part number 223-1198-1-ND
*
*  Datasheet at http://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5607-02BA03&DocType=Data+Sheet&DocLang=English
******************************************************************************/
#ifndef PRES_MS5607_H
#define PRES_MS5607_H

// Slave Address
#define PRES_MS5607_I2C_ADDRESS			0x77

// PRES_MS5607 Register Addresses
#define PRES_MS5607_REG_RESET			0x1E // Reset to load PROM to registers, needed on power up (use data = 0x0)
#define PRES_MS5607_REG_CONV_D1_256		0x40 // Write triggers a D1(pressure) conversion with OSR=256 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D1_512		0x42 // Write triggers a D1(pressure) conversion with OSR=512 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D1_1024	0x44 // Write triggers a D1(pressure) conversion with OSR=1024 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D1_2048	0x46 // Write triggers a D1(pressure) conversion with OSR=2048 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D1_4096	0x48 // Write triggers a D1(pressure) conversion with OSR=4096 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D2_256		0x50 // Write triggers a D2(temp) conversion with OSR=256 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D2_512		0x52 // Write triggers a D2(temp) conversion with OSR=512 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D2_1024	0x54 // Write triggers a D2(temp) conversion with OSR=1024 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D2_2048	0x56 // Write triggers a D2(temp) conversion with OSR=2048 (use data = 0x0)
#define PRES_MS5607_REG_CONV_D2_4096	0x58 // Write triggers a D2(temp) conversion with OSR=4096 (use data = 0x0)
#define PRES_MS5607_REG_ADC_READ		0x00 // Burst read from the ADC register (either D1 or D2 value), 24bits = 3 byte list
#define PRES_MS5607_REG_PROM_MANU		0xA0 // PROM addr_0, word reserved for manufacturer
#define PRES_MS5607_REG_PROM_C1			0xA2 // PROM addr_1, constant C1 = Pressure Sensitivity = SENS_T1
#define PRES_MS5607_REG_PROM_C2			0xA4 // PROM addr_2, constant C2 = Pressure Offset = OFF_T1
#define PRES_MS5607_REG_PROM_C3			0xA6 // PROM addr_3, constant C3 = Temp Coeff of Pressure Sensitivity = TCS
#define PRES_MS5607_REG_PROM_C4			0xA8 // PROM addr_4, constant C4 = Temp Coeff of Pressure Offset = TCO
#define PRES_MS5607_REG_PROM_C5			0xAA // PROM addr_5, constant C5 = Reference temperature = T_REF
#define PRES_MS5607_REG_PROM_C6			0xAC // PROM addr_6, constant C6 = Temp Coeff of the temperature = TEMPSENS
#define PRES_MS5607_REG_PROM_CRC		0xAE // PROM addr_7, CRC value, bitd [3:0]

// Other defines
#define REGISTER_LENGTH					2 // Number of bytes in a register
#define SENSOR_DATA_SIZE                2 // Number of bytes to sensor data

// Functions
extern bool presMs5607Reset(void);
extern bool presMs5607ReadProm(uint8_t ui8Reg);
extern bool presMs5607ReadPromC1(uint16_t *SENS_T1);
extern bool presMs5607ReadPromC2(uint16_t *OFF_T1);
extern bool presMs5607ReadPromC3(uint16_t *TCS);
extern bool presMs5607ReadPromC4(uint16_t *TCO);
extern bool presMs5607ReadPromC5(uint16_t *T_REF);
extern bool presMs5607ReadPromC6(uint16_t *TEMPSENS);

#endif /* PRES_MS5607_H */

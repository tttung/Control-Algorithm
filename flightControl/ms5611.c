#include "ms5611.h"
#include "scheduler.h"

#define W_MS5611_ADDR           0xEE //   0x77
#define R_MS5611_ADDR 		0xEF
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

#define BARO_TAB_SIZE_MAX   48

int32_t BaroOffset;
static uint32_t ms5611_ut;  // static result of temperature measurement
static uint32_t ms5611_up;  // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;

int32_t baroHigh = 0;
int32_t baroTemperature = 0;

static uint8_t sel_ms5611_reg(uint8_t reg)
{
	if(OK!=iic_start())return ERROR;
	if(OK!=i2c_senddat(W_MS5611_ADDR))
	{
		iic_stop();
		return ERROR;
	};
	if(OK!=i2c_senddat(reg))
	{
		iic_stop();
		return ERROR;
	};
	return OK;
}
static uint8_t write_ms5611(uint8_t reg,uint8_t *datbuf,uint16_t datl)
{
	if(OK!=sel_ms5611_reg(reg))return ERROR;
	for(;datl!=0;datl--)
	{
		if(OK!=i2c_senddat(*datbuf))
		{
			iic_stop();
			return ERROR;
		};
		datbuf++;
	}
	return iic_stop();
}
static uint8_t read_ms5611(uint8_t reg,uint8_t *datbuf,uint16_t datl)
{
	if(OK!=sel_ms5611_reg(reg))return ERROR;
	if(OK!=iic_rstart())return ERROR;
	if(OK!=i2c_senddat(R_MS5611_ADDR))
	{
		iic_stop();
		return ERROR;
	};

	for(;datl!=0;datl--)
	{
		*datbuf=i2c_recedat(datl);
		datbuf++;
	}
	return iic_stop();
}

static void ms5611_reset(void)
{
	write_ms5611(CMD_RESET,0x01,1);
	delayms(5);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
	uint8_t rxbuf[2] = { 0, 0 };
	read_ms5611(CMD_PROM_RD + coef_num * 2, rxbuf, 2);
	return (rxbuf[0] << 8 | rxbuf[1]);
}

static uint32_t ms5611_read_adc(void)
{
	uint8_t rxbuf[3];
	read_ms5611(CMD_ADC_READ, rxbuf, 3); // read ADC
	return ((uint32_t)rxbuf[0] << 16) | ((uint16_t)rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_start_ut(void)
{
	write_ms5611(CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 0x01,1); // D2 (temperature) conversion start!
}

static void ms5611_get_ut(void)
{
	ms5611_ut = ms5611_read_adc();
}

static void ms5611_start_up(void)
{
	write_ms5611(CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 0x01,1); // D1 (pressure) conversion start!
}

static void ms5611_get_up(void)
{
	ms5611_up = ms5611_read_adc();
}


void MS5611_Init(void)
{
	int i;
		
	ms5611_reset();	
	delayms(2);
	
	for (i = 0; i < PROM_NB; i++)
		ms5611_c[i] = ms5611_prom(i);
}

static void ms5611_calculate(int32_t *pressure, int32_t *temperature)
{
	uint32_t press;
	
	float delt;    
	float dT = ms5611_ut - ((uint32_t)ms5611_c[5] * 256);
	float temp = 2000 + ((dT * (int32_t)ms5611_c[6]) / 8388608);   
	double off = ((uint32_t)ms5611_c[2] * 65536) + (((uint32_t)ms5611_c[4] * dT) / 128);
	double sens = ((uint32_t)ms5611_c[1] * 32768) + (((uint32_t)ms5611_c[3] * dT) / 256);
 
	if (temp < 2000)	
	{
		delt = temp - 2000;
		delt = 5 * delt * delt;
		off -= delt / 2;
		sens -= delt / 4;
		if (temp < -1500)	
		{ 
				delt = temp + 1500;
				delt = delt * delt;
				off  -= 7 * delt;
				sens -= (11 * delt) / 2;
		}
	}
	
	press = ((((int32_t)ms5611_up * sens ) / 2097152) - off) / 32768;
	press = (int)((1.0f - pow(press / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
		
	if (pressure)
			*pressure = press;
	if (temperature)
			*temperature = temp; 
}

void Baro_update(void)
{
	static int state = 0;
	if(state)	
	{
		ms5611_get_up();
		ms5611_start_ut();
		ms5611_calculate(&baroHigh, &baroTemperature);
		state = 0;
	}
	else	
	{
		ms5611_get_ut();
		ms5611_start_up();
		state = 1;
	}
}


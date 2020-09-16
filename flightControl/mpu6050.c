#include "mpu6050.h"

#define W_MPU6050_ADDRESS 		0xD0
#define R_MPU6050_ADDRESS 		0xD1
#define MPU6050_REG_WHOAMI		0x75
#define MPU6050_REG_SENSOR_DAT	        0x3B
#define PWR_MGMT_1			0x6B
#define MPU6050_REG_USERC		0x6A
#define MPU6050_REG_SMPRT		0x19
#define MPU6050_REG_CONFIG		0x1A
#define MPU6050_REG_GYROC		0x1B 
#define MPU6050_REG_ACCELC		0x1C 
#define MPU6050_REG_INTP_CFG	        0x37
#define MPU6050_REG_INTP_EN		0x38
#define SAMPLE_RATE				0

#define GYRO_GAIN_RAD		((2000*3.1415926)/(32768*180))

struct mpu6050_t{
int acc_x;
int acc_y;
int acc_z;
int temp;
int gyro_x;
int gyro_y;
int gyro_z;
};

struct mpu6050_t mpu6050;
int16_xyz Gyro_Offset,Acc_Offset;
extern Mode_t Mode;

const uint8_t mpu6050initdat[][2]=
{
{PWR_MGMT_1 , 0x01},
{MPU6050_REG_USERC ,0x00},		
{MPU6050_REG_CONFIG , 0x02},
{MPU6050_REG_SMPRT , 0x00},
{MPU6050_REG_GYROC ,0x18},
{MPU6050_REG_ACCELC , 0x10},
};

static uint8_t sel_mpu6050_reg(uint32_t reg)
{
	if(OK!=iic_start())return ERROR;
	if(OK!=i2c_senddat(W_MPU6050_ADDRESS))
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
static uint8_t write_mpu6050(uint8_t reg,uint8_t *datbuf,uint16_t datl)
{
	if(OK!=sel_mpu6050_reg(reg))return ERROR;
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
static uint8_t read_mpu6050(uint8_t reg,uint8_t *datbuf,uint16_t datl)
{
	if(OK!=sel_mpu6050_reg(reg))return ERROR;
	if(OK!=iic_rstart())return ERROR;
	if(OK!=i2c_senddat(R_MPU6050_ADDRESS))
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

uint8_t MPU6050_Init(void)
{
	uint8_t i;
	const uint8_t *p;
	R_IICA0_Create();
	p=&(mpu6050initdat[0][0]);
	while(*p!=0xff)
	{
		if(OK!=write_mpu6050(*p,(p+1),1))return ERROR;
		p+=2;
	}
	return OK;
}

uint8_t get_mpu6050_dat(struct mpu6050st *datbuf)
{
	uint8_t i,*p,j;
	if(OK!=read_mpu6050(MPU6050_REG_SENSOR_DAT,(uint8_t*)(datbuf),sizeof(struct mpu6050_t)))
	{
		R_IICA0_Create();
		return ERROR;
	}
	p = (uint8_t*)(datbuf);
	
	for(i=0;i<sizeof(struct mpu6050_t);i+=2)
	{
		j=*p;
		*p=*(p+1);
		*(p+1)=j;
		p+=2;
	}
	return OK;
}

void CalOffset_Acc(void)
{
	static long tempax = 0,tempay = 0,tempaz = 0;
	static uint16_t cnt_a = 0;

	cnt_a++;	
	tempax += Acc.X;
	tempay += Acc.Y;
	tempaz += Acc.Z;
	if(cnt_a == 500)
	{
		Acc_Offset.X = tempax/cnt_a;
		Acc_Offset.Y = tempay/cnt_a;
		Acc_Offset.Z = tempaz/cnt_a - ACC_1G;
		Mode.ACC_CALIBRATED = 0;
	}	
}

void CalOffset_Gyro(void)
{
	static long tempgx = 0,tempgy = 0,tempgz = 0;
	static uint16_t cnt_g = 0;

	cnt_g++;
	tempgx += Gyro.X;
	tempgy += Gyro.Y;
	tempgz += Gyro.Z;		
	if(cnt_g == 500)
	{
		Gyro_Offset.X = tempgx/cnt_g;
		Gyro_Offset.Y = tempgy/cnt_g;
		Gyro_Offset.Z = tempgz/cnt_g;
		Mode.GYRO_CALIBRATED = 0;
	}		
}

uint8_t Read_MPU6050(void)
{
	if(OK!=get_mpu6050_dat(&mpu6050))return ERROR;
	Acc.X = mpu6050.acc_x - Acc_Offset.X;
	Acc.Y = mpu6050.acc_y - Acc_Offset.Y;
	Acc.Z = mpu6050.acc_z - Acc_Offset.Z;
	Gyro.X = mpu6050.gyro_x - Gyro_Offset.X;
	Gyro.Y = mpu6050.gyro_y - Gyro_Offset.Y;
	Gyro.Z = mpu6050.gyro_z - Gyro_Offset.Z;
	
	if(Mode.ACC_CALIBRATED)
	{
		Acc_Offset.X = 0;
		Acc_Offset.Y = 0;
		Acc_Offset.Z = 0;
		CalOffset_Acc();
	}
	if(Mode.GYRO_CALIBRATED)
	{
		Gyro_Offset.X = 0;
		Gyro_Offset.Y = 0;
		Gyro_Offset.Z = 0;
		CalOffset_Gyro();
	}
	return OK;
	
	return OK;
}

#define SCL0        PC0
#define SDA0        PC1
#define SDA0_PMD_BIT     BIT2
#define SCL0_PMD_BIT     BIT0

void I2c_StartCondition(void);
void I2c_StopCondition(void);
void I2c_WriteByte(uint8_t txByte);
uint8_t I2c_ReadByte(void);
uint8_t I2c_ReadByte_end(void);
volatile uint8_t err = 0;

void I2c_StopCondition(void)
{
	PC->PMD |= SDA0_PMD_BIT;
	SDA0  = 0;
	SCL0  = 1;
	SDA0  = 1;
	SCL0  = 0;
	SDA0  = 0;
	SCL0  = 1;
	SDA0  = 1;
}

uint8_t I2c_ReadByte_end(void)
{
	uint8_t i;
	uint8_t read_data = 0x00;

	PC->PMD &= ~SDA0_PMD_BIT;
	CLK_SysTickDelay(1);
	for (i = 0; i < 8; i++) {
		read_data <<= 1;
		SCL0 = 1;
		if (SDA0 == 1)
			read_data |= 0x01;
		CLK_SysTickDelay(1);//delay_cycles(1);
		SCL0 = 0;
		CLK_SysTickDelay(1);//delay_cycles(1);
	}
	PC->PMD |= SDA0_PMD_BIT;     // SDA設定為輸出
	SDA0 = 1;
	CLK_SysTickDelay(1);//delay_cycles(1);
	SCL0 = 1;
	CLK_SysTickDelay(1);//delay_cycles(1);
	SCL0 = 0;

	return    read_data;

}

uint8_t I2c_ReadByte(void)
{
	uint8_t i;
	uint8_t read_data = 0x00;

	PC->PMD &= ~SDA0_PMD_BIT;
	CLK_SysTickDelay(1);
	for (i = 0; i < 8; i++) {
		read_data <<= 1;
		SCL0 = 1;
		if (SDA0 == 1)
			read_data |= 0x01;
		CLK_SysTickDelay(1);
		SCL0 = 0;
		CLK_SysTickDelay(1);
	}
	PC->PMD |= SDA0_PMD_BIT;
	SDA0 = 0;
	CLK_SysTickDelay(1);
	SCL0 = 1;
	CLK_SysTickDelay(1);
	SCL0 = 0;
	return    read_data;

}

void I2c_WriteByte(uint8_t txByte)
{
	int8_t i;

	PC->PMD |= SDA0_PMD_BIT;
	for (i = 0; i < 8; i++) {
		CLK_SysTickDelay(1);
		if (txByte & 0x80)
			SDA0 = 1;
		else
			SDA0 = 0;
		SCL0 = 1;
		CLK_SysTickDelay(1);
		SCL0 = 0;
		txByte <<= 1;
	}
	PC->PMD &= ~SDA0_PMD_BIT;
	SCL0 = 1;
	CLK_SysTickDelay(1);
	if (SDA0 == 1)
		err = 1;
	CLK_SysTickDelay(1);
	SCL0 = 0;

}

void I2c_StartCondition(void)
{
	PC->PMD |= SDA0_PMD_BIT;
	SDA0  = 1;
	SCL0  = 1;
	SDA0  = 0;
	CLK_SysTickDelay(5);
	SCL0  = 0;

}

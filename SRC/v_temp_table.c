#include "Nano1X2Series.h"
char ol_chk;
//*****************************************************************************
//* V_K_TABLE.C

//*  Thermocouple K-TYPE table
//*****************************************************************************
const uint8_t ktype_n[] = {    // K-TYPE 0 TO  -200
	0X00, 0X00, 0X00, 0X09, 0XCE, 0X0A, 0X00, 0X28, 0XD0, 0X0A, 0X07, 0X08,
	0X00, 0X52, 0X65, 0X0A, 0X3D, 0X07, 0X00, 0X7C, 0XCE, 0X0A, 0X7D, 0X06,
	0X00, 0XA8, 0X34, 0X0A, 0XBD, 0X08, 0X00, 0XD4, 0XBF, 0X0B, 0X0A, 0X0B,
	0X01, 0X02, 0XAC, 0X0B, 0X69, 0X0C, 0X01, 0X32, 0X3B, 0X0B, 0XE0, 0X0D,
	0X01, 0X63, 0XC8, 0X0C, 0X64, 0X13, 0X01, 0X97, 0XC9, 0X0D, 0X07, 0X1A,
	0X01, 0XCE, 0XDA, 0X0D, 0XEE, 0X1D, 0X02, 0X09, 0XE5, 0X0F, 0X0E, 0X23,
	0X02, 0X4A, 0X4A, 0X10, 0X72, 0X4B, 0X02, 0X92, 0X84, 0X12, 0XC0, 0X78,
	0X02, 0XE7, 0X52, 0X17, 0X0C, 0XB6
};
const uint16_t ktype_p[] = {    // K-TYPE 0 TO 13720
	0x6464, 0x61DE, 0x60DD, 0x6025, 0x5FA1, 0x5F5C, 0x5F49, 0x5F61, 0x5F91,
	0x5FCC, 0x6004, 0x6025, 0x602F, 0x6027, 0x6018, 0x5FFA, 0x5FD8, 0x5FB6,
	0x5F91, 0x5F6D, 0x5F48, 0x5F25, 0x5F03, 0x5EE2, 0x5EC2, 0x5EA2, 0x5E85,
	0x5E6B, 0x5E51, 0x5E34, 0x5E22, 0x5E10, 0x5E01, 0x5DF6, 0x5DE6, 0x5DE0,
	0x5DDD, 0x5DDB, 0x5DDD, 0x5DE2, 0x5DE2, 0x5DEB, 0x5DF8, 0x5E05, 0x5E15,
	0x5E27, 0x5E3B, 0x5E51, 0x5E63, 0x5E7C, 0x5E99, 0x5EB5, 0x5ED5, 0x5EF4,
	0x5F18, 0x5F37, 0x5F64, 0x5F86, 0x5FB2, 0x5FE0, 0x6010, 0x6043, 0x6072,
	0x60AB, 0x60E7, 0x6125, 0x6165
};
const int8_t ktype_p1[] = {    // K-TYPE 0 TO 13720
	45, 16, 12, 8, 5, 1, -1, -3, -4,
	-4, -2, -1, 0, 1, 2, 2, 2, 2,
	2, 2, 2, 2, 2, 2, 2, 2, 2,
	2, 2, 1, 1, 1, 1, 1, 0, 0,
	0, 0, 0, 0, -1, -1, -1, -1, -1,
	-1, -1, -1, -2, -2, -2, -2, -2, -2,
	-2, -3, -2, -3, -3, -3, -3, -3, -4,
	-4, -4, -4, -4
};

/******************************************************************************/
/*   V to K table fix program  */
/******************************************************************************/
int32_t v_temp_fix(int32_t x)
{
	uint8_t flag = 0;  //儲存正負號,負號時flag!=0
	int32_t a, b, result, y, d, e;
	uint8_t c;
	uint8_t mul1 = 0, mul2 = 0, mul3 = 0, mul4 = 0, mul5 = 0, mul6 = 0;
	y = x;
	if (x < 0) {        // 當待轉換的值為負號時flag!=0
		flag = 1;
		x = labs(x);    //long abs(),x的絕對值
		//x *= -1;
	}

	if (flag == 0) { //正溫度
		x = x / 2;
	}   //將範圍縮小,且定義範圍位址值(第4個位元)

	a = x & (~0x0fff);  //將範圍位址值,放入a(第4個位元)
	b = x % a;        //第1~3個位元放入b

	if (flag == 1) {
		a = (a >> 12) * 6;
	} else {
		a = a >> 12;
	}             // a=範圍位址值
	if (y > 548861)              // 54.8861mV(1372度C) -- K ; 正溫度 display="OL"
		c = 1;
	if (y < -58912) { //-58912  ,-25632        // -5.8912mV(-200度C) -- K ; 負溫度 display="OL"
		c = 2;
	} else {
		c = 0;
		if (flag) {
			mul1 = ktype_n[a];
			mul2 = ktype_n[a + 1];
			mul3 = ktype_n[a + 2];
			mul4 = ktype_n[a + 3];
			mul5 = ktype_n[a + 4];
			mul6 = ktype_n[a + 5];
		} else {
			d = ktype_p[a];      //取參數1
			e = ktype_p1[a];
		}     //取參數2
	}

	if (flag == 0) {
		a = d - (e * (b / 256)); //將b數值移兩位,在作參數運算(參考TABLE)
		y = y / 10;
		result = (y * a) / 1000;
	} else {
		result = (unsigned long int)mul1 << 16 | (long int)mul2 << 8 | mul3;
		a = (b * mul6) / 1000 + (((long int)mul4 << 8) | mul5);
		result = result + (a * b / 1000);
	}

	result = result / 10;
	ol_chk = 0;
	switch (c) {
	case (0):    //未OL

		if (flag) {  //flag = 1
			result *= -1;
		}

		break;
	case (1):
		ol_chk = 1;
		break;
	case (2):
		ol_chk = 2;
		result = -8000;
		break;
	}
	return result;
}


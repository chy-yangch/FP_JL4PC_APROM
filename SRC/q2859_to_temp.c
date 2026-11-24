#include "Nano1X2Series.h"
//*****************************************************************************

const uint16_t k_p[] = { // k TYPE 0 to 80
	10339, 10512, 10567, 10614, 10655, 10693, 10722, 10753, 10781, 10805,
	10826, 10845

};

const uint16_t k_p1[] = { // k TYPE 0 to 80
	195, 55, 44, 38, 35, 28, 31, 28, 22, 19, 19, 18
};


int64_t q2859_to_temp(int64_t x)  // x = res
{
	int64_t a, b, result;
	uint16_t mul1, mul2;
	uint8_t flag;
//    x=10291-x;

	if (x <= 10291) {
		flag = 1;     // 正溫度  T-UP then Vbe-down(count)
		b = 10291 - x; // 尋找出範圍位址
	} else {
		flag = 0;    // 負溫度  T-down then Vbe-up(count)
		b = x - 10291; // 尋找出範圍位址
	}

	a = b;
	a = a >> 8;            // 定義範圍位址的位元(第3個位元)
	mul1 = k_p[a];   // 把參數1帶入
	mul2 = k_p1[a];  // 把參數2帶入
	a = b & 0x000f;
	result = b * (mul1 + ((mul2 * a) / 16)) / 1000; //作運算後,最後一位位元移位

	if (flag == 0) // 當值為負號時
		result *= -1; // 將其結果帶上負號

	return result;
}
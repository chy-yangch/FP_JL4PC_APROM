/******************************************************************************
 * @file     LCD.h
 * @brief    NANO1x2 LCD driver header file
 * @version  1.0.1
 * @date     04, September, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __LCDseg_H__
#define __LCDseg_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
	
// LCD segment definitions.
#define e_1       0X04000400                      //  AAAA
#define f_1       0x01000100                      // F    B
#define a_1       0x00010001                      // F    B
#define d_1       0x08000800                      //  GGGG
#define c_1       0x00040004                      // E    C
#define g_1       0x02000200                      // E    C
#define b_1       0x00020002                      //  DDDD

const unsigned long int lcdseg[] = {
  a_1 + b_1 + c_1 + d_1 + e_1 + f_1,                // Displays "0"
  b_1 + c_1,                                        // Displays "1"
  a_1 + b_1 + d_1 + e_1 + g_1,                      // Displays "2"
  a_1 + b_1 + c_1 + d_1 + g_1,                      // Displays "3"
  b_1 + c_1 + f_1 + g_1,                            // Displays "4"
  a_1 + c_1 + d_1 + f_1 +g_1,                       // Displays "5"
  a_1 + c_1 + d_1 + e_1 + f_1 + g_1,                // Displays "6"
  a_1 + b_1 + c_1,                                  // Displays "7"
  a_1 + b_1 + c_1 + d_1 + e_1 + f_1 + g_1,          // Displays "8"
  a_1 + b_1 + c_1 + d_1 + f_1 + g_1,                // Displays "9"
  0x0000000,                                        // Displays Blank  a
  a_1 + d_1,                                        // Displays "="    b
  g_1,                                              // Displays "-"    c
  d_1 + e_1 + f_1,                                  // Displays "L"    d
  a_1 + b_1 + c_1 +  e_1 + f_1 + g_1,               // Displays "A"    e
  a_1 + e_1 + f_1 + g_1,                            // Displays "f"    f
};       



 
#define DIS_HOLD  1
#define HOLD      BIT18
#define DIS_C     1
#define D_C       BIT25
#define DIS_F     1
#define D_F       BIT24
#define DIS_BATT  2
#define BATT      BIT1
#define DIS_DP1   0
#define DP1       BIT19
#define DIS_DP2   1
#define DP2       BIT3
#define DIS_LA    1
#define LA        BIT16
#define DIS_4BC   1
#define BC4       BIT17
       
	
	


/*@}*/ /* end of group NANO1X2_LCD_EXPORTED_FUNCTIONS */


/*@}*/ /* end of group NANO1X2_LCD_Driver */

/*@}*/ /* end of group NANO1X2_Device_Driver */


#ifdef __cplusplus
}
#endif

#endif  /* __LCD_H__ */



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



/*
 * $Id: cobalt-lcd.h,v 1.12 2001/11/30 05:38:46 asun Exp $
 * cobalt-lcd.h : some useful defines for the Cobalt LCD driver
 *		(must be useable from both kernel and user space)
 *
 * Copyright 1996-2000 Cobalt Networks, Inc.
 * Copyright 2001 Sun Microsystems, Inc.
 *
 * By:	Andrew Bose	
 *	Timothy Stonis (x86 version)
 *	Tim Hockin
 *	Adrian Sun
 *	Erik Gilling
 *	Duncan Laurie
 */
#ifndef COBALT_LCD_H
#define COBALT_LCD_H

#ifdef __KERNEL__
#include <cobalt/cobalt.h>
#endif
#include <cobalt/led.h>

#define COBALT_LCD_LINELEN	40
struct lcd_display {
	unsigned long buttons;
	int size1;
	int size2;
	unsigned char line1[COBALT_LCD_LINELEN];
	unsigned char line2[COBALT_LCD_LINELEN];
	unsigned char cursor_address;
	unsigned char character;
	unsigned char leds;
	unsigned char *RomImage;
};

/* different lcd types */
#define LCD_TYPE_UNKNOWN	0
#define LCD_TYPE_PARALLEL	1
#define LCD_TYPE_PARALLEL_B	2
#define LCD_TYPE_I2C		3

/* Function command codes for ioctl */
#define LCD_On			1
#define LCD_Off			2
#define LCD_Clear		3
#define LCD_Reset		4
#define LCD_Cursor_Left		5
#define LCD_Cursor_Right	6
#define LCD_Disp_Left		7
#define LCD_Disp_Right		8
#define LCD_Get_Cursor		9
#define LCD_Set_Cursor		10
#define LCD_Home		11
#define LCD_Read		12		
#define LCD_Write		13	
#define LCD_Cursor_Off		14
#define LCD_Cursor_On		15
#define LCD_Get_Cursor_Pos	16
#define LCD_Set_Cursor_Pos	17
#define LCD_Blink_Off		18
#define LCD_Raw_Inst		19
#define LCD_Raw_Data		20
#define LCD_Type		21

/* LED controls */
#define LED_Set			40	
#define LED_Bit_Set		41
#define LED_Bit_Clear		42
#define LED32_Set		43	
#define LED32_Bit_Set		44
#define LED32_Bit_Clear		45
#define LED32_Get		46	

/* button ioctls */
#define BUTTON_Read		50

/* Button defs */
#define BUTTON_Next		0x3D
#define BUTTON_Next_B		0x7E
#define BUTTON_Reset_B		0xFC
#define BUTTON_NONE_B		0xFE
#define BUTTON_Left_B		0xFA
#define BUTTON_Right_B		0xDE
#define BUTTON_Up_B		0xF6
#define BUTTON_Down_B		0xEE
#define BUTTON_Enter_B		0xBE

#define BUTTON_MASK             0xFE

void cobalt_lcd_start_twiddle(void);
void cobalt_lcd_stop_twiddle(void);
void cobalt_lcd_off(void);

#endif /* COBALT_LCD_H */

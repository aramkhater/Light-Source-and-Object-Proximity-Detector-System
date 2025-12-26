#ifndef _halGPIO_H_
#define _halGPIO_H_
//#include "header/servo.h"
//#include "header/aultrasonic_driver.h"
#include  "header/bsp.h"
#include  "header/app.h"
#include <stdint.h>
//extern UltrasonicDriver us;
extern int t;
extern volatile unsigned int timer_done;


extern volatile char KP;
extern volatile int count;
extern volatile int X;
extern enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable
extern char string1[5];
extern unsigned int delay_time;

extern char tele_angle[4];
extern char volt[5];
extern int tele_angle_int;
extern int tele_dist[4];
extern int tele_flag;
extern int tele_index;
extern int tele_angle_flag;
extern int tele_str_ind;
extern int buf_index ;      // next index to write
extern int sample_count ;
extern char sample_request  ;
extern int LDR1_calb[10];
extern int LDR2_calb[10];
// Variables used for script menu
extern enum FSM_script state_script;
extern char opcode [3];
extern char   arg1 [3];
extern char   arg2 [3];
extern int  opcode_int;
extern int    arg1_int;
extern int    arg2_int;
extern char script_string [64];
extern int script_length;
#define MAX_SCRIPTS 3
#define MAX_FILENAME_LENGTH 8 // including "\0"

typedef struct {
    char numScripts;
    char filenames[MAX_SCRIPTS][MAX_FILENAME_LENGTH];
    int scriptSizes[MAX_SCRIPTS];
    int file_location[MAX_SCRIPTS];
} ScriptStruct;

extern ScriptStruct scriptStruct;

extern void sysConfig(void);
extern void SetByteToPort(char); // Added By RK
extern void clrPortByte(char);
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void enable_interrupts();
extern void disable_interrupts();
extern void timer_delay_ms(unsigned int ms);

extern __interrupt void PBs_handler(void);
extern __interrupt void PBs_handler_P2(void);
extern unsigned adc_read_once(unsigned inch);   // one-shot ADC read on INCH_x
extern unsigned code_to_mV_src(unsigned code);
extern void take_and_store_sample(void);



// #define CHECKBUSY    1  // using this define, only if we want to read from LCD

#ifdef CHECKBUSY
#define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)       (!a ? (P2OUT&=~0X02) : (P2OUT|=0X02)) // P2.1 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~0X02) : (P2DIR|=0X02)) // P2.1 pin direction

#define LCD_RS(a)       (!a ? (P2OUT&=~0X08) : (P2OUT|=0X08)) // P2.3 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~0X08) : (P2DIR|=0X08)) // P2.3 pin direction

#define LCD_RW(a)       (!a ? (P2OUT&=~0X20) : (P2OUT|=0X20)) // P2.5 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~0X20) : (P2DIR|=0X20)) // P2.5 pin direction

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset
/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)           lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()             lcd_cmd(0x01)
#define lcd_putchar(x)          lcd_data(x)
#define lcd_goto(x)             lcd_cmd(0x80+(x))
#define lcd_cursor_right()      lcd_cmd(0x14)
#define lcd_cursor_left()       lcd_cmd(0x10)
#define lcd_display_shift()     lcd_cmd(0x1C)
#define lcd_home()              lcd_cmd(0x02)
#define cursor_off              lcd_cmd(0x0C)
#define cursor_on               lcd_cmd(0x0F)
#define lcd_function_set        lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line            lcd_cmd(0xC0)

extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();

#endif

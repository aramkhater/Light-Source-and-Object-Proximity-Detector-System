#ifndef _bsp_H_
#define _bsp_H_

#include  <msp430g2553.h>          // MSP430x2xx

//#include  <msp430xG46x.h>  // MSP430x4xx
extern int d;

#define   debounceVal      250
//P1.0: LDR1
//P1.3: LDR2
//P1.4: LCD Data D4
//P1.5: LCD Data D5
//P1.6: LCD Data D6
//P1.7: LCD Data D7

//P2.0:
//P2.1: LCD E
//P2.2: Ultrasonic ECHO
//P2.3: LCD RS
//P2.4: Servo PWM
//P2.5: LCD RW
//P2.6: Ultrasonic Trigger

// LEDs abstraction
//#define LEDsArrPort        P1OUT
//#define LEDsArrPortDir     P1DIR
//#define LEDsArrPortSel     P1SEL

// LCDs abstraction
#define LCD_DATA_WRITE     P1OUT
#define LCD_DATA_DIR       P1DIR
#define LCD_DATA_READ      P1IN
#define LCD_DATA_SEL       P1SEL
#define LCD_CTL_SEL        P2SEL


#define IRQdir P2IN  //select as input,the MCU read from it
#define IRQren P2REN // resistor enable
#define IRQout P2OUT // pull-up resistor
#define IRQies  P2IES //Interept selection: falling/rising edge
#define IRQen  P2EN //Interept enable
#define IRQifg  P2IFG    //interrupt flag


//   Generator abstraction
#define GenPort            P2IN
#define GenPortSel         P2SEL
#define GenPortDir         P2DIR
#define GenPortOut         P2OUT



// PushButtons abstraction
#define PBsArrPort          P2IN
#define PBsArrIntPend       P2IFG
#define PBsArrIntEn         P2IE
#define PBsArrIntEdgeSel    P2IES
#define PBsArrPortSel       P2SEL
#define PBsArrPortDir       P2DIR
#define PB0                 0x01  //P2.0

#define TXD                 BIT2
#define RXD                 BIT1


// Servo Motor abstraction
#define ServoPortDir        P2DIR
#define ServoPortSel        P2SEL
#define ServoPortSel2       P2SEL2
#define ServoPwmPin         0x10       // P2.4 (Timer A1 CCR2 PWM out)


//Ultrasonic
#define USPortDir         P2DIR
#define USPortSel         P2SEL
#define USPortSel2        P2SEL2
#define USEchoLeg         0x04       // P2.2  (ECHO -> TA1.1 capture)
#define USTrigLeg1        0x40       // P2.6  (TRIG -> TA0.1 PWM)
#define USTrigLeg2        0x80       // P2.7  (paired select bit)
extern int ISR_FLAG;

// LDR
#define LDRPortOut          P1OUT
#define LDRPortDir          P1DIR
#define LDRPortSel          P1SEL
#define LDR1leg             0x01  //p1.0
#define LDR2leg             0x08  //p1.3

extern void GPIOconfig(void);
extern void TIMER0_A0_config(void);
extern void TIMER1_A2_config(void);
extern void TIMER1_A1_config(void);
extern void ADCconfig(void);
extern void UARTconfig(void);
extern void aultrasoundconfig(void);
#endif




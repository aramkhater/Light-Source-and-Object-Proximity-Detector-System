#include  "header/bsp.h"    // private library - BSP layer
#include <stdint.h>

int ISR_FLAG;
int d = 50;
int val;
//-----------------------------------------------------------------------------  
//           GPIO configuration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
 // volatile unsigned int i; // in case of while loop usage
  
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT


  /* LDR setup */       //LDR1, P1.0 (A0)  ,  LDR2, P1.3 (A3)
      LDRPortSel        &= ~LDR1leg; // LDR1 Input
      LDRPortSel        &= ~LDR2leg; // LDR2 Input
      LDRPortDir        &= ~LDR1leg; // LDR1 Input capture
      LDRPortDir        &= ~LDR2leg; // LDR2 Input capture


  // PushButtons Setup
      PBsArrPortSel         &= ~PB0;         // PB0 is I/O
      PBsArrPortDir         &= ~PB0;         // PB0 is Input
      PBsArrIntEdgeSel      |=  PB0;         // PB0 is pull-up mode
      PBsArrIntEn           |=  PB0;         // enable PB0 interrupt
      PBsArrIntPend         &= ~PB0;         // clear pending interrupt

      // TRIG: P2.6 -> timer output, ECHO: P2.2 -> timer capture input
      USPortDir  |=  USTrigLeg1;          // P2.6 output
      USPortSel  |=  USTrigLeg1;          // route to TA0.1 function
      USPortSel  &= ~USTrigLeg2;          // make sure 2.7 isn't selecting alt func
      USPortSel2 &= ~(USTrigLeg1 | USTrigLeg2); // P2SEL2.6=0, P2SEL2.7=0
      USPortSel  |=  USEchoLeg;           // P2.2 to timer function
      USPortDir  &= ~USEchoLeg;           // P2.2 input (capture)

      /* Servo motor setup */
      ServoPortSel |=  ServoPwmPin;   // route P2.4 to timer peripheral
      ServoPortDir |=  ServoPwmPin;   // P2.4 as output


   P1IFG = 0x00;    // Clear all Port 1 interrupt flags
   P2IFG = 0x00;    // Clear all Port 2 interrupt flags
   _BIS_SR(GIE);                     // enable interrupts globally

}



//-------------------------------------------------------------------------------------
//            Timer1 A2 configuration - For state1
//-------------------------------------------------------------------------------------
void TIMER1_A2_config(void){
    TA1CCTL2 = CAP + CM_1 + CCIE + SCS + CCIS_0; // Timer1 configuration;
    // CM_1 - * Capture mode: 1 - pos. edge */
    // SCS - /* Capture synchronize */
}
void TIMER2config(void){    /* Second Timer (A0) for performing script commands with 500ms interval default (will be changed to D)  */

    // Total time: [2{up/down} * 2^11 {val} * 2^4{interrupts}] / [2^20{smclk}/2^3{divby8}] = 0.5 sec default ^_^

    WDTCTL = WDTHOLD + WDTPW;                                  // Stop WDT
    val = 4*d*10;                                              //    val = 4 * 50(defaultD)*10 ~= 2^11
    TA0CCR0   = (unsigned int)val;
    TA0CTL    = MC_3+TASSEL_2+ID_3+TACLR;      // UpDown to CCR0    +
                                                               // SMCLK             +
                                                               // Div by 8          +
                                                               // Clear Timer
}
//-------------------------------------------------------------------------------------
//           Servo PWM Timer1 A1 configuration
//-------------------------------------------------------------------------------------
void ServoTIMERconfig(void){
    ISR_FLAG = 0;
    TA1CTL =  MC_1 +TASSEL_2 ;             //Continious up CCR0 + SMCLK
}
//------------------------------------------------------------------------------------- 
//            Ultrasonic Timer configuration
//-------------------------------------------------------------------------------------
void UltrasonicTIMERconfig(void){    // Ultrasonic timing setup
    ISR_FLAG = 1;                               // tell ISR we’re in ECHO mode
    TA0CTL   = TASSEL_2 | MC_1;                 // Timer_A0 running for TRIG
    TA1CTL   = TASSEL_2 | MC_2;                 // Timer_A1 continuous, SMCLK
    TA1CCTL1 = CAP | CCIE | CCIS_1 | CM_3 | SCS;// Capture, IRQ, CCI1B, both edges, sync
}
//-------------------------------------------------------------------------------------
//            ADC configuration
//-------------------------------------------------------------------------------------
/*void ADCconfig(void){
    ADC10CTL0 = ADC10SHT_3 | ADC10ON | ADC10IE | REFON | REF2_5V;
      ADC10CTL0 |= SREF_1;
      ADC10CTL1 = INCH_3 + ADC10SSEL_3;     // Input A3 and SMCLK, was |
      //         InpChnl
      ADC10AE0 |= BIT3;                         // P1.3 ADC option select
}
*/
void ADCconfig(void){
    ADC10CTL0 = 0; ADC10CTL1 = 0;
    ADC10AE0  = BIT3 | BIT0;                               // A3 (P1.3) + A0 (P1.0)
    ADC10CTL0 = ADC10SHT_3 | ADC10ON | ADC10IE;   // no REFON/REF2_5V
    ADC10CTL0 = (ADC10CTL0 & ~SREF_7) | SREF_0;   // VR+ = AVCC, VR- = AVSS
    ADC10CTL1 = ADC10SSEL_3;                               // ADC clock = SMCLK
    __delay_cycles(1000);                                  // ref settle
}

/*
//------------------------------------------------------------------------------------- 
//            ADC configuration
//-------------------------------------------------------------------------------------
void ADCconfig(void){
    WDTCTL = WDTHOLD + WDTPW;                    // Stop WDT
    TA1CTL =  MC_1 +TASSEL_2 ;             //Continious up CCR0 + SMCLK

    ADC10CTL0 = ADC10ON + ADC10IE+ ADC10SHT_3+SREF_0;   // ADC10 On/Enable           +
                                                    // Interrupt enable          +
                                                    // use 64 x ADC10CLK cycles  +
                                                    // Set ref to Vcc and Gnd

    ADC10CTL0 = INCH_0+ADC10SSEL_3;                    // Input channel A0 (p1.0) + SMCLK
    ADC10AE0 &=  ~0x08;                                 // P1.3 Analog enable
    ADC10AE0 |=   0x01;                                 // P1.0 Analog enable
}
*/
//-------UART----------------

void UARTconfig(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    //--STABILIZE
    if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
    {
      while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
    DCOCTL = CALDCO_1MHZ;



     // P1DIR |= BIT1 + BIT2;                     // P1.1, P1.2 output
      P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
      P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
      //P1OUT &= ~(BIT1+BIT2);

      UCA0CTL1 |= UCSSEL_2;                     // CLK = SMCLK
      UCA0BR0 = 104;                           //
      UCA0BR1 = 0x00;                           //
      UCA0MCTL = UCBRS0;               //
      UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}

//------------------------------------------------------------------------------
//                             FLASH Config
//------------------------------------------------------------------------------

void FlashConfig(){
    WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer
     if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
     {
       while(1);                               // do not load, trap CPU!!
     }
     DCOCTL = 0;                               // Select lowest DCOx and MODx settings
     BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
     DCOCTL = CALDCO_1MHZ;
     FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
}


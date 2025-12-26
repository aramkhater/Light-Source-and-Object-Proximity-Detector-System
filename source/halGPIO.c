#include  "header/halGPIO.h"     // private library - HAL layer
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define CAL_LEN 10
// ============================ Globals ============================
char string1[5];
unsigned int delay_time = 500;
volatile unsigned int timer_done = 0;
int j=0;
int delay_ifg = 0;
int state_flag = 0;


int tele_angle_flag = 0;
char tele_angle [4] = {'0','0','0','\n'}; // in deg
int tele_index = 0;
int tele_angle_int = 0;
int tele_flag = 0;
int tele_str_ind = 0;
int tele_dist[4] = {'0','0','0','\n'};    // in cm
char volt[5]={'0','0','0','0','\n'};
unsigned cal_A0[CAL_LEN] = {0};
unsigned cal_A3[CAL_LEN] = {0};
static volatile char cal_tx_line[8 + (CAL_LEN*5)];
 int buf_index = 0;      // next index to write
 int sample_count = 0;
  char sample_request  = 0;
int t;
// Variables used for script menu
int scriptFlag = 0;
enum FSM_script state_script;
char script_string [64];
int script_index = 0;
int script_length = 0;
ScriptStruct scriptStruct = {
    .numScripts = 0,
    .filenames = {NULL},
    .scriptSizes = {0},
    .file_location = {0x1000, 0x1040, 0x1080}
};
char *Flash_ptr;                          // Flash pointer
char opcode [3] = {'0','0','\n'};
char   arg1 [3] = {'0','0','\n'};
char   arg2 [3] = {'0','0','\n'};
int  opcode_int = 0;
int    arg1_int = 0;
int    arg2_int = 0;
// ------------- Echo capture state (ISR-owned) -------------
static volatile uint16_t echo_rise = 0, echo_fall = 0, echo_ticks = 0;
static volatile uint8_t  echo_state = 0;   // 0: waiting for rise, 1: waiting for fall
static volatile uint8_t  echo_ready = 0;   // set to 1 when a complete pulse is measured



// ---- Script upload globals ----
#define SCRIPT_MAX_BYTES      255u                     // matches PC-side limit
#define SCRIPT_MAX_HEX_CHARS  (SCRIPT_MAX_BYTES * 2u)  // 2 ASCII nibbles per byte


volatile unsigned int   expected_hex_chars = 0;       // = script_length * 2






//--------------------------------------------------------------------
//             System Configuration
//--------------------------------------------------------------------
void sysConfig(void){
     GPIOconfig();
     lcd_init();
     ADCconfig();
     UARTconfig();
}

void USTrigConfig(void){
    TA0CCR0  = 65535;        // 1 MHz SMCLK -> 1 tick = 1 µs
    TA0CCR1  = 10;           // 10 µs high pulse
    TA0CCTL1 = OUTMOD_7;     // reset/set PWM
    TA0CTL   = TASSEL_2 | MC_1; // SMCLK, up mode
}

void ServoPWMConfig(int freq){
    TA1CCR0 = (int) 20000;// 20 ms @ 1 MHz SMCLK
    TA1CCR2 = (int) freq;     // pulse width in timer ticks
    TA1CCTL2=OUTMOD_7;         // reset/set PWM
    TA1CTL =  MC_1 +TASSEL_2 ;       // up mode, SMCLK
}



//--------------------------------------------------------------------
//---------------------------------------------------------------------
//            LCD
//---------------------------------------------------------------------
//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
            LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
    }
    else
    {
            LCD_DATA_WRITE = c;
            lcd_strobe();
    }

    LCD_RS(0);
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){

    while(*s)
        lcd_data(*s++);
}
//******************************************************************
// write an int to the LCD
//******************************************************************
void lcd_putint(int num) {
    char buf[7];  // enough for -32768\0 (6 chars + terminator)
    char *p = &buf[6];
    *p = '\0';

    int n = num;
    int neg = 0;

    if (n == 0) {
        lcd_puts("0");
        return;
    }

    if (n < 0) {
        neg = 1;
        n = -n;
    }

    while (n > 0 && p > buf) {
        *--p = (n % 10) + '0';
        n /= 10;
    }

    if (neg && p > buf) {
        *--p = '-';
    }

    lcd_puts(p);
}

//******************************************************************
//    write frequency template to LCD
//******************************************************************
void write_freq_tmp_LCD(){
   lcd_clear();
   lcd_home();
    const char SquareWaveFreq[] = "fin=";
    const char Hz[] = "Hz";
     lcd_puts(SquareWaveFreq);
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_puts(Hz);
}
void lcd_cursor2(){
    lcd_cmd(0xC0);
}

void lcd_cursorLeft(){
    lcd_cmd(0x10);
}
//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(){

    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm("NOP");
 // asm("NOP");
  LCD_EN(0);
}

//---------------------------------------------------------------------
//            Polling based Delay function
//---------------------------------------------------------------------
void delay(unsigned int t){  //
    volatile unsigned int i;

    for(i=t; i>0; i--);
}
//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop"); // the command asm("nop") takes roughly 1usec

}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000); // the command asm("nop") takes roughly 1usec

}
//**************************** Flash Memory *********************************

void init_flash_write(int addr){
    Flash_ptr = (char *) addr;                // Initialize Flash pointer  // 0x1000,0x1040,0x1080
    FCTL1 = FWKEY + ERASE;                    // Set Erase bit
    FCTL3 = FWKEY;                            // Clear Lock bit
    *Flash_ptr = 0;                           // Dummy write to erase Flash segment
    FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
}

void disable_flash_write(){
    FCTL1 = FWKEY;                            // Clear WRT bit
    FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

void write_with_addr_flash_char(char value, int addr){

    Flash_ptr = (char *) addr;
    FCTL1 = FWKEY;                      // Set WRT bit for write operation
    FCTL3 = FWKEY;                            // Clear Lock bit
    FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

    *Flash_ptr = (char)value;
}

void write_flash_char(char value){
    *Flash_ptr++ = (char)value;               // Write value to flash
}


//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){
    if (LPM_level == 0x00)
      _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01)
      _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02)
      _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
    else if(LPM_level == 0x03)
      _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04)
      _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}
//---------------------------------------------------------------------
//            Enable interrupts
//---------------------------------------------------------------------
void enable_interrupts(){
  _BIS_SR(GIE);
}
//---------------------------------------------------------------------
//            Disable interrupts
//---------------------------------------------------------------------
void disable_interrupts(){
  _BIC_SR(GIE);
}

//---------------------------------------------------------------------
//                          Timer1 - Timer A1
//---------------------------------------------------------------------
void enableTimerA1() {
    TA1CCTL1 |= CCIE;          // Enable Timer A1 CCR1 interrupt
}

void disableTimerA1() {
    TA1CTL   &= ~TAIE;         // Disable Timer A1 global interrupt
    TA1CCTL1 &= ~CCIE;         // Disable Timer A1 CCR1 interrupt
    TA1CCTL2 &= ~CCIE;         // Disable Timer A1 CCR2 interrupt
    TA1CCTL2  = OUTMOD_5;      // Reset PWM mode
}
void resetTimerA1() {
    TA1CTL |= TACLR;           // Clear Timer A1 counter
    //TA1R = 0x00;              // Explicit reset if needed
}
//---------------------------------------------------------------------
//                           Timer2 - Timer A0
//---------------------------------------------------------------------
void enable_timerA0(){
    TA0CCTL0 |=  CCIE;          // Timer A1 CCR0 interrupt enable
}

void disable_timerA0(){
    TA0CCTL0 &= ~CCIE;          // Timer A1 CCR2 interrupt disable
}

void reset_timerA0(){
    TA0CTL   |= TACLR;          // Timer A0 counter clear
}
//*********************************************************************
//            TimerA0 Interrupt Service Routine
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    __bic_SR_register_on_exit(LPM0_bits);
}


//*********************************************************************
//            TimerA1 A1 Vector: CCR1 (echo capture), CCR2, TAIFG
//*********************************************************************
// ----------------- Timer1_A1 ISR (CCR1/CCR2/TAIFG) -----------------
#pragma vector=TIMER1_A1_VECTOR  // same alias as PWM_LDR_or_Echo
__interrupt void Timer_1_ISR(void){
    if (ISR_FLAG==0){
        TA1CTL &= ~TAIFG;
        __bic_SR_register_on_exit(LPM0_bits);
    } else {
        static int echo_arr[2];
        static int capture_index = 0;

        echo_arr[capture_index++] = (int)TA1CCR1; // latches at edge
        TA1CCTL1 &= ~CCIFG;
        TA1CTL   &= ~TAIFG;

        if(capture_index==2){
            t = echo_arr[1] - echo_arr[0];     // pulse width in µs (@1 MHz)
            capture_index = 0;
            __bic_SR_register_on_exit(LPM0_bits); // wake measure_distance()
        }
    }
}


//*********************************************************************
//            Port2 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void PBs_handler(void){
    delay(debounceVal);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------
        if(PBsArrIntPend & PB0){
          PBsArrIntPend &= ~PB0;
          sample_request = 1;
          __delay_cycles(100000);
          __bic_SR_register_on_exit(LPM0_bits); //out from sleep
        }
//---------------------------------------------------------------------
//            Exit from a given LPM
//---------------------------------------------------------------------
        switch(lpm_mode){
        case mode0:
             LPM0_EXIT; // must be called from ISR only
             break;

        case mode1:
             LPM1_EXIT; // must be called from ISR only
             break;

        case mode2:
             LPM2_EXIT; // must be called from ISR only
             break;

        case mode3:
             LPM3_EXIT; // must be called from ISR only
             break;

        case mode4:
             LPM4_EXIT; // must be called from ISR only
             break;
    }
}

//*********************************************************************
//            ADC10 Vector Interrupt Service Routine
//*********************************************************************
static inline unsigned adc_read_once(unsigned inch){
    while (ADC10CTL1 & ADC10BUSY);
    ADC10CTL0 &= ~ENC;                                     // change INCH only with ENC=0
    ADC10CTL1 = (ADC10CTL1 & ~0xF000) | inch | ADC10SSEL_3;
    ADC10CTL0 |= ENC | ADC10SC;                            // start
    __bis_SR_register(LPM0_bits | GIE);                    // sleep until ADC10 ISR
    return ADC10MEM;                                       // 10-bit code
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void){ __bic_SR_register_on_exit(LPM0_bits); }

static inline unsigned code_to_mV_src(unsigned code){
    // 2.5 V ref at pin, 1:1 divider -> source range 0–5.000 V
    return     (unsigned)((code * (unsigned long)5000UL + 511) / 1023);

}

void Pot_vol_dual_back2back(void){
    while (state == state4){
        unsigned cA3 = adc_read_once(INCH_3);              // P1.3
        unsigned cA0 = adc_read_once(INCH_0);              // P1.0
        unsigned mV3 = code_to_mV_src(cA3);
        unsigned mV0 = code_to_mV_src(cA0);

        lcd_clear();
        lcd_puts("A3="); lcd_putint(mV3/1000); lcd_puts(".");
        if (mV3%1000<100) lcd_puts("0"); if (mV3%1000<10) lcd_puts("0");
        lcd_putint(mV3%1000); lcd_puts("V ");

        lcd_puts("A0="); lcd_putint(mV0/1000); lcd_puts(".");
        if (mV0%1000<100) lcd_puts("0"); if (mV0%1000<10) lcd_puts("0");
        lcd_putint(mV0%1000); lcd_puts("V");


        __delay_cycles(100000);
    }
}
static inline unsigned adc_read_mV_0to5(unsigned inch){
    while (ADC10CTL1 & ADC10BUSY);
    ADC10CTL0 &= ~ENC;                                      // change INCH safely
    ADC10CTL1 = (ADC10CTL1 & ~0xF000) | inch | ADC10SSEL_3; // INCH_[0..7]
    ADC10CTL0 |= ENC | ADC10SC;                             // start conversion
    __bis_SR_register(LPM0_bits | GIE);                     // wait for ADC10 ISR
    unsigned code = ADC10MEM;                               // 10-bit
    // 2.5 V ref at the pin, 1:1 divider -> sensor range 0..5.000 V
    //unsigned mV = (unsigned)((code * 5000UL + 511) / 1023); // rounded
    unsigned mV = (unsigned)((code * (unsigned long)3300U + 511) / 1023);
    if (mV > 5000) mV = 5000;
    return mV;
}
static inline void take_and_store_sample(void)
{
    // Read both channels
    unsigned c0 = adc_read_once(INCH_0);   // P1.0
    unsigned c3 = adc_read_once(INCH_3);   // P1.3

    unsigned v0 = code_to_mV_src(c0);
    unsigned v3 = code_to_mV_src(c3);

    cal_A0[buf_index] = v0;
    cal_A3[buf_index] = v3;

    buf_index++;
    sample_count++;
}
void Pot_vol_store10_on_button(void)
{
    // Clear previous data
    buf_index = 0;
    sample_count = 0;
    sample_request = 0;

    // Optional: clear LCD at start
    // lcd_clear(); lcd_puts("Press P2.0 x10");

    while (state == state5)
    {
        if (sample_request)
        {
            sample_request = 0;

            if (sample_count < 10)
            {
                take_and_store_sample();

                // Optional: brief feedback on LCD
                 lcd_clear();
                 //lcd_puts("Saved #");
                 lcd_putint(sample_count);
                 lcd_puts("A3:");
                 lcd_putint(cal_A3[buf_index-1]);
                 lcd_puts("A0:");
                 lcd_putint(cal_A0[buf_index-1]);
                // lcd_puts("  A0="); lcd_putint(mV0_buf[buf_index-1]);
                // lcd_puts("mV A3="); lcd_putint(mV3_buf[buf_index-1]); lcd_puts("mV");
            }
            // If already have 10, ignore further presses (debounce ISR blocks new requests)
        }
        if(sample_count>=10)
                state=state0;

        // Sleep until an interrupt (button/ADC/debounce) wakes us
        __bis_SR_register(LPM0_bits | GIE);
        __no_operation();
    }

    // (Optional) When leaving state4 you can process or transmit the arrays:
    // for (unsigned i=0;i<sample_count;i++){ send mV0_buf[i], mV3_buf[i]; }
}
static void format_cal_line(volatile char *dst, const volatile unsigned *arr, unsigned n, char which)
{
    unsigned i;
    char *p = (char *)dst;
    *p++='C'; *p++='A'; *p++='L'; *p++=which; *p++=':';
    for (i=0; i<n; ++i){
        unsigned v = arr[i];
        *p++ = (char)('0' + ((v/1000)%10));
        *p++ = (char)('0' + ((v/100)%10));
        *p++ = (char)('0' + ((v/10)%10));
        *p++ = (char)('0' + (v%10));
        if (i+1 < n) *p++ = ',';
    }
    *p++ = '\n';
    *p   = '\0';
}
static void send_cal_line(char which)
{
    if (which == '0') {
        format_cal_line(cal_tx_line, cal_A0, CAL_LEN, '0');
        tele_flag = 4;
    } else { /* '3' */
        format_cal_line(cal_tx_line, cal_A3, CAL_LEN, '3');
        tele_flag = 5;
    }
    tele_str_ind = 0;
    IE2 |= UCA0TXIE;
    UCA0TXBUF = cal_tx_line[tele_str_ind];

    /* Block until ISR finishes this line (tele_flag cleared there) */
    while (tele_flag == 4 || tele_flag == 5) { __no_operation(); }
}
static inline void u16_to_4digits(char out[5], unsigned val){     // 0000..9999 + '\n'
    out[3] = (val % 10) + '0'; val/=10;
    out[2] = (val % 10) + '0'; val/=10;
    out[1] = (val % 10) + '0'; val/=10;
    out[0] = (val % 10) + '0';
    out[4] = '\n';
}
static inline void u16_to_3digits(char out[4], unsigned val){     // 000..999 + '\n'
    if (val > 999) val = 999;
    out[2] = (val % 10) + '0'; val/=10;
    out[1] = (val % 10) + '0'; val/=10;
    out[0] = (val % 10) + '0';
    out[3] = '\n';
}
/*
void scanANDmeasureVolt(void){
    int angle, i;
    servo_set_angle(0);

    while (state == state3) {
        for (angle = 0; angle <= 180 && state == state3; angle += 2) {
            servo_set_angle(angle);
            __delay_cycles(50000);                       // let LDR settle

            // --- pick channel by angle and read millivolts ---
            unsigned mV =
                (angle < 90) ? adc_read_mV_0to5(INCH_0)  // P1.0 (A0)
                             : adc_read_mV_0to5(INCH_3); // P1.3 (A3)

            // --- send voltage (vvvv\n) ---
            u16_to_4digits((char*)volt, mV);
            tele_flag = 3; tele_str_ind = 0;
            IE2 |= UCA0TXIE;                             // enable USCI_A0 TX IRQ
            UCA0TXBUF = volt[tele_str_ind];

            __delay_cycles(50000);

            // --- send angle (aaa\n) ---
            u16_to_3digits((char*)tele_angle, (unsigned)angle);
            tele_flag = 2; tele_str_ind = 0;
            IE2 |= UCA0TXIE;
            UCA0TXBUF = tele_angle[tele_str_ind];

            __delay_cycles(100000);
        }
        lcd_clear();
        state = state0;
    }
}
*/
static void send_calibration_arrays(void)
{
    send_cal_line('0');  // CAL0:...
    send_cal_line('3');  // CAL3:...
}
void scanANDmeasureVolt(void){
    int angle;
    servo_set_angle(0);

    /* Send the calibration arrays once before the sweep */
    send_calibration_arrays();

    while (state == state3) {
        for (angle = 0; angle <= 180 && state == state3; angle += 2) {
            servo_set_angle(angle);
            __delay_cycles(50000);  // settle

            /* Read both channels (A0 & A3), convert to mV */
            unsigned cA0 = adc_read_once(INCH_0);      // P1.0 (A0)
            unsigned cA3 = adc_read_once(INCH_3);      // P1.3 (A3)
            unsigned mV0 = code_to_mV_src(cA0);        // 0..5000 mV
            unsigned mV3 = code_to_mV_src(cA3);

            /* Choose which value to send based on angle */
            unsigned mV_to_send = (angle < 90) ? mV0 : mV3;
           // unsigned mV_to_send = mV3;
            /* --- send voltage (vvvv\n) with tele_flag == 3 --- */
            u16_to_4digits((char*)volt, mV_to_send);
            tele_flag = 3; tele_str_ind = 0;
            IE2 |= UCA0TXIE;
            UCA0TXBUF = volt[tele_str_ind];
            __delay_cycles(50000);

            /* --- send angle (aaa\n) with tele_flag == 2 --- */
            u16_to_3digits((char*)tele_angle, (unsigned)angle);
            tele_flag = 2; tele_str_ind = 0;
            IE2 |= UCA0TXIE;
            UCA0TXBUF = tele_angle[tele_str_ind];
            __delay_cycles(100000);
        }
        state = state0;
    }
}

void scanANDmeasureDistVolt(void){
    int angle,i;
    int distance = 0;
    servo_set_angle(0);

    /* Send the calibration arrays once before the sweep */
    send_calibration_arrays();

    while (state == state4) {
        for (angle = 0; angle <= 180 && state == state4; angle += 2) {
            servo_set_angle(angle);
            __delay_cycles(50000);  // settle

            /* Read both channels (A0 & A3), convert to mV */
            unsigned cA0 = adc_read_once(INCH_0);      // P1.0 (A0)
            unsigned cA3 = adc_read_once(INCH_3);      // P1.3 (A3)
            unsigned mV0 = code_to_mV_src(cA0);        // 0..5000 mV
            unsigned mV3 = code_to_mV_src(cA3);
            distance = DistMeasure();

                       // -------- Send Distance --------
                       if (distance < 0)   distance = 0;
                       if (distance > 999) distance = 999;

                       int d = distance;
                       for (i = 2; i >= 0; --i) {
                           tele_dist[i] = (d % 10) + '0';
                           d /= 10;
                       }
                       tele_dist[3] = '\n';
                       tele_flag = 1;
                       tele_str_ind = 0;
                       IE2 |= UCA0TXIE;
                       UCA0TXBUF = tele_dist[tele_str_ind];

                       __delay_cycles(50000);
            /* Choose which value to send based on angle */
            unsigned mV_to_send = (angle < 90) ? mV0 : mV3;
            //unsigned mV_to_send = (mV0+mV3)/2;
            /* --- send voltage (vvvv\n) with tele_flag == 3 --- */
            u16_to_4digits((char*)volt, mV_to_send);
            tele_flag = 3; tele_str_ind = 0;
            IE2 |= UCA0TXIE;
            UCA0TXBUF = volt[tele_str_ind];
            __delay_cycles(50000);

            // -------- Send Angle  --------
                        int a = angle;
                        for (i = 2; i >= 0; --i) {
                            tele_angle[i] = (a % 10) + '0';
                            a /= 10;
                        }
                        tele_angle[3] = '\n';
                        tele_flag = 2;
                        tele_str_ind = 0;
                        IE2 |= UCA0TXIE;
                        UCA0TXBUF = tele_angle[tele_str_ind];

                        __delay_cycles(100000);
        }
        state = state0;
    }
}
//*********************************************************************
//                           USCI_A0 TX ISR
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    // send distance from telemeter to PC_side
    if(tele_flag == 1){
        UCA0TXBUF = tele_dist[tele_str_ind++];
        if (tele_dist[tele_str_ind-1] == '\n'){     // TX over
            IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
            tele_flag = 0;
        }
    }
    // send angle from telemeter to PC_side
    else if(tele_flag == 2){
    UCA0TXBUF = tele_angle[tele_str_ind++];
        if (tele_angle[tele_str_ind-1] == '\n'){           // TX over?
            IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
                tele_flag = 0;
         }
    }
    else if(tele_flag == 3){
        UCA0TXBUF = volt[tele_str_ind++];
            if (volt[tele_str_ind-1] == '\n'){           // TX over?
                IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
                    tele_flag = 0;
             }
        }
    else if (tele_flag == 4 || tele_flag == 5) {
            /* NEW: send one full CALx:... line from cal_tx_line[] */
            UCA0TXBUF = cal_tx_line[tele_str_ind++];
            if (cal_tx_line[tele_str_ind-1] == '\n'){
                IE2 &= ~UCA0TXIE;
                tele_flag = 0;
            }
        }
    else{
           IE2 &= ~UCA0TXIE;  // Disable
        }
}

//*********************************************************************
//                           USCI_A0 RX ISR
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    unsigned char rx = UCA0RXBUF;

    // --- Angle capture mode: read exactly 3 ASCII digits ---
    if (tele_angle_flag) {
        if (rx >= '0' && rx <= '9') {
            tele_angle[tele_index++] = rx;
            if (tele_index == 3) {
                tele_angle[3]   = '\0';
                tele_angle_flag = 0;
                // Wake Telemeter() now that we have full angle
                __bic_SR_register_on_exit(LPM0_bits);
            }
        } else {
            // Non-digit during capture: reset the partial entry
            tele_index = 0;
        }
        return; // IMPORTANT: do not fall through to state switching or LPM exit below
    }
    else if (scriptFlag == 1) {
        unsigned char rx = UCA0RXBUF; // the "length" byte sent by PC
        script_length = rx;           // keep for logging if you want
        script_index = 0;
        scriptFlag = 2;               // start collecting payload until '$'
    }


    else if (scriptFlag == 2) {
        // Read the byte that just arrived
        unsigned char rx = UCA0RXBUF;
            while (!(IFG2 & UCA0TXIFG));                  // TX ready
            UCA0TXBUF = 'K';
        // Store with bounds check (assumes: static char script_string[64]; int script_index;)
        if (script_index < (int)sizeof(script_string) - 1) {
            script_string[script_index++] = rx;
        }

        // TERMINATE ON '$'  (no length-based gate anymore)
        if (rx == '$') {

            script_string[script_index] = '\0';   // defensive NUL termination
            script_length = script_index;         // record actual received size
            scriptFlag = 0;
            __bic_SR_register_on_exit(LPM0_bits); // wake main loop so it can send ACK
        }
    }



        else{
    switch (rx) {
        case '0':
            state = state0;
            __bic_SR_register_on_exit(LPM0_bits);  // wake main loop
            break;

        case '1':
            state = state1;
            __bic_SR_register_on_exit(LPM0_bits);  // wake main loop
            break;

        case '2':
            state = state2;
            tele_angle_flag = 1;
            tele_index = 0;
            __bic_SR_register_on_exit(LPM0_bits);  // wake Telemeter() now
            break;

        case '3':
            state = state3;
            __bic_SR_register_on_exit(LPM0_bits);  // wake main loop
            break;
        case '4':
              state = state4;
              __bic_SR_register_on_exit(LPM0_bits);  // wake main loop
             break;
        case '5':
             state = state5;
             __bic_SR_register_on_exit(LPM0_bits);  // wake main loop
             break;
        case '6':
             state = state6;
             __bic_SR_register_on_exit(LPM0_bits);  // wake main loop
             break;
        case '7':
             state = state7;
             __bic_SR_register_on_exit(LPM0_bits);  // wake main loop
             break;
        case 'A':
                        state_script = upload_file1;
                        scriptFlag = 1;
                        break;

                    case 'B':
                        state_script = upload_file2;
                        scriptFlag = 1;
                        break;

                    case 'C':
                        state_script = upload_file3;
                        scriptFlag = 1;
                        break;

                    case 'D':
                        state_script = play_file1;
                        break;

                    case 'E':
                        state_script = play_file2;
                        break;

                    case 'F':
                        state_script = play_file3;
                        break;
    }
        }


}


#include  "header/api.h"           // private library - API layer
#include  "header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"
#include <stdlib.h>



int DistMeasure(void){
    int dist = 0;
    USTrigConfig();   // 10 µs TRIG pulse (TA0.1 on P2.6)
    UltrasonicTIMERconfig();       // start TA0/TA1 and enable capture on P2.2
    enterLPM(lpm_mode);   // wait for ISR to fill 'diff' and wake us

    if (t > 0 && t < 65535){
        dist = t / 58;   // µs -> cm conversion
    }
    disableTimerA1();          // stop capture timer
    return dist;
}

void servo_set_angle(int angle){
    angle = (angle * 10) + 100;     // angle*(2000-200)/180 + 100
    ServoTIMERconfig();
    ServoPWMConfig(angle);
    __delay_cycles(500000);
    disableTimerA1();
}


/*void scanANDmeasure(){
    int i;
    int dist=0;
    servo_set_angle(0);
    for(i=0;i<=180;i+=2){
        servo_set_angle(i);
        lcd_clear();
        dist=DistMeasure();
        lcd_putint(dist);
    }
    lcd_clear();
    enterLPM(lpm_mode);
}*/

/*void Telemeter(int angle){
    int dist=0;
    servo_set_angle(angle);
    while(state == state2){
        lcd_clear();
        dist=DistMeasure();
        lcd_putint(dist);
        __delay_cycles(50000);
    }
}*/

/*void Telemeter(){
    int distance;
    int i;
    enable_interrupts();
    enterLPM(lpm_mode);  // expect to receive angle (array of chars)
    tele_angle[3] = '\0';
    tele_angle_int = atoi(tele_angle);
    servo_set_angle(tele_angle_int);
    while(state==state2){
        distance = DistMeasure();
        if (distance < 0)   distance = 0;
        if (distance > 999) distance = 999;
        // convert distance(int) to array of chars tele_dist[] (defined in hal).

        for (i = 2; i >= 0; --i) {
          tele_dist[i] = (distance % 10) + '0';
          distance /= 10;
        }
        tele_dist[3] = '\n';
        tele_flag = 1;
        tele_str_ind = 0;
        IE2 |= UCA0TXIE;
        UCA0TXBUF = tele_dist[tele_str_ind];
        __delay_cycles(100000);
    }
}*/
void Telemeter(){
    int distance;
    int i;

    // Arm angle capture BEFORE sleeping
    tele_angle_flag = 1;
    tele_index = 0;

    enable_interrupts();
    enterLPM(lpm_mode);  // RX ISR will wake us ONLY after 3 digits are received

    tele_angle[3] = '\0';
    tele_angle_int = atoi(tele_angle);
    if (tele_angle_int < 0)   tele_angle_int = 0;
    if (tele_angle_int > 180) tele_angle_int = 180;
    servo_set_angle(tele_angle_int);

    while (state == state2) {
        distance = DistMeasure();
        if (distance < 0)   distance = 0;
        if (distance > 999) distance = 999;

        // int -> "DDD\n"
        for (i = 2; i >= 0; --i) {
            tele_dist[i] = (distance % 10) + '0';
            distance /= 10;
        }
        tele_dist[3] = '\n';

        tele_flag = 1;
        tele_str_ind = 0;
        IE2 |= UCA0TXIE;
        UCA0TXBUF = tele_dist[tele_str_ind];

        __delay_cycles(100000);
    }
}


void scanANDmeasure(void){
    int i, angle;
    int distance = 0;

    servo_set_angle(0);
    while (state == state1) {
        for (angle = 0; angle <= 180 && state == state1; angle += 2) {
            servo_set_angle(angle);
            __delay_cycles(50000);
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

        lcd_clear();
        state = state0;
    }
}


void objectDetect(){
        enable_interrupts();
        lcd_clear();
        lcd_puts("Scanning");
        scanANDmeasure();
}



void LightDetect(){
        enable_interrupts();
        lcd_clear();
        lcd_puts("Scanning");
        scanANDmeasureVolt();
}

void ObjectLightDetect(){
        enable_interrupts();
        lcd_clear();
        lcd_puts("Scanning");
        scanANDmeasureDistVolt();
}
//------------------------------------------------------------------------------
//                              Scripts
//------------------------------------------------------------------------------
void script_fsm(){

 state_script = sleep;
 while(state == state6){
    switch(state_script){
    case sleep:
        enterLPM(lpm_mode);
        break;

    case upload_file1:
        lcd_init();
        lcd_puts("Uploading File 1");
        enable_interrupts();
        enterLPM(lpm_mode);  // expect to receive script data
/*
        IE2 |= UCA0TXIE;                       // Enable USCI_A0 TX interrupt
          UCA0TXBUF = '1';
        addScript("script0", script_length); // NEED TO ADD ARRAY OF POINTERS TO THE START ADDRRESS OF EVERY FILE
        flash_write(1);   // put script_string into flash
*/
        IE2 |= UCA0TXIE;
        UCA0TXBUF = '1';
        while (!(IFG2 & UCA0TXIFG)) { /* wait */ }
        UCA0TXBUF = '\0';  // let PC's "read-until-NUL" finish cleanly

        addScript("script0", script_length);
        flash_write(1);


        state_script = sleep;
    break;

    case upload_file2:
        lcd_init();
        lcd_puts("Uploading File 2");
        enable_interrupts();
        enterLPM(lpm_mode);  // expect to receive script data
    /*    IE2 |= UCA0TXIE;                       // Enable USCI_A0 TX interrupt
        UCA0TXBUF = '2';
        addScript("script1", script_length);
        flash_write(2);   // put script_string into flash
*/
        IE2 |= UCA0TXIE;
                UCA0TXBUF = '2';
                while (!(IFG2 & UCA0TXIFG)) { /* wait */ }
                UCA0TXBUF = '\0';  // let PC's "read-until-NUL" finish cleanly

                addScript("script1", script_length);
                flash_write(2);

        state_script = sleep;
    break;

    case upload_file3:
        lcd_init();
        lcd_puts("Uploading File 3");
        enable_interrupts();
        enterLPM(lpm_mode);  // expect to receive script data
     /*   IE2 |= UCA0TXIE;                       // Enable USCI_A0 TX interrupt
        UCA0TXBUF = '3';
        addScript("script2", script_length);
        flash_write(3);   // put script_string into flash
*/
        IE2 |= UCA0TXIE;
                UCA0TXBUF = '3';
                while (!(IFG2 & UCA0TXIFG)) { /* wait */ }
                UCA0TXBUF = '\0';  // let PC's "read-until-NUL" finish cleanly

                addScript("script2", script_length);
                flash_write(3);

        state_script = sleep;
        break;

    case play_file1:
        lcd_init();
        lcd_puts("Playing File1");
        enable_interrupts();
        play_script(1);
        lcd_clear();
        lcd_puts("Done Playing");
        lcd_cursor2();
        lcd_puts("File 1");

        state_script = sleep;
        break;

    case play_file2:
        lcd_init();
        lcd_puts("Playing File2");
        enable_interrupts();
        play_script(2);
        lcd_clear();
        lcd_puts("Done Playing");
        lcd_cursor2();
        lcd_puts("File 2");
        state_script = sleep;
        break;

    case play_file3:
        lcd_init();
        lcd_puts("Playing File3");
        enable_interrupts();
        play_script(3);
        lcd_clear();
        lcd_puts("Done Playing");
        lcd_cursor2();
        lcd_puts("File 3");
        state_script = sleep;
        break;

    }
  }
}
//------------------------------------------------------------------------------
// Play a Script
//------------------------------------------------------------------------------
void play_script(int script_num){
    char d_string [12] = {'d',' ','=',' ','$','$',' ','[','m','s',']','\0'};
    int sleep_state_flag = 0;
    int start_addr = 0x1000;

    start_addr = scriptStruct.file_location[script_num-1]; // get script's start addr in flash
    char * read_ptr = (char *) start_addr; // set the pointer to the start address.

    while (sleep_state_flag == 0){
        opcode[0] = *read_ptr++;
        opcode[1] = *read_ptr++;
        IE2 |= UCA0TXIE;
        UCA0TXBUF = opcode[1]; // send opcode to pc_side
        __delay_cycles(1250000);


        if(*read_ptr != 10){    // if there are any arguments:
            // get first argument:
            arg1[0] = *read_ptr++;
            arg1[1] = *read_ptr++;

            if(*read_ptr != 10){ // if there are 2 arguments:
                      // get second argument:
                      arg2[0] = *read_ptr++;
                      arg2[1] = *read_ptr++;
                      *read_ptr++; // next line
            }
            else{ // next line
                *read_ptr++;
            }
        }
        else{ // next line
            *read_ptr++;
        }

     // after receiving a whole line:
        opcode_int = atoi(opcode);
        arg1_int =   strtol(arg1, NULL, 16);
        arg2_int =   strtol(arg2, NULL, 16);

        switch(opcode_int){
            case 1:
                inc_lcd(arg1_int);
            break;

            case 2:
                dec_lcd(arg1_int);
            break;

            case 3:
                rra_lcd(arg1_int);
            break;

            case 4:
                d_string[4] = (arg1_int / 10) + '0';
                d_string[5] = (arg1_int % 10) + '0';
                lcd_clear();
                lcd_puts(d_string);
                set_delay(arg1_int);
            break;

            case 5:
                lcd_clear();
            break;

            case 6:
                servo_deg(arg1_int);
                __delay_cycles(100000); // IMPORTANT
            break;

            case 7:
                lcd_clear();
                lcd_puts("Scanning");
                lcd_cursor2();
                lcd_puts("Environment");
                servo_scan(arg1_int, arg2_int);
            break;

            case 8:
                sleep_state_flag = 1;
            break;
        }
    }
}



void addScript(const char * filename, int scriptSize) {
    if (scriptStruct.numScripts >= MAX_SCRIPTS) {
        return;
    }
    strncpy(scriptStruct.filenames[scriptStruct.numScripts], filename, MAX_FILENAME_LENGTH - 1);
    scriptStruct.filenames[scriptStruct.numScripts][MAX_FILENAME_LENGTH - 1] = '\0';
    scriptStruct.scriptSizes[scriptStruct.numScripts] = scriptSize;
    scriptStruct.numScripts++;
}



void inc_lcd(int x){
  int upcounter = 0;
  char upcounter_str[10];

  TIMER2config();
  lcd_init();
  reset_timerA0();
  disable_interrupts();
  while(upcounter <= x)
  {
      enable_timerA0(); //enable the timer with delay of d[msec]
      enterLPM(lpm_mode);
      disable_timerA0();
      sprintf(upcounter_str, "%d", upcounter);
      lcd_clear();
      lcd_puts(upcounter_str);      //print
      upcounter ++;
  }
  lcd_clear();
  disable_timerA0();
}


void dec_lcd(int x){
  int downcounter = x;
  char downcounter_str[10];

  TIMER2config();
  lcd_init();
  reset_timerA0();
  while(downcounter >= 0)
  {
      enable_timerA0(); //enable the timer with delay of d[msec]
      enterLPM(lpm_mode);
      disable_timerA0();
      sprintf(downcounter_str, "%d", downcounter);
      lcd_clear();
      lcd_puts(downcounter_str);      //print
      downcounter --;
  }
  lcd_clear();
  disable_timerA0();
}



void rra_lcd(int x){
    char char_to_rotate [2] = {'0','\0'};
    char_to_rotate[0] = x + '0';
    int counter=0;
    TIMER2config();
    lcd_init();
    lcd_clear();

    while (counter < 32){
        enable_timerA0();
        enterLPM(lpm_mode);
        disable_timerA0();


        lcd_cursorLeft();
        lcd_puts(" ");
        lcd_puts(char_to_rotate);


        if (counter == 15){
            lcd_cursorLeft();
            lcd_puts(" ");
            lcd_cursor2();
        }
        else if(counter == 31){
            lcd_clear();
            disable_timerA0();
        }
        counter++;
    }
}


void set_delay(int delay){
  d=delay;
}
void servo_deg(int p){
    int distance;
        int i,j;
        tele_index = 0;

        enable_interrupts();

        servo_set_angle(p);

        for(j=0;j<=50;j++) {
            distance = DistMeasure();
            if (distance < 0)   distance = 0;
            if (distance > 999) distance = 999;


            for (i = 2; i >= 0; --i) {
                tele_dist[i] = (distance % 10) + '0';
                distance /= 10;
            }
            tele_dist[3] = '\n';

            tele_flag = 1;
            tele_str_ind = 0;
            IE2 |= UCA0TXIE;
            UCA0TXBUF = tele_dist[tele_str_ind];

            __delay_cycles(100000);
        }
}
void servo_scan(int l, int r){
    int i, angle;
        int distance = 0;

        servo_set_angle(l);

            for (angle = l; angle <= r; angle += 2) {
                servo_set_angle(angle);
                __delay_cycles(50000);
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

            lcd_clear();
            //state = state0;

}
//------------------------------------------------------------------------------
//                                   FLASH
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Write to flash memory
//------------------------------------------------------------------------------
void flash_write(int script_num){
   int i=0;
   int addr=0x1000;
   FlashConfig();

   switch(script_num){
    case 1:
        addr=0x1000;
    break;

    case 2:
        addr=0x1040;
    break;

    case 3:
        addr=0x1080;
    break;
   }

   init_flash_write(addr);

   while(script_string[i]!= '$' && i!=63){
       write_flash_char(script_string[i]);
       i++;
   }

   disable_flash_write();
}

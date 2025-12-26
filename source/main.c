#include <msp430.h>
#include "header/app.h"
#include  "header/halGPIO.h"

enum FSMstate state;
enum SYSmode lpm_mode;



int main(void) {
    WDTCTL = WDTPW | WDTHOLD;

    //state = state4;
    lpm_mode = mode0;

    sysConfig();

    int distance;
    int i;

    servo_set_angle(90);


    while (1) {

        switch (state) {
            case state0:
                lcd_clear();
                lcd_puts("Main Menu");
                enterLPM(lpm_mode);
                break;

            case state1:
                lcd_clear();
                lcd_puts("Object Detector");
                enable_interrupts();
                objectDetect();
                state=state0;
                break;

            case state2:
                lcd_clear();
                lcd_puts("Telemeter");
                enable_interrupts();
                Telemeter();
                state = state0;
                break;

            case state3:
                lcd_clear();
                lcd_puts("Light Detector");
                enable_interrupts();
                LightDetect();
                state = state0;
                break;

            case state4:
                lcd_clear();
                lcd_puts("Object & Light Detector");
                enable_interrupts();
                ObjectLightDetect();
                state=state0;
                break;
            case state5:
                lcd_clear();
                lcd_puts("Calibration");
                enable_interrupts();
                Pot_vol_store10_on_button();
                state=state0;
                break;

            case state6:
                lcd_clear();
                lcd_puts("Script Mode");
                enable_interrupts();
                script_fsm();
                state = state0;
                    break;

        }
    }
}

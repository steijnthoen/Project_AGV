#include <avr/io.h>
#include "pindefines.h"

#define JOY_THRES 60
/*
int fwbw_speed, lr_speed;

void get_speeds(void)
{
    //get fw-bw
    ADMUX  = 0b01000000;    //read from pin A0
    ADCSRA |= (1<<ADSC);    //start conversion
    loop_until_bit_is_clear(ADCSRA,ADSC);
    if(ADC > 512+JOY_THRES || ADC < 512-JOY_THRES)  //filter threshold
    {
        fwbw_speed = ADC-512;
        fwbw_speed /= 2;
    }
    else
    {
        fwbw_speed = 0;
    }

    //get left-right
    ADMUX  = 0b01000001;    //read from pin A1
    ADCSRA |= (1<<ADSC);    //start conversion
    loop_until_bit_is_clear(ADCSRA,ADSC);
    if(ADC > 512+JOY_THRES || ADC < 512-JOY_THRES)   //filter threshold
    {
        lr_speed = ADC-512;
        lr_speed /= 2;
    }
    else
    {
        lr_speed = 0;
    }

}

void joystick_move(int fwbw_speed, int lr_speed) //moves -255/255
{
    get_speeds();
    TCCR0A |= (1<<COM0A1)| (1<<COM0B1); //enable PWM
    TCCR1A |= (1<<COM1A1)| (1<<COM1B1); //enable PWM

    //standstill
    if(fwbw_speed == 0 && lr_speed == 0)
    {
        ledoff();
        TCCR0A |= (1<<COM0A1)| (1<<COM0B1); //disable PWM
        TCCR1A |= (1<<COM1A1)| (1<<COM1B1); //disable PWM

        RSPEED_FW = 0;
        LSPEED_FW = 0;
        RSPEED_BW = 0;
        LSPEED_BW = 0;
    }

    else if(fwbw_speed == 0)
    {

    }
    //move
    else
    {
        ledon();
        //forward
        if(fwbw_speed>0)
        {
            RSPEED_BW = 0;
            LSPEED_BW = 0;

            if(lr_speed > 0)      //fw-right turn
            {
                RSPEED_FW = fwbw_speed - lr_speed;
                LSPEED_FW = fwbw_speed;
            }
            else if(lr_speed < 0) //fw-left turn
            {
                RSPEED_FW = fwbw_speed;
                LSPEED_FW = fwbw_speed + lr_speed;
            }
        }


        //backward
        else if(fwbw_speed<0)
        {
            RSPEED_FW = 0;
            LSPEED_FW = 0;

            if(lr_speed > 0)      //bw-right turn
            {
                RSPEED_FW = -fwbw_speed + lr_speed;
                LSPEED_FW = -fwbw_speed;
            }
            else if(lr_speed < 0) //bw-left turn
            {
                RSPEED_FW = -fwbw_speed;
                LSPEED_FW = -fwbw_speed - lr_speed;
            }
        }

    }
}
*/

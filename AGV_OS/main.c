#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pindefines.h"

#define SPEED 200
#define TURNADJUST 90
#define TURNTHRESH 20

#define SENSEWALL  0
#define SENSETREES 1

#define DIST_DIF (read_adc(0) - read_adc(1))

int timercounter2 = 0;

ISR(TIMER2_OVF_vect)
{
    timercounter2++;
}

int read_adc(uint8_t analogreadpin)     //get distance values from IR sensors
{
    ADMUX &= 0b11111000;            //clear pin selection bits
    analogreadpin &= 0b00000111;    //filter pin selection bits
    ADMUX |= analogreadpin;         //set specified pin

    ADCSRA |= (1<<ADSC);    //start conversion
    loop_until_bit_is_clear(ADCSRA,ADSC);   //wait for conversion to finish

    return ADC;
}

int main(void)
{
    pinsetup();
    pwm_init();
    adc_init();
    timer_init();
    sei();

//    loop_until_bit_is_clear(START_PIN,START_BUTTON);    //wait for start signal

    while(1)
    {
        //wall detection
        if (read_adc(2)<800 && SENSEWALL)
        {
            turn_left();
        }

        //tree detection (move to ISR probably)
        if(bit_is_set(INPUT_PIN,IR_SIDE) && SENSETREES)
        {
            move_stop();
            ledon();
            _delay_ms(1000);
            while(bit_is_set(INPUT_PIN,IR_SIDE))  //move until tree is passed
            {
                RSPEED = SPEED;
                LSPEED = SPEED;
            }
            ledoff();
        }


//        while(timercounter2<6); //limit distance measurements
        timercounter2 = 0;

        if(DIST_DIF < TURNTHRESH && DIST_DIF > -TURNTHRESH)
        {
            RSPEED = SPEED;
            LSPEED = SPEED;
        }

        else if (read_adc(0) > read_adc(1))     //left turn
        {
            RSPEED = SPEED;
            LSPEED = SPEED - TURNADJUST;
            TIFR2 |= (1<<TOV2);
        }
        else if (read_adc(0) < read_adc(1))     //right turn
        {
            LSPEED = SPEED;
            RSPEED = SPEED - TURNADJUST;
            TIFR2 |= (1<<TOV2);
        }


    }

    return 0;
}


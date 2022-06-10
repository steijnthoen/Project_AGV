#include <avr/io.h>
#include <util/delay.h>
#include "pindefines.h"

#define ROTATIONSPEED 255
#define TURNTIME  600
#define DRIVESPEED 255

void move_forward(int time)
{
    RSPEED = DRIVESPEED;
    LSPEED = DRIVESPEED;
    if(time!=0)
    {
        _delay_ms(time);
        move_stop();
    }
}

void move_backward(int time)
{
    RSPEED = 0;
    LSPEED = 0;

    RSPEED = DRIVESPEED;
    LSPEED = DRIVESPEED;
}

void turn_right(void)
{
    RPWM_PORT |= (1<<RDIR);

    RSPEED = 255-ROTATIONSPEED;
    LSPEED = ROTATIONSPEED;
    _delay_ms(TURNTIME);

    RPWM_PORT &=~(1<<RDIR);
}

void turn_left(void)
{
    LPWM_PORT |= (1<<LDIR);

    RSPEED = ROTATIONSPEED;
    LSPEED = 255-ROTATIONSPEED;
    _delay_ms(TURNTIME);

    LPWM_PORT &=~(1<<LDIR);
}

void u_turn(void)
{
}

void move_stop(void)
{
    RSPEED = 0;
    LSPEED = 0;
}


void ledon(void)
{
    PORTB |= (1<<PB5);
}

void ledoff(void)
{
    PORTB &=~(1<<PB5);
}

void toggleled(void)
{
    PORTB ^= (1<<PB5);
}

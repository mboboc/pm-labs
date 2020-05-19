/* 
 * Lab00: Blinky LEDs
 * 
 * PM 2020
 */

#include <avr/io.h>
#include <util/delay.h>

#define DELAY 500

/* blink one LED */
void blinky() {
    /* Set pin PB0 as output */
    DDRB |= (1 << PB1);

    /* Turn on LED */
    PORTB |= (1 << PB1);

    while (1) {
        /* Toggle pin state */
        PORTB ^= (1 << PB1);

        _delay_ms(DELAY);
    }
}

/* simple 1 */
void multiple_blinky_1() {
    unsigned char i = 0;
    unsigned char bit;

    /* set all PORTC pins on output*/
    DDRD = 0xFF;

    while (1) {
        bit = i % 8;
        PORTD |= (1 << bit);
        _delay_ms(DELAY);
        PORTD &= ~(1 << bit);
        i++;
    }
}

/* simple 2 */
void multiple_blinky_2() {
    unsigned char i = 0;
    unsigned char bit;

    /* set all PORTC pins on output*/
    DDRD = 0xFF;

    while (1) {
        bit = i % 8;
        if (bit == 7) PORTD = 0x0;
        PORTD |= (1 << bit);
        _delay_ms(DELAY);
        i++;
    }
}

/* simple 3 */
void multiple_blinky_3() {
    unsigned char i = 0;
    unsigned char j = 7;

    /* set all PORTC pins on output*/
    DDRD = 0xFF;

    while (1) {
        PORTD |= (1 << (i % 8));
        PORTD |= (1 << (j % 8));
        _delay_ms(DELAY);
        PORTD &= ~(1 << (i % 8));
        PORTD &= ~(1 << (j % 8));
        i++;
        j--;
    }
}

/* avansat 1 */
void multiple_blinky_4() {
    unsigned char i = 0;
    unsigned char reverse = 0;

    /* set all PORTC pins on output*/
    DDRD = 0xFF;

    while (1) {
        if (i == 8) reverse = 1;
        if (reverse == 1) {
            PORTD |= (1 << i);
            _delay_ms(DELAY);

            PORTD &= ~(1 << i);
            i--;
            if (i == 0) reverse = 0;
        } else {
            PORTD |= (1 << i);
            _delay_ms(DELAY);
            PORTD &= ~(1 << i);
            i++;
        }
    }
}

/* avansat 2 */
void multiple_blinky_5() {
    unsigned char i = 0;
    unsigned char reverse = 0;

    /* set all PORTC pins on output*/
    DDRD = 0xFF;

    while (1) {
        if (i == 8) {
            if (reverse == 1)
                reverse = 0;
            else
                reverse = 1;
            i = 0;
        }
        if (reverse == 1)
            PORTD &= ~(1 << i);
        else
            PORTD |= (1 << i);
        i++;
        _delay_ms(DELAY);
    }
}

/* avansat 3 */
void multiple_blinky_6() {
    unsigned char i = 0;
    char j = -3;
    char k = -6;

    /* set all PORTC pins on output*/
    DDRD = 0xFF;

    while (1) {
        PORTD |= (1 << (i % 8));
        if (j >= 0) PORTD |= (1 << (j % 8));
        if (k >= 0) PORTD |= (1 << (k % 8));
        _delay_ms(DELAY);
        PORTD &= ~(1 << (i % 8));
        if (j >= 0) PORTD &= ~(1 << (j % 8));
        if (k >= 0) PORTD &= ~(1 << (k % 8));
        i++;
		j++;
		k++;
		if (j == 255)
			j = -3;
		if (k == 255);
			k = -6;
    }
}

int main() {
    multiple_blinky_6();
    return 0;
}

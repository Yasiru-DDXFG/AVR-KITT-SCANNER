#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define HC595_PORT PORTB
#define HC595_DDR DDRB
#define HC595_DS_POS PB0    // Data pin (DS) pin location
#define HC595_SH_CP_POS PB1 // Shift Clock (SH_CP) pin location
#define HC595_ST_CP_POS PB2 // Store Clock (ST_CP) pin location

#define HC595DataHigh() (HC595_PORT |= (1 << HC595_DS_POS))
#define HC595DataLow() (HC595_PORT &= (~(1 << HC595_DS_POS)))

volatile uint8_t ctr1 = 0;
volatile uint8_t ctr2 = 0;
volatile uint8_t ctr3 = 0;
volatile uint8_t state = 0;
volatile int direction = 1;
volatile uint8_t data[6] = {0b00000000,
                            0b00000000,
                            0b00000000,
                            0b00000000,
                            0b00000000,
                            0b00000000};

void shiftWrite(uint8_t data)
{
    // Send each 8 bits serially
    // Order is MSB first
    for (uint8_t i = 0; i < 8; i++)
    {
        // Output the data on DS line according to the
        // Value of MSB
        if (data & 0b10000000)
        {
            // MSB is 1 so output high
            HC595DataHigh();
        }
        else
        {
            // MSB is 0 so output high
            HC595DataLow();
        }
        // Pulse the Shift Clock
        HC595_PORT |= (1 << HC595_SH_CP_POS);    // HIGH
        HC595_PORT &= (~(1 << HC595_SH_CP_POS)); // LOW
        data = data << 1;                        // Now bring next bit at MSB position
    }

    // Now all 8 bits have been transferred to shift register
    // Move them to output latch at one
    // Pulse the Store Clock
    HC595_PORT |= (1 << HC595_ST_CP_POS); // HIGH
    _delay_loop_1(1);
    HC595_PORT &= (~(1 << HC595_ST_CP_POS)); // LOW
    _delay_loop_1(1);
}

ISR(TIM0_COMPA_vect)
{
    shiftWrite(data[ctr2]);
    ctr2++;
    if (ctr2 > 5)
    {
        ctr2 = 0;
    }
}

int main()
{
    // Setup timer0 interrupt at 360Hz
    // Assume clkio is 9.6MHz
    // 1+OCR0A = 9.6MHz/(360 * 2 * N) According to datasheet
    // Therefore N = 256, OCR0A = 51
    TCCR0A |= 1 << WGM01; // CTC Mode
    TCCR0B |= 1 << CS02;  // clock/256 prescale
    OCR0A = 1;
    TIMSK0 |= 1 << OCIE0A; // Timer0 Comp match A interrupt enable
    sei();

    // Init. Make the Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) lines output
    HC595_DDR |= ((1 << HC595_SH_CP_POS) | (1 << HC595_ST_CP_POS) | (1 << HC595_DS_POS));

    while (1)
    {
        cli();
        data[0] &= data[1];
        data[1] &= data[2];
        data[2] &= data[3];
        data[3] &= data[4];
        data[4] &= data[5];
        data[5] ^= data[5];
        if (state == 0)
        {
            if (ctr1 >= 0 && ctr1 < 8)
            {
                data[0] |= 1 << ctr1;
                data[1] |= 1 << ctr1;
                data[2] |= 1 << ctr1;
                data[3] |= 1 << ctr1;
                data[4] |= 1 << ctr1;
                data[5] |= 1 << ctr1;
                ctr1 += direction;
            }
            else
            {
                direction *= -1;
                ctr1 += direction;
                state = 1;
            }
        }
        if (state == 1)
        {
            ctr3++;
            if (ctr3 > 4)
            {
                ctr3 = 0;
                state = 0;
            }
        }
        sei();
        _delay_ms(70);
    }
}
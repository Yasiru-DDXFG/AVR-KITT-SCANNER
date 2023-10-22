/*
typedef struct
{
    volatile uint8_t *port_reg;
    volatile uint8_t *ddr_reg;
    uint8_t offset;
} pinType;

pinType pins[10] = {{(uint8_t *)0x2b, (uint8_t *)0x2a, 0},
                    {(uint8_t *)0x2b, (uint8_t *)0x2a, 1},
                    {(uint8_t *)0x2b, (uint8_t *)0x2a, 2},
                    {(uint8_t *)0x2b, (uint8_t *)0x2a, 3},
                    {(uint8_t *)0x2b, (uint8_t *)0x2a, 4},
                    {(uint8_t *)0x2b, (uint8_t *)0x2a, 5},
                    {(uint8_t *)0x2b, (uint8_t *)0x2a, 6},
                    {(uint8_t *)0x2b, (uint8_t *)0x2a, 7},
                    {(uint8_t *)0x25, (uint8_t *)0x24, 0},
                    {(uint8_t *)0x25, (uint8_t *)0x24, 1}};

*/
/*
ISR (TIMER0_COMPA_vect)
{

}
*/

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

uint8_t ctr = 0;
uint8_t ctr2 = 0;
uint8_t direction = 0;
uint8_t data[] = {0b00000011,
                  0b00000011,
                  0b00000010,
                  0b00000010,
                  0b00000111,
                  0b00000110,
                  0b00000100,
                  0b00000100,
                  0b00001110,
                  0b00001100,
                  0b00001000,
                  0b00001000,
                  0b00011100,
                  0b00011000,
                  0b00010000,
                  0b00010000,
                  0b00111000,
                  0b00110000,
                  0b00100000,
                  0b00100000,
                  0b01110000,
                  0b01100000,
                  0b01000000,
                  0b01000000,
                  0b11100000,
                  0b11000000,
                  0b10000000,
                  0b10000000};

void shiftWrite(uint8_t data, uint8_t dir)
{
    // Send each 8 bits serially
    // Order is MSB first
    if (dir == 0)
    {
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
    }
    else
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            // Output the data on DS line according to the
            // Value of MSB
            if (data & 0b00000001)
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
            data = data >> 1;                        // Now bring next bit at MSB position
        }
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
    shiftWrite(data[ctr + ctr2++], direction);
    if (ctr2 > 3)
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
        _delay_ms(70);
        if (!((ctr + 4) > 27))
        {
            ctr += 4;
        }
        else
        {
            cli();
            ctr = 0;
            direction = !direction;
            sei();
        }
    }
}
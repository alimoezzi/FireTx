#define F_CPU 1000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

char upDown = 0;

ISR(INT1_vect)
{
    if (upDown)
    {
        upDown = 0;
    }
    else
    {
        upDown = 0xFF;
    }
}

void TransmitUARTString(uint8_t *data);
void TransmitUART(uint8_t data);
uint8_t *ReceiceUARTString(void);
uint8_t ReceiceUART(void);
volatile unsigned char receicedData;

ISR(USART_RXC_vect){
    receicedData = UDR;
}

int main(void)
{
    DDRC |= (1 << PINC1); /* Make PORTC as output PORT*/
    PORTC &= ~(1 << PINC1);

    sei(); /* Enable Global Interrupt */
    // UART specs
    int UBBR_value = 25; // 2400 baud
    UBRRH = (unsigned char)UBBR_value >> 8;
    UBRRL = (unsigned char)UBBR_value;
    UCSRB = (1 << RXEN) | (1 << TXEN); // enable both tx and rx
    UCSRA |= (1 << USBS);              // use 2 stop bits
    UCSRC |= (3 << UCSZ0);             // use 8-bit dataframe
    UCSRB |= 1 << RXCIE;                    // recieve interrupt rx UART

    while (1)
    {
        if (!upDown)
        {
            PORTC &= ~(1 << PINC1);
        }
        else
        {
            PORTC ^= (1 << PINC1); /* Toggle PORTC */
            _delay_ms(1000);       /* Software debouncing control delay */
            TransmitUARTString("SARM");
        }
    }
}

uint8_t ReceiceUART(void)
{
    while (!(UCSRA & (1 << RXC)))
        ;
    return UDR;
}

uint8_t *ReceiceUARTString(void)
{
    unsigned char string[50], x, i = 0;
    do
    {
        x = ReceiceUART();
        string[i++] = x;
    } while ((x != '\0'));
    string[i] = '\0';

    return string;
}

void TransmitUART(uint8_t data)
{
    while (!(UCSRA & (1 << UDRE)))
        ;
    UDR = data;
}

void TransmitUARTString(uint8_t *data)
{
    unsigned char x, i = 0;
    do
    {
        x = *(data + (i++));
        TransmitUART(x);
    } while ((x != '\0'));
}

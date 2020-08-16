#define F_CPU 1000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void TransmitUARTString(uint8_t *data);
void TransmitUART(uint8_t data);
uint8_t *ReceiceUARTString(void);
uint8_t ReceiceUART(void);
volatile unsigned char receicedData;

ISR(USART_RXC_vect)
{
    receicedData = UDR;
}

int main(void)
{
    DDRC |= (1 << PINC0) | (1 << PINC1) | (1 << PINC2);     /* Make PORTC as output PORT*/
    PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2)); // choose first LM35

    DDRD |= 1 << PIND5;     /* PIND5(OCR1A) as Output */
    PORTD &= ~(1 << PIND5); /* Make pull up low */
    // selecting WGM mode
    // The max can be set in 8 to 10bit using ICR1 and while using CTC OCR1A
    // 1. Phase correct:  /\.
    // 2. Fast PWM /|(19999)/|/|
    // we here us mode 14 (FAST PWM) period is determined by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    ICR1 = 19999;
    // Creating Period
    // if we have servo accepting input at 50Hz(20mx)
    // microcontroller @ 1mhz have 1_000_000 / 50 = 20_0000 cyles in a sec to set for ICR1
    // using this we don't need prescaling
    TCCR1B |= (1 << CS10);
    // output can be inverted  or not to use inverted we use COM1A1, COM1AO and output is OCR1A
    // COMA10 only for inverted mode
    TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
    // the pulse then should be started @ 19_999 - 2000 to reach 20ms
    sei(); /* Enable Global Interrupt */
    // UART specs
    int UBBR_value = 25; // 2400 baud
    UBRRH = (unsigned char)UBBR_value >> 8;
    UBRRL = (unsigned char)UBBR_value;
    UCSRB = (1 << RXEN) | (1 << TXEN); // enable both tx and rx
    UCSRA |= (1 << USBS);              // use 2 stop bits
    UCSRC |= (3 << UCSZ0);             // use 8-bit dataframe
    UCSRB |= 1 << RXCIE;               // recieve interrupt rx UART

    while (1)
    {
        OCR1A = ICR1 - 800;
        _delay_ms(1000);
        OCR1A = ICR1 - 2200;
        _delay_ms(1000);
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

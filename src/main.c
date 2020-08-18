#define F_CPU 1000000L
#define LAST(k, n) ((k) & ((1 << (n)) - 1))
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void TransmitUARTString(uint8_t *data);
void TransmitUART(uint8_t data);
uint8_t *ReceiceUARTString(void);
uint8_t ReceiceUART(void);
volatile unsigned char receicedData;
volatile int indx = 0;

unsigned char Tempu[6];

int main(void)
{
    Tempu[5] = '\n';
    Tempu[6] = '\0';
    DDRC = 1;  /* Make PORTC as output PORT*/
    PORTC = 0; // choose first LM35

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
    // configure ADC 2 option
    DDRA = 0;  // set PINA0 for input
    PORTA = 0; // set PINA0 for low pulling
    /* Having ADC start conversion through  interrupts or finilize through interrupts
    or through determining if a flag is set and reading the resullt */
    ADCSRA |= 1 << ADIE; // enable interruts function in ADC ( the ADC conersion completion interrupt)

    ADMUX &= ~(1 << ADLAR);                                                // enable ADC in 8-bit
    ADMUX &= ~(1 << MUX0 | 1 << MUX1 | 1 << MUX2 | 1 << MUX3 | 1 << MUX4); // enable ADC in 8-bit
    // enable prescaller for ADC - determine by our clock by default we need 50KHZ to 200KHZ (1_000_000 / 50_000 = 20) (1_000_000 / 200_000 = 5)
    // for 8-bits going above 200KHZ is ok
    ADCSRA |= 1 << ADPS2;  // 16 for prescale
    ADMUX |= (1 << REFS0); // choose 2.5 internal for refrence voltage
    ADCSRA |= 1 << ADEN;   // turn on ADC
    // UART specs
    int UBBR_value = 25; // 2400 baud
    UBRRH = (unsigned char)UBBR_value >> 8;
    UBRRL = (unsigned char)UBBR_value;
    UCSRB |= (1 << RXEN) | (1 << TXEN); // enable both tx and rx
    UCSRA |= (1 << USBS);               // use 2 stop bits
    UCSRC |= (3 << UCSZ0);              // use 8-bit dataframe
    UCSRB |= 1 << RXCIE;                // recieve interrupt rx UART
    sei();                              /* Enable Global Interrupt */
    // ADCSRA |= 1 << ADSC;                // start first conversion to start conversion we high the ADC start conversion bit

    while (1)
    {
        OCR1A = ICR1 - 800;
        _delay_ms(1000);
        OCR1A = ICR1 - 2200;
        _delay_ms(1000);
        TransmitUARTString(Tempu);
        ADCSRA |= 1 << ADSC; // start again conversion to start conversion we high
    }
}

ISR(USART_RXC_vect)
{
    receicedData = UDR;
}

ISR(ADC_vect)
{

    if (indx == 4)
    {
        indx = 0;
    }
    else
    {
        uint8_t theLow = ADCL;
        uint16_t tenBit = (theLow | (ADCH << 8));
        Tempu[indx] = (tenBit/5);
        switch (indx)
        {
        case 0:

            ADMUX = 65;
            break;
        case 1:
            ADMUX = 66;

            break;
        case 2:
            ADMUX = 67;

            break;
        case 3:
            ADMUX = 64;
            break;

        default:
            break;
        }
        indx += 1;

        _delay_ms(1000);     // start conversion every one seconde
        ADCSRA |= 1 << ADSC; // start again conversion to start conversion we high
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
    unsigned char x, i = 0;
    unsigned char *string = (unsigned char *)malloc(50 * sizeof(char));
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

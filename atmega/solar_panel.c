#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 1000000UL
#define BAUD 2400
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

// define posible commands we can recieve via UART
#define READ_0 ((unsigned char) 0)
#define READ_1 ((unsigned char) 1)
#define READ_2 ((unsigned char) 2)
#define READ_3 ((unsigned char) 3)
#define MOVE_RIGHT ((unsigned char) 4)
#define MOVE_LEFT ((unsigned char) 5)
#define OK ((unsigned char 0))

void init_uart () {
    // set baudrate
    UBRRH = (BAUDRATE >> 8);
    UBRRL = BAUDRATE;
    // enable receiver and transmitter and interupts
    UCSRB |= (1 << TXEN) | (1 << RXEN) | (1 << RXCIE);
    // 8 bit data format
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

void init_adc () {
    // enable ADC and set ADCSRA register with division fasctor 32
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);
    // use AVCC as VRef (reference voltage) and 8 bit precision
    ADMUX = (1 << REFS0) | (1 << ADLAR);
}

void init_gpio () {
    // define output ports
    DDRD = (1 << PIND4) | (1 << PIND7);
    DDRB = (1 << PINB2) | (1 << PINB3) | (1 << PINB6) | (1 << PINB7);
    // make them high since the 'ouput' is active when low
    PORTD = 0xFF;
    PORTB = 0xFF;
}

void uart_send (unsigned char data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

unsigned char read_voltage(unsigned char pin) {
    // select pin to read
    // (careful! don't forget to alse set REFS0 and ADLARsince it uses same register)
    ADMUX = pin | (1 << REFS0) | (1 << ADLAR);
    // start conversion
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    // since we use 8 bit percision we only need to read ADCH register
    return ADCH;
}

void move_right () {
    // we have to have PIND7 and PINB2 allways low, since they are 'ground'
    PORTB = ~(1 << PINB3) & ~(1 << PINB2); // orange and white
    PORTD = 0xFF;
    _delay_ms(2000);
    PORTB = ~(1 << PINB6); // red
    PORTD = ~(1 << PIND7); // black
    _delay_ms(2000);
    PORTB = ~(1 << PINB7) & ~(1 << PINB2); // blue and white
    PORTD = 0xFF;
    _delay_ms(2000);
    PORTD = ~(1 << PIND4) & ~(1 << PIND7); // yellow and black
    PORTB = 0xFF;
    _delay_ms(2000);

    // reset all the ports
    PORTD = 0xFF;
    PORTB = 0xFF;
}

int main () {
    init_uart();
    init_adc();
    init_gpio();
    sei();
    while (1) {};
}

// handle interupts
ISR(USART_RXC_vect) {
    unsigned char command = UDR;
    unsigned char res;
    switch(command) {
        case READ_0:
        case READ_1:
        case READ_2:
        case READ_3:
            res = read_voltage(command);
            uart_send(res);
            break;
        case MOVE_RIGHT:
            move_right();
            break;
    }
}

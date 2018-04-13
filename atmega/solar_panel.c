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
#define MOVE_RIGHT_FULL ((unsigned char) 6)
#define MOVE_LEFT_FULL ((unsigned char) 7)
#define OK ((unsigned char) 0)

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
    // Wire colors:
    // -PINB6: WHITE
    // -PINB7: BLACK
    // -PIND5: ORANGE
    // -PIND6: BLUE
    // -PIND7: RED
    // -PINB0: YELLOW
    DDRD = (1 << PIND5) | (1 << PIND6) | (1 << PIND7);
    DDRB = (1 << PINB0) | (1 << PINB6) | (1 << PINB7);
    // make them high since the 'ouput' is active when low
    PORTD = 0xFF;
    PORTB = 0xFF;
    // PORTD &= ~(1 << PIND5);
    // PORTD &= ~(1 << PIND6);
    // PORTD &= ~(1 << PIND7);
    // PORTB &= ~(1 << PINB0);
    // PORTB &= ~(1 << PINB6);
    // PORTB &= ~(1 << PINB7);
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
    // move order: orange -> red -> blue -> yellow
    // STEP 1:
    PORTD = ~(1 << PIND5); // orrange
    PORTB = ~(1 << PINB6); // white
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);
    // STEP 2:
    PORTD = ~(1 << PIND7); // red
    PORTB = ~(1 << PINB7); // black
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);
    // STEP 3:
    PORTD = ~(1 << PIND6); // blue
    PORTB = ~(1 << PINB6); // white
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);
    // STEP 4:
    PORTD = 0xFF; // we don't use it
    PORTB = ~(1 << PINB0) & ~(1 << PINB7); // yeelow and black
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);

    // reset all the ports
    PORTD = 0xFF;
    PORTB = 0xFF;
    uart_send(OK);
}
void move_left () {
    // move order: blue -> red -> orange -> yellow
    // STEP 1:
    PORTD = ~(1 << PIND6); // blue
    PORTB = ~(1 << PINB6); // white
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);
    // STEP 2:
    PORTD = ~(1 << PIND7); // red
    PORTB = ~(1 << PINB7); // black
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);
    // STEP 3:
    PORTD = ~(1 << PIND5); // orange
    PORTB = ~(1 << PINB6); // white
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);
    // STEP 4:
    PORTD = 0xFF; // we don't use it
    PORTB = ~(1 << PINB0) & ~(1 << PINB7); // yeelow and black
    _delay_ms(200);
    PORTD = 0xFF;
    PORTB = 0xFF;
    _delay_ms(200);

    // reset all the ports
    PORTD = 0xFF;
    PORTB = 0xFF;
    uart_send(OK);
}

void move_right_full () {
    // move order: orange&red -> red&blue -> blue&yellow -> yellow&orange
    // STEP1:
    PORTD = ~(1 << PIND5) & ~(1 << PIND7); // orange and red
    _delay_ms(200);

    // STEP 2:
    PORTD = ~(1 << PIND6) & ~(1 << PIND7); // blue and red
    _delay_ms(200);

    // STEP 3:
    PORTB = ~(1 << PINB0); // yellow
    PORTD = ~(1 << PIND6); // blue
    _delay_ms(200);

    // STEP 4:
    PORTB = ~(1 << PINB0); // yellow
    PORTD = ~(1 << PIND5); // orange
    _delay_ms(200);
}

void move_left_full () {
    // move order: yellow&blue -> blue&red -> red&orange -> orange&yellow
    // STEP1:
    PORTB = ~(1 << PINB0); // yellow
    PORTD = ~(1 << PIND6); // blue
    _delay_ms(200);

    // STEP 2:
    PORTD = ~(1 << PIND6) & ~(1 << PIND7); // blue and red
    _delay_ms(200);

    // STEP 3:
    PORTD = ~(1 << PIND5) & ~(1 << PIND7); // orange and red
    _delay_ms(200);

    // STEP 4:
    PORTB = ~(1 << PINB0); // yellow
    PORTD = ~(1 << PIND5); // orange
    _delay_ms(200);
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
        case MOVE_LEFT:
            move_left();
            break;
        case MOVE_RIGHT_FULL:
            move_right_full();
            break;
        case MOVE_LEFT_FULL:
            move_left_full();
            break;
    }
}

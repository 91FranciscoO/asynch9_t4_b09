// The native serial support happens via UART
#include <Arduino.h>

#define MASTER_ADDR 15
#define SLAVE1_ADDR 13
#define SLAVE2_ADDR 14

#define USART_BUFFER_MAX 100 // Buffersizer in Byte

#define LED_PIN 13
#define ADDR3_PIN 11
#define ADDR2_PIN 10
#define ADDR1_PIN 9
#define ADDR0_PIN 8
#define BUT1_PIN 7
#define BUT2_PIN 6
#define REDE 2
#define BAUD 9600 // BAUD Rate
#define FOSC 1843200
#define UBRR_VAL FOSC/16/BAUD - 1
uint8_t dat = 1;
uint8_t dat2 = 0;

/*USART has to be initialized before any communication can take place*/
void asynch9_init(unsigned int ubrr) {
    // UCSR0B - USART Control Status Register B
    UCSR0B = (1<<UCSZ02) | (1<<TXEN0) | (1<<RXEN0) ; // enables transmission + RECEP+ 9CHARC
    // UCSR0C - USART Control Status Register C
    UCSR0C = (0<<UMSEL01) | (0<<UMSEL00); // Asynchronous USART selected
    // UCSR0A - USART Control Status Register A
    UCSR0A = (0<<U2X0) | (1<<MPCM0); // asynchronous normal operation, F_CPU((16*BAUD))-1; 
    // setting frame format, 9 bit
    UCSR0C =  (1<<UCSZ01) | (1<<UCSZ00);
    // parity mode disabled
    UCSR0C = (0<<UPM01) | (0<<UPM00);
    // Using only 1 stop bit
    UCSR0C = (0<<USBS0);
    // Set USART BAUD Rate, defined in definitions above
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
}

void send_addr(uint8_t addr) {
    // put your code here, to send a data byte:
    while(!(UCSR0A & (1<<UDRE0)));
    //Copy 9th bit to TXB8
    UCSR0B |= (1<<TXB80);
    UDR0=addr;
}

void send_data(uint8_t data) {
    // put your code here, to send a data byte:
    while(!(UCSR0A & (1<<UDRE0)));
    //Copy 9th bit to TXB8
    UCSR0B &= ~(1<<TXB80);
    UDR0 = data;
}

uint16_t get_data(void) {
    // put your code here, to receive a data byte using multi processor communication mode:
    unsigned char status, resh, resl;
    //Wait for data to be received
    while (!(UCSR0A & (1<<RXC0)));
    // Get status and 9th bit, then data from buffer
    status = UCSR0A;
    resh = UCSR0B;
    resl = UDR0;
    // If error, return -1
    if (status & ((1<<FE0)|(1<<DOR0)|(1<<UPE0))) 
        return -1;
    // Filter the 9th bit, then return
    resh = (resh >> 1) & 0x01;
    //return ((resh << 8) | resl);
    return ((resh << 8) | resl);
}

void setup() {
    // put your setup code here, to run once:
    asynch9_init(UBRR_VAL);

    pinMode(BUT1_PIN,INPUT_PULLUP);
    pinMode(BUT2_PIN,INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(REDE, OUTPUT);
    pinMode(ADDR3_PIN,INPUT_PULLUP);
    pinMode(ADDR2_PIN,INPUT_PULLUP);
    pinMode(ADDR1_PIN,INPUT_PULLUP); //slave 2, isto é 0
    pinMode(ADDR0_PIN,INPUT_PULLUP); //slave 1, isto é 0   
}

void loop() {
    // put your main code here, to run repeatedly:
    int addr0 = digitalRead(ADDR0_PIN);
    int addr1 = digitalRead(ADDR1_PIN);
    int addr2 = digitalRead(ADDR2_PIN);
    int addr3 = digitalRead(ADDR3_PIN);

    int but1 = digitalRead(BUT1_PIN);
    int but2 = digitalRead(BUT2_PIN);

    uint8_t addr = (addr3<<3)|(addr2<<2)|(addr1<<1)|(addr0<<0);
    
    if (addr == MASTER_ADDR){
        digitalWrite(REDE, HIGH);
        //unsigned int dat1 = get_data();
        if (but1 == LOW){
            send_addr(14);
            send_data(dat);
            //digitalWrite(LED_PIN, HIGH); para confirmar
            delay (10);
        }
        else if (but2 == LOW){
            send_addr(13);
            send_data(dat);
            //digitalWrite(LED_PIN, HIGH); para confirmar
            delay (10);
        }
    }
    else {
        unsigned int da = get_data();
        uint8_t ninth_bit = (da>>8);
        uint8_t fullbyte=da;
        digitalWrite(REDE, LOW);
        if (ninth_bit == 1){
            if (fullbyte == addr){
                UCSR0A &= ~(1<<MPCM0);
            }
            else
            {
                UCSR0A |= (1<<MPCM0);
            }
            
        }
        else if (ninth_bit == 0){
            if (fullbyte == dat){
                digitalWrite(LED_PIN, HIGH);
                delay(25);
                digitalWrite(LED_PIN, LOW);
            }
        }

    }
}
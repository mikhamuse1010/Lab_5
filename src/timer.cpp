// Nick Blanchard, Nicholas Gullo, Konner Curtis, Salman Marafie
// ECE 372 Lab 5
// 11/29/21

#include <avr/io.h>

void initTimer1(){

    // Set waveform generation bits to CTC mode
    // Check datasheet to see which bits to set to what
    // According to datasheet: WGM13 = 0, WGM12 = 1, WGM11 = 0, WGM10 = 0 (for mode 4, CTC mode)
    // Datasheet also shows the locations of these four bits
    // Register TCCR1A: WGM10 and WGM11
    // Register TCCR1B: WGM12 and WGM13

    TCCR1A &= ~(1 << WGM10);    // Clear WGM10 bit in TCCR1A
    TCCR1A &= ~(1 << WGM11);    // Clear WGM11 bit in TCCR1A
    TCCR1B |=  (1 << WGM12);    // Set WGM11 bit in TCCR1B
    TCCR1B &= ~(1 << WGM13);    // Clear WGM13 bit in TCCR1B

}



void delayMs(unsigned int delay){
   // For the ms delay, I will use Timer 1 (the 16-bit timer)

    // this function takes an argument called delay which is the number of ms to delay
    //
    // the value of unsigned int delay can be any number that can fit within the scope of an unsigned int
    // (because I am using software looping, there is no theoretical harware limitation to how high we can count, during the delay)
    //

    // CALCULATIONS:
    // OCR1A = Td * Fclk / Ps - 1
    // OCR1A = (1 * 10^-3) *  (16 * 10^6) / 64 - 1 = 249
    // Prescaler: 64, OCR0A: 249

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // clear the TCNT1 registers (the counter registers)
    TCNT1 = 0;
    // TCNT1H = 0;
    // TCNT1L = 0;
    

    // Set the values for OCR1A
    // REMEMBER: OCR1A is an 16-bit register that is accessed over an 8-bit bus.
    // Therefore, we can store a value up to 2^16-1 = 65535. In two different byte registers.
    // Recall Count = (Td * frequency)/Prescaler = (1 ms * 16Mhz)/64 - 1 = 250 - 1 = 249
    // OCR1A = 249; each ms delay requires us to count up to exactly 249
    OCR1A = 249;


    // Set the OCF1A overflow flag down (by writing a 1 to it - it's unintuitive but correct)
    TIFR1 |= (1 << OCF1A);


    // set the prescaler to 64 and start the timer
    // CS12 = 0, CS11 = 1, CS10 = 1 for a prescaler of 1 (no prescaler)
    // After the following lines run, the clock is ON
    TCCR1B &= ~(1 << CS12); // Clear CS12
    TCCR1B |= (1 << CS11);  // Set CS11
    TCCR1B |= (1 << CS10);  // Set CS10


    // The following loop uses a 1 ms base delay to create a multi-ms total delay
    for (unsigned int i = 0; i < delay; i++) {

        // Checking the output compare flag
        while (!(TIFR1 & (1 << OCF1A))); // do nothing while flag is down
        // NOTE: OCF1A, the flag bit, is stored in the TIFR1 register ????

        // NOTE: When we reach this point in the code, the counter has reached the compare value (249) and the flag is triggered.

        // Clear the OCF1A flag
        TIFR1 |= (1 << OCF1A);

        // clear the TCNT1 register (the counter register)
        TCNT1 = 0;
    }

    // Turn the timer off before exiting
    TCCR1B &= ~(1 << CS11 | 1 << CS10);
}
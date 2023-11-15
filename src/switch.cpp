// Nick Blanchard, Nicholas Gullo, Konner Curtis, Salman Marafie
// ECE 372 Lab 5
// 11/29/21

#include <avr/io.h>

/*
 * Initializes pull-up resistor on PE0 and sets it into input mode
 */
void initSwitchPE0(){

    // Initialize PE0 (pin 0) for input (set DDR to input). Enable pullup resistor.
    // NOTE: To set the port to input, we must 'clear' the bit by setting it to zero
    // while simultaneously preserving the other bits in the DDR register.

    DDRE &= ~( 1 << DDE0);  // Clear the bit in DDRE0
    PORTE |= (1 << PORTE0); // Enable pullup resistor by writing 1 to the port.

    // Pin-Change Interrupts must be enabled for pin PE0
    // We basically want to use INT0 as a pin change interrupt
    //EICRA |= (1 << ISC00); EICRA &= ~(1 << ISC01); // Set INT0 to trigger asynchronously on any edge (high or low)
    PCICR |= (1<<PCIE1);
    PCMSK1 |= (1<<PCINT8);
    //EIMSK |= (1 << PCINT8);   // Enable INT8 in the EIMSK register
}

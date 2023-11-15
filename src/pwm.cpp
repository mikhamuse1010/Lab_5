// Nick Blanchard, Nicholas Gullo, Konner Curtis, Salman Marafie
// ECE 372 Lab 5
// 11/29/21

#include <avr/io.h>
#include "pwm.h"

void initPWMTimer4() {

    // set DDR for output pin
    // We are using PORTH5 (pin 8 on board) (this is the OCR4C output pin)
    //
    // NOTE: When using Mode 15, OCR4A is needed to set the top value. Therefore, we must
    // use OCR4C for out output compare, and out output pin must be PH5 (pin 8).
    DDRH |= (1 << DDH5);

    // We will use PWM mode 15 (so we can vary frequency)
    // WGM13 = 1, WGM12 = 1, WGM11 = 1, WGM10 = 1 (these bits are stored in the TCCR4A and TCCR4B registers)
    TCCR4A |= ((1 << WGM40) | (1 << WGM41));
    TCCR4B |= (1 << WGM43);
    TCCR4B |= (1 << WGM42);

    // Frequency calculation:
    // We will use a prescaler of 1. The 'top value' is OCR4A.
    // fpwm = fclk / (prescaler * (1 + top value)), we will use a prescaler of 1

    // Set prescaler to 1
    TCCR4B |= (1 << CS40);
    TCCR4B &= ~(1 << CS41);
    TCCR4B &= ~(1 << CS42);

    // Set to non-inverting mode - output starts high and then is low
    // See table 17-4
    // Non-inverting: COM4C1 = 1, COM4C0 = 0 (located in TCCR1A register)
    TCCR4A |= (1 << COM4C1);
    TCCR4A &= ~(1 << COM4C0);
    // NOTE: When using Mode 15 (variable frequency), we use OCR4C for output compare. Therefore, we
    // need to use COM4C1 and COM4C0 here. Otherwise, we would use COM4A1 and COM4A0.


    // NOTE: OCR4C and OCR4A are NOT set here. To determine the frequenc and duty cycle,
    // they must be set by the change_frequency function.
}

void change_frequency(unsigned int freq_val) {
    // To vary the frequency, we need to to change OCR4A (the TOP value)
    OCR4A = 16000000/freq_val;  // (ignoring the 1 in the formula)
}

void change_duty_cycle(double duty_val) {
    // To vary the duty cycle, we need to to change OCR4C (the COMPARE value)
    OCR4C = OCR4A * duty_val;
}
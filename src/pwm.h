// Nick Blanchard, Nicholas Gullo, Konner Curtis, Salman Marafie
// ECE 372 Lab 5
// 11/29/21

#ifndef PWM_H
#define PWM_H
void initPWMTimer4();
void change_frequency(unsigned int freq_val);
void change_duty_cycle(double duty_val);
#endif
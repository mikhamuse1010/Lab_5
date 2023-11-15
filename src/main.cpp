// Nick Blanchard, Nicholas Gullo, Konner Curtis, Salman Marafie
// ECE 372 Lab 5
// 11/29/21

// Header and source files
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "switch.h"
#include "timer.h"
#include "pwm.h"
#include "spi.h"
#include "i2c.h"

// MPU I2C Slave address and internal data addresses
#define SLA 0x68  // MPU_6050 address with PIN AD0 grounded
#define PWR_MGMT  0x6B
#define WAKEUP 0x00
#define SL_MEMA_XAX_HIGH  0x3B
#define SL_MEMA_XAX_LOW   0x3C
#define SL_MEMA_YAX_HIGH  0x3D
#define SL_MEMA_YAX_LOW   0x3E
#define SL_MEMA_ZAX_HIGH  0x3F
#define SL_MEMA_ZAX_LOW   0x40


//----------------------------------------------------------------------//
// State machines

/*
 * Define a set of states that can be used in the state machine using an enum.
 */
typedef enum stateEnum {wait_press, debounce_press, wait_release, debounce_release} stateEnum;
typedef enum LEDstate {smile, frown} LEDstate;
// Initialize states. Remember to use volatile
volatile stateEnum button_state = wait_press; // ASSUMING WE BEGIN IN A STATE WHERE THE BUTTON IS NOT BEING PRESSED
volatile LEDstate AccelerationState = smile;    // Initialize the state to smile face 

//----------------------------------------------------------------------//
// Main function
int main(void) {

  Serial.begin(9600);   // Serial (for debugging)
  
  // VARIABLES
  int freq_count = 500;
  // int data = 0;
  bool trigger = false;

  // HARDWARE INITIALIZATIONS
  initSwitchPE0();      // Initialize the switch
  SPI_MASTER_Init();    // Initialize the SPI module
  initPWMTimer4();      // Initialize timer 4 for PWM mode 15
  initTimer1();         // Initialize timer 1 (for millisecond delay)
  initI2C();            // Initialize I2C bus
  sei();                // Enable global interrupts

  // SETUP DISPLAY
  write_execute(0x0A,0x08); // Brightness control
  write_execute(0x0B,0x07); // Scan all rows and columns
  write_execute(0x0C,0x01); // Set shutdown register to normal operation
  write_execute(0x0F,0x00); // Display test register set to normal operation

  // SETUP I2C
  signed int x_val = 0;
  signed int y_val = 0;
  signed int z_val = 0;
  StartI2C_Trans(SLA);
  write(PWR_MGMT);// address on SLA for Power Management
  write(WAKEUP);  // send data to Wake up from sleep mode
  StopI2C_Trans();


  // BEGIN WHILE LOOP
  while(1) {
    
    // BUZZER:
    if(trigger){
      change_duty_cycle(0.05);
      change_frequency(freq_count);
      freq_count = freq_count + 500;
      if(freq_count > 10000){
        freq_count = 500;
      }
    }
    else{
      freq_count = freq_count;
      change_duty_cycle(0);
    }
  

    // READ ACCELEROMETER (Single-byte read)
    Read_from(SLA,SL_MEMA_XAX_HIGH);
    x_val= Read_data(); // Read upper byte value
    Read_from(SLA,SL_MEMA_XAX_LOW);
    // Shifted by 8 since the byte is 16-bit value
    x_val = (x_val << 8 )| Read_data(); // Append lower byte value
    StopI2C_Trans();

    Read_from(SLA,SL_MEMA_YAX_HIGH);
    y_val= Read_data(); // Read upper byte value
    Read_from(SLA,SL_MEMA_YAX_LOW);
    y_val = (y_val << 8 )| Read_data(); // Append lower byte value

    Read_from(SLA,SL_MEMA_ZAX_HIGH);
    z_val= Read_data(); // Read upper byte value
    Read_from(SLA,SL_MEMA_ZAX_LOW);
    z_val = (z_val << 8 )| Read_data(); // Append lower byte value


    Serial.print("X-axis accel =  ");
    Serial.println(x_val);

    Serial.print("Y-axis accel =  ");
    Serial.println(y_val);

    Serial.print("Z-axis accel =  ");
    Serial.println(z_val);
    Serial.println("\n");

    delayMs(1000);

    // DISPLAY STATE MACHINE:
    // Acceleration threshold value: 10000 -> 45 degrees
    if ((fabs(x_val) > 10000) |  (fabs(y_val) > 10000)) {
      AccelerationState = frown;
    }
    else {
      AccelerationState = smile;
    }

    switch(AccelerationState){
        case smile:
          write_execute(0x01,0b00000000);
          write_execute(0x02,0b00000000);
          write_execute(0x03,0b00100100);
          write_execute(0x04,0b00000000);
          write_execute(0x05,0b10000001);
          write_execute(0x06,0b01100110);
          write_execute(0x07,0b00111100);
          write_execute(0x08,0b00000000);
          break;
        case frown:
          trigger = true; // Enable buzzer sound
          write_execute(0x01,0b00000000);
          write_execute(0x02,0b00000000);
          write_execute(0x03,0b00100100);
          write_execute(0x04,0b00000000);
          write_execute(0x05,0b00011000);
          write_execute(0x06,0b01100110);
          write_execute(0x07,0b10000001);
          write_execute(0x08,0b00000000);
          break;
    } // end display state machine

    // BUTTON STATE MACHINE:
    switch(button_state) {

      case wait_press:
        // Do nothing; we are waiting for the button to be pressed.
        //Serial.println("wait press");
        break;
      case debounce_press:
        // Wait for the noisy 'debounce' state to pass. Then, we are awaiting release.
        delayMs(1);
        button_state = wait_release;
        break;
      case wait_release:
        // Do nothing; we are waiting for the button to be released.
         trigger = false;
        break;
      case debounce_release:
        // The button has been released.
        // Wait for the noisy 'debounce' state to pass. Then, we are awaiting press.
        delayMs(1);
        button_state = wait_press;
        break;

    } // end switch state machine  
  } // end while loop
} // end main



//----------------------------------------------------------------------//
// Interrupt

ISR (PCINT1_vect) {

  Serial.println("INTERUPT");
  // When the interrupt flag is triggered:
  if (button_state == wait_press) {
    // The button was pushed while we were waiting for it to be pressed. Enter the first debounce state.
    button_state = debounce_press;
  }
  else if (button_state == wait_release) {
    // The button was released while we were waiting for it to be released. Enter the second debounce state.
    button_state = debounce_release;

    change_duty_cycle(0); // Turn the buzzer off
  } // end if/else
  // If the flag triggers while the button is 1 in one of the noisy debounce states, we do nothing.
} // end ISR

// Nick Blanchard, Nicholas Gullo, Konner Curtis, Salman Marafie
// ECE 372 Lab 5
// 11/29/21

#include <avr/io.h>
#include "Arduino.h"
#include "i2c.h"

// Wait for the TWI to finish its current task (read or write)
// In other words, while the flag is low, do nothing
#define wait_for_completion while(!(TWCR & (1 << TWINT)));
#define WRITE_BIT 0;
#define READ_BIT 1;


void initI2C() {
  
  // Wake up/jump start the I2C module on the ATMEGA2560 power management register
  PRR0 &= ~(1 << PRTWI);
  
  // Set prescaler to 1
  TWSR |= (1 << TWPS0);  // prescaler = (4 ^ 1) = 4
  TWSR &= ~(1 << TWPS1); //

  // 𝑇𝑊𝐵𝑅=((𝐶𝑃𝑈 𝐶𝑙𝑜𝑐𝑘 𝑓𝑟𝑒𝑞𝑢𝑒𝑛𝑐𝑦)/(𝑆𝐶𝐿 𝑓𝑟𝑒𝑞𝑢𝑒𝑛𝑐𝑦)−16)/(2∗〖(4)〗^𝑇𝑊𝑃𝑆 )
  // Start with SCL frequency of 10kHz to have a slower and smooth output testing
  // Thereafter, a larger SCL frequency in the hundreds of thousand in frequency can be used
  // The slave device is ensured to be able to respond to the SCL frequency of 10kHz based on its datasheet
  // TWBR = [(f_CPU/f_SCL)-16]/(2*(4^TWPS)) = [(16M/10k)-16]/(2*(4^1)) = 198
  TWBR = 0xC6;

  // Enable two wire interface
  TWCR |= (1 << TWINT )| (1 << TWEN);
  // TWINT -> Clear the interrupt flag (or initiate the I2C transmission)
  // TWEN -> Enable the I2C module

}

// Trigger the I2C action
void triggerI2C() {
	
	TWCR = (1 << TWINT) | (1 << TWEN);  // Trigger action: clear flag and enable TWI
}


// Initiates a start condition and calls slave device with SLA
void StartI2C_Trans(unsigned char SLA) {
  
  // Setup I2C transmission
  TWCR = (1 << TWINT) | ( 1 << TWSTA) | (1 << TWEN); // clear TWINT, intiate a start condition and enable
  // TWINT -> Interrupt flag; flagged when reads or writes are completed
  // TWSTA -> Start condition bit
  // TWEN -> Enable the I2C module

  wait_for_completion;   // Wait until the setup above is completed

  // The TWCR are all equals without OR because we have to manipulate every single bit in the register every time a write is performed on it
  // And so, overwriting every single bit in the register is required after every transmission
  // For example, the TWCR is set as the following.
  // ----------------------------------------------------------------
  // TWCR  | TWINT | TWEA | TWSTA | TWSTO | TWWC | TWEN |
  // ----------------------------------------------------------------
  // value |   1   |  X   |   1   |   0   |   X  |   1  |
  // ----------------------------------------------------------------
  // And by using EQUAL (=) instead of OR EQUAL (|=), the bits set in the previous one is reset, and only set the desired bits
  // ----------------------------------------------------------------
  // TWCR  | TWINT | TWEA | TWSTA | TWSTO | TWWC | TWEN |
  // ----------------------------------------------------------------
  // value |   1   |  X   |   0   |   0   |   X  |   1  |
  // ----------------------------------------------------------------
  
  // ACK and NACK occur behind the scene

  // Send the address to initiate communication with the slave device
  // Inform slave device that data is to be written to it at the specified address
  TWDR = (SLA << 1) | WRITE_BIT; // Slave address + write bit '0'

  TWCR = (1 << TWINT) | (1 << TWEN);  // Trigger action: clear flag and enable TWI
  wait_for_completion;   // Wait until the I2C action above is completed
}


// Sends a stop condition to stop I2C transmission
void StopI2C_Trans() {

	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); // Trigger action:  send stop condition
	// TWINT -> Interrupt flag; flagged when reads or writes are completed
	// TWEN -> Enable the I2C module
	// TWSTO -> Stop condition
}


// Loads the data passed into the I2C data register and transmits
void write(unsigned char data){
	// Load data into TWDR register
	// OR
	// Load register address of a slave device
	TWDR = data; // Register value in the data register

	triggerI2C();

	// Wait until the restart sequence above is completed
  	wait_for_completion;
}



// Setup read mode for the slave device
void Read_from(unsigned char SLA, unsigned char MEMADDRESS){

	// Start from write first and then to switch to read by performing the restart sequence of the I2C
  	StartI2C_Trans(SLA); // device address
 
	// Inform Master the targeted register address of the slave device
  	write(MEMADDRESS);
	
	// Restart to switch to read mode
	// In other words, switch master to read (receiver) mode and slave to transmitter
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Set another start condition
	// TWINT -> Interrupt flag; flagged when reads or writes are completed
	// TWSTA -> Start condition bit
	// TWEN -> Enable the I2C module

	// Wait until the restart sequence above is completed
	wait_for_completion;

	// Send slave device address to Master and inform to read this time
	TWDR = (SLA << 1) | READ_BIT; // 7 bit address for slave plus read bit

	// TWEA is the acknowledge bit the master use upon the read operation
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Trigger with master sending ack
	// Set the acknowledge bit so that when the data is sent to the master, the Microcontroller will send that ACK bit to the slave device
	// Here the acknowledge bit is set to 1 even though acknowledge is represented as 0

	wait_for_completion; // Wait until the the data is sent back and the Master has sent the acknowledged bit to the slave device are completed

	// Check the status register to see whether the communication is successful
	// The recent I2C communication is retrieved
    Serial.println(TWSR & (0xF8),HEX); // Bitmask the upper 5 bits as only those are the status bits in the TWSR register
	Serial.flush();
	// See table 24-4 in the ATMEGA2560 datasheet for more specific conditions to see the status

	TWCR = (1 << TWINT) | (1 << TWEN); // master can send a nack now
	wait_for_completion; // Wait until the action above is completed

	// Stop condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	// After this function is executed, the TWDR register has the data from SLA that Master wants to read
}


// Returns the last byte from the data register
unsigned char Read_data()
{
  return TWDR;
}
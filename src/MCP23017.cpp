#include "MCP23017.h"

//#define USE_SMBUS // if undefined, uses i2c-dev
#ifdef USE_SMBUS
extern "C" {
#include <i2c/smbus.h>
}
#else // USE_SMBUS
#include <linux/i2c-dev.h>
// heuristic to guess what version of i2c-dev.h we have:
#ifndef I2C_SMBUS_BLOCK_MAX
// If this is not defined, we have the "stock" i2c-dev.h
// so we include linux/i2c.h
#include <linux/i2c.h>
typedef unsigned char i2c_char_t;
#else
typedef char i2c_char_t;
#endif
#endif // USE_SMBUS

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14


#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255

MCP23017::MCP23017(int bus, int address) {
    kI2CBus = bus;           // I2C bus
    kI2CAddress = address ; // Address of MCP23017; defaults to 0x20
    error = 0 ;
}

MCP23017::~MCP23017() {
    closeI2C() ;
}

//helper function to replace bitRead function from Arduino library
bool MCP23017::bitRead(uint8_t num, uint8_t index)
{
    return (num >> index) & 1;
}


//helper function to replace bitWrite function from Arduino library
void MCP23017::bitWrite(uint8_t &var, uint8_t index, uint8_t bit)
{
    uint new_bit = 1 << index;
    if(bit)
    {
        var = var | new_bit;
    }
    else {
        new_bit = ~new_bit;
        var = var & new_bit;
    }
}


//open I2C communication
bool MCP23017::openI2C()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
       error = errno ;
       return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0) {

        // Could not open the device on the bus
        error = errno ;
        return false ;
    }
    // set defaults!
    // all inputs on port A and B
    if(writeRegister(MCP23017_IODIRA,0b11111110))
        return false;
    if(writeRegister(MCP23017_IODIRB,0b11111110))
        return false;
    return true ;
}

//close I2C communication
void MCP23017::closeI2C()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }
}



/**
 * Bit number associated to a given Pin
 */
uint8_t MCP23017::bitForPin(uint8_t pin){
	return pin%8;
}

/**
 * Register address, port dependent, for a given PIN
 */
uint8_t MCP23017::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return(pin<8) ?portAaddr:portBaddr;
}

/**
 * Reads a given register
 */
uint8_t MCP23017::readRegister(uint8_t addr)
{
#ifdef USE_SMBUS
    int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, addr);
#else // USE_SMBUS
    uint8_t i2C_address = kI2CAddress;
    int i2C_file = kI2CFileDescriptor;
    unsigned int reg = addr;

    i2c_char_t inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /* Reading a register involves a repeated start condition which needs ioctl() */
    outbuf = reg;
    messages[0].addr  = i2C_address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = i2C_address;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    int toReturn;
    if(ioctl(i2C_file, I2C_RDWR, &packets) < 0) {
        toReturn = -1;
    } else {
        toReturn = inbuf;
    }
#endif // USE_SMBUS
    if (toReturn < 0) {
        printf("MCP23017 Read Byte error: %d\n",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    // For debugging
    // printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", kI2CAddress, toReturn, readRegister);
    return toReturn ;
}

/**
 * Writes a given register
 */
uint8_t MCP23017::writeRegister(uint8_t addr, uint8_t writeValue)
{   // For debugging:
    // printf("Wrote: 0x%02X to register 0x%02X \n",writeValue, writeRegister) ;

#ifdef USE_SMBUS
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, addr, writeValue);
#else // USE_SMBUS
    unsigned int reg = addr;
    unsigned int value = writeValue;
    int i2C_file = kI2CFileDescriptor;
    char buf[2] = { static_cast<char>(reg & 0xFF), static_cast<char>(value & 0xFF) };
    errno = 0; // to avoid ambiguity in the error message in case write() returns 0
    int toReturn;
    if(write(i2C_file, buf, 2) != sizeof(buf))
        toReturn = -1;
    else
        toReturn = 0;
#endif // USE_SMBUS
    if (toReturn < 0) {
        perror("Write to I2C Device failed");
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}

/**
 * Reads a byte
 */
uint8_t MCP23017::readByte()
{
#ifdef USE_SMBUS
    int toReturn = i2c_smbus_read_byte(kI2CFileDescriptor);
#else // USE_SMBUS
    i2c_char_t byte;
    int toReturn = read(kI2CFileDescriptor, &byte, sizeof(byte));
    if(sizeof(byte) == toReturn)
        toReturn = byte;
#endif // USE_SMBUS
    if (toReturn < 0) {
        printf("MCP23017 Read Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    // For debugging
    // printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", kI2CAddress, toReturn, readRegister);
    return toReturn ;
}

/**
 * Writes a byte
 */
uint8_t MCP23017::writeByte(uint8_t writeValue)
{   // For debugging:
    // printf("Wrote: 0x%02X to register 0x%02X \n",writeValue, writeRegister) ;
#ifdef USE_SMBUS
    int toReturn = i2c_smbus_write_byte(kI2CFileDescriptor, writeValue);
#else // USE_SMBUS
    errno = 0; // to avoid ambiguity in the error message in case write() returns 0
    int toReturn = write(kI2CFileDescriptor, &writeValue, sizeof(writeValue));
    if(toReturn == sizeof(writeValue))
        toReturn = 0;
    else
        toReturn = -1;
#endif // USE_SMBUS
    if (toReturn < 0) {
        printf("MCP23017 Write Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}

/**
 * INTCAPA and INTCAPB capture the state of PORT A and PORT B at the moment an interrupt occured.
 * Reading this will also clear the corresponding interrupt.
 * Reads values from the specified register.
 */
uint8_t MCP23017::readINTCAP(uint8_t b){
	if(0 == b)
		return readRegister(MCP23017_INTCAPA);
	else
		return readRegister(MCP23017_INTCAPB);
}

/**
 * See readINTCAP(). This reads values from both registers.
 */
uint16_t MCP23017::readINTCAPAB(){
	uint16_t cap = readINTCAP(0);
	cap |=  readINTCAP(1) << 8;
	return cap;
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void MCP23017::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr=regForPin(pin,portAaddr,portBaddr);
	uint8_t bit=bitForPin(pin);
	regValue = readRegister(regAddr);

	// set the value for the particular bit
	bitWrite(regValue,bit,pValue);

	writeRegister(regAddr,regValue);
}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void MCP23017::pinMode(uint8_t p, Direction d) {
	updateRegisterBit(p,(d==INPUT),MCP23017_IODIRA,MCP23017_IODIRB);
}

/*
 * Sets the device to "byte mode" (when `repeat` is true) or "sequential mode"
 * (when `repeat` is false). The latter is default. This features would allow
 * to use readByte() and writeByte() to repeatedly access the same address (sec
 * 3.2.1 of the datasheet). Unfortunately, it doesn't seem to behave as
 * expected.
 */
void MCP23017::setRepeatedRW(bool repeat){
	const unsigned int SEQOP_BIT = 5;
	// configure the port A
	uint8_t ioconfValue=readRegister(MCP23017_IOCONA);
	bitWrite(ioconfValue,SEQOP_BIT,repeat);
	writeRegister(MCP23017_IOCONA,ioconfValue);

	// Configure the port B
	ioconfValue=readRegister(MCP23017_IOCONB);
	bitWrite(ioconfValue,SEQOP_BIT,repeat);
	writeRegister(MCP23017_IOCONB,ioconfValue);
}
/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 */
uint16_t MCP23017::readGPIOAB() {
	uint16_t ba = 0;
	uint8_t a;

	// read the current GPIO output latches
	writeByte(MCP23017_GPIOA);

	a = readByte();
	ba = readByte();
	ba <<= 8;
	ba |= a;

	return ba;
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * Parameter b should be 0 for GPIOA, and 1 for GPIOB.
 */
uint8_t MCP23017::readGPIO(uint8_t b) {

	// read the current GPIO output latches
	if (b == 0)
		writeByte(MCP23017_GPIOA);
	else {
		writeByte(MCP23017_GPIOB);
	}


	uint8_t value = readByte();
	return value;
}

/**
 * Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void MCP23017::writeGPIOAB(uint16_t ba) {

	writeByte(MCP23017_GPIOA);
	writeByte(ba & 0xFF);
	writeByte(ba >> 8);

}

void MCP23017::digitalWrite(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	uint8_t bit=bitForPin(pin);


	// read the current GPIO output latches
	uint8_t regAddr=regForPin(pin,MCP23017_OLATA,MCP23017_OLATB);
	gpio = readRegister(regAddr);

	// set the pin and direction
	bitWrite(gpio,bit,d);

	// write the new GPIO
	regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	writeRegister(regAddr,gpio);
}

void MCP23017::pullUp(uint8_t p, Level d) {
	updateRegisterBit(p,d,MCP23017_GPPUA,MCP23017_GPPUB);
}

bool MCP23017::digitalRead(uint8_t pin) {
	uint8_t bit=bitForPin(pin);
	uint8_t regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	return (readRegister(regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false, false, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 */
void MCP23017::setupInterrupts(bool mirroring, bool openDrain, Level polarity){
	// configure the port A
	uint8_t ioconfValue=readRegister(MCP23017_IOCONA);
	bitWrite(ioconfValue,6,mirroring);
	bitWrite(ioconfValue,2,openDrain);
	bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONA,ioconfValue);

	// Configure the port B
	ioconfValue=readRegister(MCP23017_IOCONB);
	bitWrite(ioconfValue,6,mirroring);
	bitWrite(ioconfValue,2,openDrain);
	bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONB,ioconfValue);
}

/**
 * Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
 *
 */
void MCP23017::setupInterruptPin(uint8_t pin, InterruptMode mode) {

	// set the pin interrupt control (0 means change, 1 means compare against given value);
	updateRegisterBit(pin,(mode!=CHANGE),MCP23017_INTCONA,MCP23017_INTCONB);
	// if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

	// In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
	// In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
	updateRegisterBit(pin,(mode==FALLING),MCP23017_DEFVALA,MCP23017_DEFVALB);

	// enable the pin for interrupt
	updateRegisterBit(pin,HIGH,MCP23017_GPINTENA,MCP23017_GPINTENB);

}

uint8_t MCP23017::getLastInterruptPin(){
	uint8_t intf;

	// try port A
	intf=readRegister(MCP23017_INTFA);
	for(int i=0;i<8;i++) if (bitRead(intf,i)) return i;

	// try port B
	intf=readRegister(MCP23017_INTFB);
	for(int i=0;i<8;i++) if (bitRead(intf,i)) return i+8;

	return MCP23017_INT_ERR;

}
uint8_t MCP23017::getLastInterruptPinValue(){
	uint8_t intPin=getLastInterruptPin();
	if(intPin!=MCP23017_INT_ERR){
		uint8_t intcapreg=regForPin(intPin,MCP23017_INTCAPA,MCP23017_INTCAPB);
		uint8_t bit=bitForPin(intPin);
		return (readRegister(intcapreg)>>bit) & (0x01);
	}

	return MCP23017_INT_ERR;
}

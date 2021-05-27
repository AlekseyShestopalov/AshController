/*
ASH_link.h - Class for Arduino for communication with PC
Aleksey Shestopalov ash******ov(dog)yandex.ru
*/
#ifndef ASH_link_h 
#define ASH_link_h 

#include "Arduino.h"

#define ASH_AMX_INCBUFFER_SIZE	128

// status byte bits values
#define ASH_STS_ERROR	 (1<<7)    // error
#define ASH_STS_BUSY 	 (1<<6)    // busy
#define ASH_STS_ZPOS 	 (1<<5)    // position error 
#define ASH_STS_MOVE 	 (1<<4)    // in moving
#define ASH_STS_PLANFULL (1<<3)    // plan full
#define ASH_STS_READY		  1	   // ready to command

// commands
// commands 0x00-0x20 reserved for ASH_Link, 0x21-0x7F is available for main-app extentions
// commands 0x80-0xFF reserved for ansver with error message
#define ASH_CMD_GETCODE   	0x00    // get conrollers code
#define ASH_CMD_GETNAME   	0x01    // get controllers name
#define ASH_CMD_GETCONFIG 	0x02    // get configuration
#define ASH_CMD_GETSTATE	0x04    // get state
#define ASH_CMD_SET  		0x10    // Write new subsystem state
#define ASH_CMD_GET  		0x11   	// Load subsystems state
#define ASH_CMD_UD  		0x20    // users data, command for main-app protocols extention

// extention for commands ASH_CMD_GET | ASH_CMD_SET
#define ASH_CMDGETSET_REGS		0x01	// microcontrollers registers
#define ASH_CMDGETSET_EEPROM	0x02	// EEPROM value

// Arduino-PC communications object
class ASH_link {
 public:
	ASH_link(byte DN);  	// constructor
	int CheckIncomming();	// main application have to call this every loop
	void SetParser(
			int (*ParseAndRespSet) (unsigned char *inbuf, unsigned char *outbuf, unsigned char in, unsigned char on)
		);
	byte SetStateBit(unsigned char bit, unsigned char val);	// change one bit in state
	byte GetState();

	
  private:
	byte _DN;       // device number
	byte _sts;		// current device state
	byte buffer[ASH_AMX_INCBUFFER_SIZE];	// incomming buffer
	byte buflen;		// current buffer fullness
	uint32_t buftms;	// last incomming activity time (for timeout calculation)
	// additional user command parser
	int (*F_ParseAndRespSet) (unsigned char *inbuf, unsigned char *outbuf, unsigned char in, unsigned char on);
	
 private:
	int _OutPacket(byte cmd, byte *buf, size_t len);
	int _ParseIncommingPacket();
	int _ParseIncommingPacket_Get(byte cmd, byte in, byte *outbuf, byte on);
	int _ParseIncommingPacket_Set(byte cmd, byte in, byte *outbuf, byte on);
//	int _GetNextByte(unsigned char *b);
//	int _ReadIncommingPacket(unsigned char *buf, int len);
	
};	// ASH_link_h

#endif
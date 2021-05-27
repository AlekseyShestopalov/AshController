/*
ASH_link.h - Class for Arduino for communication with PC
Aleksey Shestopalov ash******ov(dog)yandex.ru
*/
#include "ASH_link.h"
#include "Arduino.h"
#include <EEPROM.h>

const char DevCode[16] = "0-0-0";
const char DevName[16] = "BaseController";
const char DevLibVer[16] = "1.0.4 May 2021";

//for debugging 
//LED on port 13 - flash count times 
int _flash13(int count, int delay_ms)
{
  for( int i=0; i<count; i++ )
  {	  
	  digitalWrite(13, 1);
	  delay(delay_ms);
	  digitalWrite(13, 0);
	  delay(delay_ms);
  }
}

// constructor
ASH_link::ASH_link(byte DN)
{
	this->_DN = DN;
	this->_sts = 0;
	this->buflen = 0;
	this->F_ParceAndRespSet = NULL;
}  

// check incomming symbols. 
// main application have to call this every loop
// result: 0 - nothing, 1 - message parsed 
int ASH_link::CheckIncomming()
{	
	unsigned char val;
	if( !Serial.available() ) return 0; // nothing to read

	if( this->buflen>1 && millis() - this->buftms > 200 ) // timeout
		this->buflen = 0;			// The current buffer contents should be reset before read
		
	this->buftms = millis();	
	do {
		this->buffer[this->buflen++] = Serial.read(); // load one byte from PC
	} while( Serial.available() && this->buflen<ASH_AMX_INCBUFFER_SIZE-1); 
	
	// checks protocols error
	if( this->buffer[0] != ':' || // wrong start symbol of message, should be ignored
		(this->buflen>1 && this->buffer[1]>ASH_AMX_INCBUFFER_SIZE-2))	// too big message size, should be ignored
	{
		this->buflen = 0;
		return 0;	// start again when next call happen
	}

	if( this->buflen < this->buffer[1]+2) // message not compleate
		return 0; 	// continue when next call happen

	// message compleate, start message parser
	this->_ParceIncommingPacket();	
	this->buflen = 0;
	return 1; // parced
}

int ASH_link::_ParceIncommingPacket_Get(byte cmd, byte in, byte outbuf, byte on)
{
	byte cid, pid;
	cid = this->buffer[n++];
	pid = this->buffer[n++];
	outbuf[on++] = cid; 
	outbuf[on++] = pid; 
	if( cid == ASH_CMDGETCID_REGS ) //0x01 	  // регистры микроконтроллера
	{
		if( pid == 0x01 ) // цифровые регистры микроконтроллера
		{
			outbuf[on++] = DDRD;  // The Port D Data Direction Register (регистр направления передачи данных порта D)
			outbuf[on++] = PORTD; // The Port D Data Register (регистр данных порта D)
			outbuf[on++] = DDRB;  // The Port B Data Direction Register (регистр направления передачи данных порта B)
			outbuf[on++] = PORTB; // The Port B Data Register (регистр данных порта B)
		}
		else if( pid == 0x02 ) // аналоговые регистры микроконтроллера
		{
			outbuf[on++] = DDRC; // The Port C Data Direction Register (регистр направления передачи данных порта C)
			outbuf[on++] = PORTC;// The Port C Data Register (регистр данных порта C)
		}
		else if( pid == 0x03 ) // отдельный аналоговый регистр микроконтроллера
		{
			b0 = this->buffer[n++]; // номер порта
			val = analogRead( A0+b0 );
			outbuf[on++] = b0;  
			outbuf[on++] = val & 0xFF;  
			outbuf[on++] = (val>>8) & 0xFF;  
		}
	}
	else if( cid == ASH_CMDGETCID_EEPROM )	// 0x02	- содержимое EEPROM
	{
		b0 = pid; 		// номер первой ячейки 
		b1 = 0; n++; //this->buffer[n++];  // номер первой ячейки
		val = (b1<<8)+b0;
		b0 = this->buffer[n++]; // количество ячеек
		b1 = this->buffer[n++]; // количество ячеек
		val2 =(b1<<8)+b0;
		
		outbuf[on++] = b0;
		outbuf[on++] = b1;
		for( int i=0; i<val2; i++ )
		{
			outbuf[on++] = EEPROM[val+i];				
		}
	}
	this->_OutPacket( cmd, outbuf, on);  // отправляем ответ устройству
	
	return 0;
}

int ASH_link::_ParceIncommingPacket_Get(byte cmd, byte in, byte outbuf, byte on)
{
	byte cid, pid;

	cid = this->buffer[n++];
	pid = this->buffer[n++];
	b0 = this->buffer[n++];
	b1 = this->buffer[n++];
	outbuf[on++] = cid; 
	outbuf[on++] = pid; 
	if( cid == 0x01 )		// регистры микроконтроллера
	{
	if( pid == 0x03 ) // цифровые регистры микроконтроллера, запись значения
	  digitalWrite( b0, b1);        
	else if( pid == 0x04 ) // цифровые регистры микроконтроллера, запись направления
	  pinMode( b0, b1==0?INPUT:OUTPUT);        
	}
	this->_OutPacket( cmd, outbuf, on);  // отправляем ответ устройству
}

// packet parser
int ASH_link::_ParceIncommingPacket()
{
  byte cnt, dn, cmd, n=1, on=0;
  unsigned short b0, b1, b2;
  byte outbuf[128];
  unsigned short val, val2;
  cnt = this->buffer[n++];
  dn  = this->buffer[n++];
//_flash13(1, 800);
  if( dn != this->_DN ) // wrong device number, message should be ignored
  {
//	_flash13(2, 400);
	return -1;
  }

  cmd = this->buffer[n++];
  switch(cmd)      
  {
    case ASH_CMD_GETCODE: 	//0x00
	//flash13(2, 200);
		this->_OutPacket( cmd, (const unsigned char*)DevCode, strlen(DevCode));  // reply
    break;
    case ASH_CMD_GETNAME: 	//0x01
	//_flash13(4, 200);
		this->_OutPacket( cmd, (const unsigned char*)DevName, strlen(DevName));  // reply
    break;
    case ASH_CMD_GETCONFIG: //0x02
		this->_OutPacket( cmd, (const unsigned char*)DevLibVer, strlen(DevLibVer));  // reply
    break;
    case ASH_CMD_GET: // 0x11 - чтение данных и состояния
		this->_ParceIncommingPacket_Get();
    break;
    case ASH_CMD_SET: // 0x10 - запись данных и состояния
    break;
    case ASH_CMD_GETSTATE: //0x04  -  Запрос байта статуса устройства.
      this->_OutPacket( cmd, outbuf, on);  // отправляем ответ устройству
    break;
	default:	// unknown command
	case ASH_CMD_UD: // передаем устройству на дальнейший разбор самостоятельно
//		_flash13(1, 200);
		if( this->F_ParseAndRespSet )
			on = (*this->F_ParseAndRespSet) (this->buffer+n-1, outbuf, 0, on);
		this->_OutPacket( cmd, outbuf, on);  // отправляем ответ устройству
	break;
//	default:	// unknown command
//     this->_OutPacket( cmd, outbuf, on);  // отправляем ответ устройству
//	return -1;
  }
  return 0;
}

// reply len bytes to PC
// wraps bytes with the prefix and checksum required by the protocol
int ASH_link::_OutPacket(const unsigned char cmd, const unsigned char *buf, size_t len)
{
  byte outbuf[256];
  byte checksumm=0;
  int n=0;
  outbuf[n++] = '+';
  outbuf[n++] = len+4; //full message len: userlen + dn, cmd, _sts, cs
  outbuf[n++] = this->_DN;
  outbuf[n++] = cmd;
  outbuf[n++] = this->_sts; 
  checksumm += outbuf[1];
  checksumm += outbuf[2];
  checksumm += outbuf[3];
  checksumm += outbuf[4];
  for(int i=0; i<len; i++)
  {
      outbuf[n++] = buf[i];
	  checksumm += buf[i];  
  }
  outbuf[n++] = (byte)(256-checksumm);
  Serial.write( outbuf, n ); //+, len ... cs
//_flash13(2, 200);
  return 0; // пакет успешно отправлен
}

// set one or more bits in the state byte
// usage: SetStateBit( ASH_STS_MOVE, 1 ) - sets the bit "move"
// returns new value of _sts
byte ASH_link::SetStateBit(unsigned char bit, unsigned char val)
{
	if(val) // установить
		this->_sts |= bit;	
	else	// сбросить
		this->_sts &= ~bit;	
		
	return this->_sts;
}

// getter for _sts
byte ASH_link::GetState()
{
	return _sts;	
}

void ASH_link::SetParser(
		int (*ParseAndRespSet) (unsigned char *inbuf, unsigned char *outbuf, unsigned char in, unsigned char on)
	)
{
	this->F_ParseAndRespSet = ParseAndRespSet;
}
 
 // next byte reads with timeount
/*
int ASH_link::_GetNextByte(unsigned char *b)
{
    int timing = millis();
    while( !Serial.available() ) // ждем символ, но не более 500 ms
        if( millis() - timing > 500 ) 
		{
			//_flash13(1, 400);
			return -1; // таймаут      
		}

    *b = Serial.read ();  
    return 0;  // символ получен
}
*/

// вычитывание входящего пакета по протоколу
/*
int ASH_link::_ReadIncommingPacket(unsigned char *buf, int len)
{
  // вычитываем длину пакета
  if( 0 != this->_GetNextByte(&buf[1]) ) // символ не вычитан
    return -1;
  if( buf[1] > (len-2) ) return -2; // слишком длинный пакет
  
  // дочитываем сам пакет
  for( int i=2; i<=buf[1]; i++ )
    if( 0 != this->_GetNextByte(&buf[i]) ) // символ не вычитан
      return -1;
      
  return 0; // success
}
*/

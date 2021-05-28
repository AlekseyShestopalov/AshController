# AshController  (Arduino Simple Host Controller)
Class for Arduino-PC communication.
==============================================================================
Realise a simple communication bytes protocol throw RS234, RS485 or USB converters to RS232

PC->Arduino request:
<RP><CNT><DN><CMD>[<CMD_DATA>]<CS>
<RP> - prefix, everywhere ":"
<CNT> - command length - bytes count, started from <DN> to <CS>
<DN>  - device number - unic number for device on the same physical line
<CMD> - command code
  .... command data
<CS> - checsumm
  
PC->Arduino response:
<AP><CNT><DN><CMD><STS>[<CMD_DATA>]<CS>
<RP> - prefix, everywhere "+"
<CNT> - command length - bytes count, started from <DN> to <CS>
<DN>  - device number - unic number for device on the same physical line
<CMD> - command code
<STS> - device status
  .... command data
<CS> - checsumm
  

==============================================================================
 
usage 1: this code is sufficient to control the digital and analog pins of the Arduino from the PC

==============================================================================
#include <ASH_link.h>

#define DN 22

ASH_link *al;

void setup()
{
  al = new ASH_link( DN );
}

void loop()
{
  al->CheckIncomming();  
}

==============================================================================

usage 2: example of controlling a servomotor throw AshController from PC 

==============================================================================
#include <ASH_link.h>
#include <Servo.h>

#define DN 22
#define SERVO_PIN 3

ASH_link *al;
Servo srv;

void setup()
{
  Serial.begin(115200);
  al = new ASH_link( DN );
  srv.attach(SERVO_PIN);
}

void loop()
{
  al->CheckIncomming();  
  al->SetParser( MyParser ); // additional command parcer
}

int MyParser(unsigned char *buf, unsigned char *outbuf, unsigned char in, unsigned char on)
{
   int n=0;
   unsigned char cmd = buf[n++];
   
   switch( cmd )
   {
      case 0x21:  // servo position 
        byte pos = buf[n++];
        PenServo.write( pos );
      break;
   }
}

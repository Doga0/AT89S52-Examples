#include "REG51.h"   

sbit LED = P2^1;   // define PORT 2 pin 1

void Delay(int timer);

void main(void) {

	while(1)          
	{
		LED = 0; 
		Delay(4000);

		LED = 1; 
		Delay(4000);
	}
}

void Delay(int timer)
{
	 
 for(int i=0;i<timer;i++); // delay loop

}
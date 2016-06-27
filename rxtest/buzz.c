#include <machine.h>
#include "iodefine.h"
#define buzz PORTB.DR.BIT.B7
#define ON 1
#define OFF 0 
//#define BUZZ_PERMIT 1

void BUZZ_ON()
{	
#ifdef BUZZ_PERMIT
	short i=0;
	short buzz_hz=500;

	for(i=0; i<25; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
		if(i%2==0)
			buzz_hz/=2;
		else{
			buzz_hz*=2;
		}
	}
#endif
}

void BUZZ_ON2()
{	
#ifdef BUZZ_PERMIT
	short i=0;
	short buzz_hz=500;

	for(i=0; i<10; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
		if(i%2==0)
			buzz_hz/=2;
		else{
			buzz_hz*=2;
		}
	}
#endif
}

void BUZZ_ON3()
{	
#ifdef BUZZ_PERMIT
	short i=0;
	short buzz_hz=150;

	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	buzz_hz=250;
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	buzz_hz=200;
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	buzz_hz=150;
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
#endif	
}

void BUZZ_ON4()
{	
#ifdef BUZZ_PERMIT
	short i=0;
	short buzz_hz=100;

	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	TIMER_WAIT(50);
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
#endif
}

void BUZZ_ON5()
{	
	buzz=~buzz;
}

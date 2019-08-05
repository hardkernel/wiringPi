#ifndef	__WIRING_PRIVATE_H__
#define	__WIRING_PRIVATE_H__

#include "wiringPi.h"

void setupCheck (const char *fName);
void usingGpioMemCheck (const char *what);
void setUsingGpioMem( const unsigned int value );

#endif	/* __WIRING_PRIVATE_H__ */
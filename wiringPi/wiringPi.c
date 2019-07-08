/*----------------------------------------------------------------------------*/
/*

	WiringPi Library for ODROIDs

 */
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

/*----------------------------------------------------------------------------*/
#include "softPwm.h"
#include "softTone.h"

/*----------------------------------------------------------------------------*/
#include "wiringPi.h"
#include "../version.h"

#include "odroidc1.h"
#include "odroidc2.h"
#include "odroidxu3.h"
#include "odroidn1.h"
#include "odroidn2.h"

/*----------------------------------------------------------------------------*/
// Const string define
/*----------------------------------------------------------------------------*/
const char *piModelNames [16] =
{
	"Unknown",
	"ODROID-C1/C1+",
	"ODROID-C2",
	"ODROID-XU3/4",
	"ODROID-N1",
	"ODROID-N2",
};

const char *piRevisionNames [16] =
{
	"00",
	"01",
	"02",
	"03",
	"04",
	"05",
	"06",
	"07",
	"08",
	"09",
	"10",
	"11",
	"12",
	"13",
	"14",
	"15",
} ;

const char *piMakerNames [16] =
{
	"Unknown",	// 0
	"AMLogic",	// 1
	"SAMSUNG",	// 2
	"ROCKCHIP",	// 3
	"INTEL",	// 4
	"AMD",		// 5
	"Unknown06",	// 6
	"Unknown07",	// 7
	"Unknown08",	// 8
	"Unknown09",	// 9
	"Unknown10",	// 10
	"Unknown11",	// 11
	"Unknown12",	// 12
	"Unknown13",	// 13
	"Unknown14",	// 14
	"Unknown15",	// 15
} ;

const int piMemorySize [8] =
{
	256,		//	 0
	512,		//	 1
	1024,		//	 2
	2048,		//	 3
	4096,		//	 4
	8192,		//	 5
	0,		//	 6
	0,		//	 7
} ;

/*----------------------------------------------------------------------------*/
// Misc
static volatile int    pinPass = -1 ;
static pthread_mutex_t pinMutex ;

// Debugging & Return codes
int wiringPiDebug       = FALSE ;
int wiringPiReturnCodes = FALSE ;

// ODROID Wiring Library
struct libodroid	libwiring;

/*----------------------------------------------------------------------------*/
//
// Return true/false if the supplied module is loaded
//
/*----------------------------------------------------------------------------*/
int moduleLoaded (char *modName)
{
	int len   = strlen (modName) ;
	int found = FALSE ;
	FILE *fd = fopen ("/proc/modules", "r") ;
	char line [80] ;

	if (fd == NULL) {
		fprintf (stderr, "gpio: Unable to check /proc/modules: %s\n",
			strerror (errno)) ;
		exit (1) ;
	}

	while (fgets (line, 80, fd) != NULL) {
		if (strncmp (line, modName, len) != 0)
			continue ;

		found = TRUE ;
		break ;
	}
	fclose (fd) ;

	return found ;
}

/*----------------------------------------------------------------------------*/
//
// ODROID System Message function
//
/*----------------------------------------------------------------------------*/
int msg (int type, const char *message, ...)
{
	va_list argp;
	char buffer [1024];

	va_start (argp, message) ;
	vsnprintf (buffer, 1023, message, argp);
	va_end (argp) ;

	fprintf (stderr, "%s : %s", type == MSG_WARN ? "warn" : "err", buffer) ;

	if (type != MSG_WARN)
		exit (EXIT_FAILURE) ;
	return 0 ;
}

/*----------------------------------------------------------------------------*/
static void warn_msg(const char *func)
{
	msg(MSG_WARN, "(%s) : This function is not supported by ODROID Board.\n", func);
}
/*----------------------------------------------------------------------------*/
//
// Unsupport Function list on ODROIDs
//
/*----------------------------------------------------------------------------*/
static 	void UNU piGpioLayoutOops	(const char UNU *why)	{ warn_msg(__func__); return; }
	void pwmSetMode		(int UNU mode)		{ warn_msg(__func__); return; }
	void pwmSetRange	(unsigned int UNU range)	{ warn_msg(__func__); return; }
	void pwmSetClock	(int UNU divisor)		{ warn_msg(__func__); return; }
	void gpioClockSet	(int UNU pin, int UNU freq)	{ warn_msg(__func__); return; }

	/* core unsupport function */
	void pinModeAlt		(int UNU pin, int UNU mode)	{ warn_msg(__func__); return; }
	void pwmWrite		(int UNU pin, int UNU value)	{ warn_msg(__func__); return; }
	void analogWrite	(int UNU pin, int UNU value)	{ warn_msg(__func__); return; }
	void pwmToneWrite	(int UNU pin, int UNU freq)	{ warn_msg(__func__); return; }
	void digitalWriteByte2	(const int UNU value)	{ warn_msg(__func__); return; }
	unsigned int digitalReadByte2 (void)		{ warn_msg(__func__); return -1; }
/*----------------------------------------------------------------------------*/
// Extend wiringPi with other pin-based devices and keep track of
//	them in this structure
/*----------------------------------------------------------------------------*/
struct wiringPiNodeStruct *wiringPiNodes = NULL ;

struct wiringPiNodeStruct *wiringPiFindNode (int UNU pin) {	return NULL; }

static		void pinModeDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int mode)  { return ; }
static		void pullUpDnControlDummy	(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int pud)   { return ; }
static	unsigned int UNU digitalRead8Dummy		(UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return 0 ; }
static		void UNU digitalWrite8Dummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static		int  digitalReadDummy		(UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return LOW ; }
static		void digitalWriteDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static		void pwmWriteDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static		int  analogReadDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin)            { return 0 ; }
static		void analogWriteDummy		(UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }

struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins)
{
	int	pin ;
	struct wiringPiNodeStruct *node ;

	// Minimum pin base is 64
	if (pinBase < 64)
		(void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

	// Check all pins in-case there is overlap:
	for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
		if (wiringPiFindNode (pin) != NULL)
			(void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

	node = (struct wiringPiNodeStruct *)calloc (sizeof (struct wiringPiNodeStruct), 1) ;	// calloc zeros
	if (node == NULL)
		(void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

	node->pinBase		= pinBase ;
	node->pinMax		= pinBase + numPins - 1 ;
	node->pinMode		= pinModeDummy ;
	node->pullUpDnControl	= pullUpDnControlDummy ;
	node->digitalRead	= digitalReadDummy ;
	//node->digitalRead8	= digitalRead8Dummy ;
	node->digitalWrite	= digitalWriteDummy ;
	//node->digitalWrite8	= digitalWrite8Dummy ;
	node->pwmWrite		= pwmWriteDummy ;
	node->analogRead	= analogReadDummy ;
	node->analogWrite	= analogWriteDummy ;
	node->next		= wiringPiNodes ;
	wiringPiNodes		= node ;

	return node ;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*
 * wiringPiFailure:
 *	Fail. Or not.
 */
/*----------------------------------------------------------------------------*/
int wiringPiFailure (int fatal, const char *message, ...)
{
	va_list argp ;
	char buffer [1024] ;

	if (!fatal && wiringPiReturnCodes)
		return -1 ;

	va_start (argp, message) ;
	vsnprintf (buffer, 1023, message, argp) ;
	va_end (argp) ;

	fprintf (stderr, "%s", buffer);
	exit (EXIT_FAILURE);

	return 0 ;
}

/*----------------------------------------------------------------------------*/
int piGpioLayout (void)
{
	FILE *cpuFd ;
	char line [120] ;
	char *c ;
	static int  gpioLayout = -1 ;

	if (gpioLayout != -1)
		return	gpioLayout;

	gpioLayout = 1;

	if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
		wiringPiFailure (WPI_FATAL, "Unable to open /proc/cpuinfo") ;

	while (fgets (line, 120, cpuFd) != NULL)
		if (strncmp (line, "Hardware", 8) == 0)
			break ;

	if (strncmp (line, "Hardware", 8) != 0)
		wiringPiFailure (WPI_FATAL, "No \"Hardware\" line") ;

	if (wiringPiDebug)
		printf ("piGpioLayout: Hardware: %s\n", line) ;

	if (!(strstr (line, "ODROID")))
		wiringPiFailure (WPI_FATAL, "** This board is not ODROID. **") ;

	rewind (cpuFd) ;
	while (fgets (line, 120, cpuFd) != NULL)
		if (strncmp (line, "Revision", 8) == 0)
			break ;
	fclose (cpuFd) ;

	if (strncmp (line, "Revision", 8) != 0)
		wiringPiFailure (WPI_FATAL, "No \"Revision\" line") ;

	for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
		*c = 0 ;

	if (wiringPiDebug)
		printf ("piGpioLayout: Revision string: %s\n", line) ;

	// Scan to the first character of the revision number
	for (c = line ; *c ; ++c)
		if (*c == ':')
			break ;

	if (*c != ':')
		wiringPiFailure (WPI_FATAL, "Bogus \"Revision\" line (no colon)") ;

	// Chomp spaces
	++c ;
	while (isspace (*c))
		++c ;

	if (!isxdigit (*c))
		wiringPiFailure (WPI_FATAL, "Bogus \"Revision\" line (no hex digit at start of revision)") ;

	// Make sure its long enough
	if (strlen (c) < 4)
		wiringPiFailure (WPI_FATAL, "Bogus revision line (too small)") ;

	// Isolate  last 4 characters: (in-case of overvolting or new encoding scheme)
	c = c + strlen (c) - 4 ;

	if (wiringPiDebug)
		printf ("piGpioLayout: last4Chars are: \"%s\"\n", c) ;

	if ((strcmp (c, "0002") == 0) || (strcmp (c, "0003") == 0) ||
	    (strcmp (c, "000a") == 0) || (strcmp (c, "0100") == 0) ||
	    (strcmp (c, "0400") == 0) )
		gpioLayout = 1;
	else
		gpioLayout = 2;

	if (strcmp (c, "000a") == 0) {
		libwiring.model	= MODEL_ODROID_C1;
		libwiring.maker	= MAKER_AMLOGIC;
		libwiring.mem	= 2;
		libwiring.rev	= 1;
	} else if (strcmp (c, "0100") == 0) {
		libwiring.model	= MODEL_ODROID_XU3;
		libwiring.maker	= MAKER_SAMSUNG;
		libwiring.mem	= 3;
		libwiring.rev	= 1;
	} else if (strncmp (c, "02", 2) == 0) {
		libwiring.model	= MODEL_ODROID_C2;
		libwiring.maker	= MAKER_AMLOGIC;
		libwiring.mem	= 3;
		{
			int fd = 0;
			char buf[2];

			if ((fd = open ("/sys/class/odroid/boardrev", O_RDONLY)) < 0) {
				printf ("ERROR : file not found.(boardrev)\n");
				libwiring.rev = 1;
			} else {
				(void)read (fd, buf, sizeof(buf));
				close(fd);
				libwiring.rev = atoi(buf) + 1;
			}
		}
	} else if (strncmp (c, "03", 2) == 0) {
		libwiring.model	= MODEL_ODROID_N1;
		libwiring.maker	= MAKER_ROCKCHIP;
		libwiring.mem	= 4;
		libwiring.rev	= 1;
	} else if (strncmp (c, "04", 2) == 0) {
		libwiring.model	= MODEL_ODROID_N2;
		libwiring.maker	= MAKER_AMLOGIC;
		libwiring.mem	= 4;
		libwiring.rev	= 1;
	} else {
		libwiring.model	= MODEL_UNKNOWN;
		libwiring.maker	= MAKER_UNKNOWN;
		libwiring.mem	= 0;
		libwiring.rev	= 0;
	}

	if (wiringPiDebug)
		printf ("BoardRev: Returning revision: %d\n", libwiring.rev) ;

	return libwiring.rev;
}

/*----------------------------------------------------------------------------*/
int piBoardRev (void)
{
	return	piGpioLayout ();
}

/*----------------------------------------------------------------------------*/
/*
 * piBoardId:
 *	Return the real details of the board we have.
 *
 *  000a - Model ODROID C0/C1/C1+, Rev 1.0, 1024M, Hardkernel
 *  added :
 *  0100 - Model ODROID XU3/4, Rev 1.0, 2048M, Hardkernel
 *  added :
 *  02xx - Model ODROID C2, 2048M, Hardkernel
 *         Rev 1.0 : /sys/class/odroid/boardrev value is 0 (Dev board)
 *         Rev 1.1 : /sys/class/odroid/boardrev value is 1 (Mass board)
 *  03xx - Model ODROID N1, 4096M, Hardkernel
 *  04xx - Model ODROID N2, 4096M, Hardkernel
 */
/*----------------------------------------------------------------------------*/
void piBoardId (int *model, int *rev, int *mem, int *maker, int *warranty)
{
	// Call this first to make sure all's OK. Don't care about the result.
	(void)piGpioLayout () ;

	*model	= libwiring.model;
	*maker	= libwiring.maker;
	*rev	= libwiring.rev;
	*mem	= libwiring.mem;
	*warranty = 1;
}

/*----------------------------------------------------------------------------*/
/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 */
/*----------------------------------------------------------------------------*/
int wpiPinToGpio (int wpiPin)
{
	if (libwiring.getModeToGpio)
		return	libwiring.getModeToGpio(MODE_PINS, wpiPin);

	return	-1;
}

/*----------------------------------------------------------------------------*/
/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 */
/*----------------------------------------------------------------------------*/
int physPinToGpio (int physPin)
{
	if (libwiring.getModeToGpio)
		return	libwiring.getModeToGpio(MODE_PHYS, physPin);

	return	-1;
}

/*----------------------------------------------------------------------------*/
/*
 * setPadDrive:
 *	Set the PAD driver value
 */
/*----------------------------------------------------------------------------*/
void setPadDrive (int pin, int value)
{
	if (libwiring.setPadDrive)
		return	libwiring.setPadDrive(pin, value);
}

/*----------------------------------------------------------------------------*/
/*
 * getPadDrive:
 *	Get the PAD driver value
 */
/*----------------------------------------------------------------------------*/
int getPadDrive (int pin)
{
	if (libwiring.getPadDrive)
		return	libwiring.getPadDrive(pin);

	return	-1;
}

/*----------------------------------------------------------------------------*/
/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 */
/*----------------------------------------------------------------------------*/
int getAlt (int pin)
{
	if (libwiring.getAlt)
		return	libwiring.getAlt(pin);

	return	-1;
}

/*----------------------------------------------------------------------------*/
/*
 * getPUPD:
 *	Returns the PU/PD bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 */
/*----------------------------------------------------------------------------*/
int getPUPD (int pin)
{
	if (libwiring.getPUPD)
		return	libwiring.getPUPD(pin);

	return	-1;
}

/*----------------------------------------------------------------------------*/
/*
 * Core Functions
 */
/*----------------------------------------------------------------------------*/
void pinMode (int pin, int mode)
{
	if (libwiring.pinMode)
		return	libwiring.pinMode(pin, mode);

}

/*----------------------------------------------------------------------------*/
void pullUpDnControl (int pin, int pud)
{
	if (libwiring.pullUpDnControl)
		return	libwiring.pullUpDnControl(pin, pud);
}

/*----------------------------------------------------------------------------*/
int digitalRead (int pin)
{
	if (libwiring.digitalRead)
		return	libwiring.digitalRead(pin);

	return	-1;
}

/*----------------------------------------------------------------------------*/
void digitalWrite (int pin, int value)
{
	if (libwiring.digitalWrite)
		return	libwiring.digitalWrite(pin, value);
}

/*----------------------------------------------------------------------------*/
int analogRead (int pin)
{
	if (libwiring.analogRead)
		return	libwiring.analogRead(pin);

	return	-1;
}

/*----------------------------------------------------------------------------*/
void digitalWriteByte (const int value)
{
	if (libwiring.digitalWriteByte)
		return	libwiring.digitalWriteByte(value);
}

/*----------------------------------------------------------------------------*/
unsigned int digitalReadByte (void)
{
	if (libwiring.digitalReadByte)
		return	libwiring.digitalReadByte();

	return	-1;
}

/*----------------------------------------------------------------------------*/
int waitForInterrupt (int pin, int mS)
{
	int fd, x;
	uint8_t c;
	struct pollfd polls;

	if ((fd = libwiring.sysFds[PIN_NUM_CALC_SYSFD(pin)]) ==  -1)
		return	-2;

	// Setup poll structure
	polls.fd     = fd ;
	polls.events = POLLPRI | POLLERR ;

	// Wait for it ...
	x = poll (&polls, 1, mS) ;

	// If no error, do a dummy read to clear the interrupt
	//	A one character read appars to be enough.
	if (x > 0) {
		lseek (fd, 0, SEEK_SET) ;	// Rewind
		(void)read (fd, &c, 1) ;	// Read & clear
	}
	return x ;
}

/*----------------------------------------------------------------------------*/
static void *interruptHandler (UNU void *arg)
{
	int myPin ;

	(void)piHiPri (55) ;	// Only effective if we run as root

	myPin   = pinPass ;
	pinPass = -1 ;

	for (;;)
		if (waitForInterrupt (myPin, -1) > 0)
			libwiring.isrFunctions [PIN_NUM_CALC_SYSFD(myPin)] () ;

	return NULL ;
}

/*----------------------------------------------------------------------------*/
int wiringPiISR (int pin, int mode, void (*function)(void))
{
	pthread_t threadId;
	const char *modeS;
	char fName   [64];
	char  pinS [8];
	pid_t pid;
	int   count, i;
	char  c;
	int   GpioPin;

	if (libwiring.mode == MODE_UNINITIALISED)
		return wiringPiFailure (
			WPI_FATAL,
			"wiringPiISR: wiringPi has not been initialised. " \
			"Unable to continue.\n") ;

	if (libwiring.getModeToGpio)
		GpioPin = libwiring.getModeToGpio(libwiring.mode, pin);
	else
		return wiringPiFailure (
			WPI_FATAL,
			"%s: getModeToGpio function not initialize!\n",
			__func__);

	// Now export the pin and set the right edge
	// We're going to use the gpio program to do this, so it assumes
	// a full installation of wiringPi. It's a bit 'clunky', but it
	// is a way that will work when we're running in "Sys" mode, as
	// a non-root user. (without sudo)
	if (mode != INT_EDGE_SETUP)
	{
		if (mode == INT_EDGE_FALLING)
			modeS = "falling" ;
		else if (mode == INT_EDGE_RISING)
			modeS = "rising" ;
		else
			modeS = "both" ;

		sprintf (pinS, "%d", GpioPin) ;

		if ((pid = fork ()) < 0)	// Fail
			return wiringPiFailure (
				WPI_FATAL,
				"wiringPiISR: fork failed: %s\n",
				strerror (errno));

		// Child, exec
		if (pid == 0) {
			if (access ("/usr/local/bin/gpio", X_OK) == 0) {
				execl ("/usr/local/bin/gpio", "gpio", "edge",
					pinS, modeS, (char *)NULL) ;
				return wiringPiFailure (
					WPI_FATAL,
					"wiringPiISR: execl failed: %s\n",
					strerror (errno));
			} else if (access ("/usr/bin/gpio", X_OK) == 0) {
				execl ("/usr/bin/gpio", "gpio", "edge",
					pinS, modeS, (char *)NULL) ;
				return wiringPiFailure (
					WPI_FATAL,
					"wiringPiISR: execl failed: %s\n",
					strerror (errno));
			} else
				return wiringPiFailure (
					WPI_FATAL,
					"wiringPiISR: Can't find gpio program\n");
		}
		else	// Parent, wait
			wait (NULL) ;
	}

	// Now pre-open the /sys/class node - but it may already be open if
	//	we are in Sys mode...

	if (libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)] == -1) {
		sprintf (fName, "/sys/class/gpio/gpio%d/value", GpioPin) ;

		if ((libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)] = open (fName, O_RDWR)) < 0)
			return wiringPiFailure (
				WPI_FATAL,
				"wiringPiISR: unable to open %s: %s\n",
				fName, strerror (errno)) ;
	}

	// Clear any initial pending interrupt
	ioctl (libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)], FIONREAD, &count) ;
	for (i = 0 ; i < count ; ++i)
		(void)read (libwiring.sysFds [PIN_NUM_CALC_SYSFD(GpioPin)], &c, 1) ;

	libwiring.isrFunctions [PIN_NUM_CALC_SYSFD(GpioPin)] = function ;

	pthread_mutex_lock (&pinMutex) ;
	pinPass = GpioPin ;
	pthread_create (&threadId, NULL, interruptHandler, NULL) ;
	while (pinPass != -1)
		delay (1) ;
	pthread_mutex_unlock (&pinMutex) ;

	return 0 ;
}

/*----------------------------------------------------------------------------*/
static void initialiseEpoch (void)
{
#ifdef	OLD_WAY
	struct timeval tv;

	gettimeofday (&tv, NULL) ;
	libwiring.epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    +
				(uint64_t)(tv.tv_usec / 1000) ;
	libwiring.epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 +
				(uint64_t)(tv.tv_usec) ;
#else
	struct timespec ts;

	clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
	libwiring.epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    +
				(uint64_t)(ts.tv_nsec / 1000000L) ;
	libwiring.epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 +
				(uint64_t)(ts.tv_nsec /    1000L) ;
#endif
}

/*----------------------------------------------------------------------------*/
void delay (unsigned int howLong)
{
	struct timespec sleeper, dummy;

	sleeper.tv_sec  = (time_t)(howLong / 1000) ;
	sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

	nanosleep (&sleeper, &dummy) ;
}

/*----------------------------------------------------------------------------*/
void delayMicrosecondsHard (unsigned int howLong)
{
	struct timeval tNow, tLong, tEnd;

	gettimeofday (&tNow, NULL) ;
	tLong.tv_sec  = howLong / 1000000 ;
	tLong.tv_usec = howLong % 1000000 ;
	timeradd (&tNow, &tLong, &tEnd) ;

	while (timercmp (&tNow, &tEnd, <))
		gettimeofday (&tNow, NULL) ;
}

/*----------------------------------------------------------------------------*/
void delayMicroseconds (unsigned int howLong)
{
	struct timespec sleeper;
	unsigned int uSecs = howLong % 1000000;
	unsigned int wSecs = howLong / 1000000;

	if (howLong == 0)
		return ;
	else if (howLong < 100)
		delayMicrosecondsHard (howLong);
	else {
		sleeper.tv_sec  = wSecs;
		sleeper.tv_nsec = (long)(uSecs * 1000L);
		nanosleep (&sleeper, NULL);
	}
}

/*----------------------------------------------------------------------------*/
unsigned int millis (void)
{
	uint64_t now;

#ifdef	OLD_WAY
	struct timeval tv;

	gettimeofday (&tv, NULL);
	now = (uint64_t)tv.tv_sec * (uint64_t)1000 +
		(uint64_t)(tv.tv_usec / 1000);
#else
	struct  timespec ts;

	clock_gettime (CLOCK_MONOTONIC_RAW, &ts);
	now = (uint64_t)ts.tv_sec * (uint64_t)1000 +
		(uint64_t)(ts.tv_nsec / 1000000L);
#endif
	return (uint32_t)(now - libwiring.epochMilli);
}

/*----------------------------------------------------------------------------*/
unsigned int micros (void)
{
	uint64_t now;
#ifdef	OLD_WAY
	struct timeval tv;

	gettimeofday (&tv, NULL);
	now = (uint64_t)tv.tv_sec * (uint64_t)1000000 +
		(uint64_t)tv.tv_usec;
#else
	struct timespec ts;

	clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
	now = (uint64_t)ts.tv_sec * (uint64_t)1000000 +
		(uint64_t)(ts.tv_nsec / 1000);
#endif
	return (uint32_t)(now - libwiring.epochMicro);
}

/*----------------------------------------------------------------------------*/
void wiringPiVersion (int *major, int *minor)
{
	*major = VERSION_MAJOR ;
	*minor = VERSION_MINOR ;
}

/*----------------------------------------------------------------------------*/
int wiringPiSetup (void)
{
	int i;
	static int alreadyDoneThis = FALSE;

	if (alreadyDoneThis)
		return 0;

	alreadyDoneThis = TRUE;

	// libwiring init
	memset(&libwiring, 0x00, sizeof(struct libodroid));
	// sysFds init
	for(i = 0; i < 256; i++)
		libwiring.sysFds[i] = -1;
	// init wiringPi mode
	libwiring.mode = MODE_UNINITIALISED;

	if (getenv (ENV_DEBUG) != NULL)
		wiringPiDebug = TRUE;

	if (getenv (ENV_CODES) != NULL)
		wiringPiReturnCodes = TRUE;

	(void)piGpioLayout();

	if (wiringPiDebug) {
		printf ("wiringPi: wiringPiSetup called\n") ;
		printf ("Model Name  : %s\n", piModelNames[libwiring.model]);
		printf ("Model Maker : %s\n", piMakerNames[libwiring.maker]);
		printf ("Model MEM   : %d\n", libwiring.mem);
		printf ("Model REV   : %d\n", libwiring.rev);
	}

	switch (libwiring.model) {
	case MODEL_ODROID_C1:
		init_odroidc1(&libwiring);
	break;
	case MODEL_ODROID_C2:
		init_odroidc2(&libwiring);
	break;
	case MODEL_ODROID_XU3:
		init_odroidxu3(&libwiring);
	break;
	case MODEL_ODROID_N1:
		init_odroidn1(&libwiring);
	break;
	case MODEL_ODROID_N2:
		init_odroidn2(&libwiring);
	break;
	default:
		return wiringPiFailure (WPI_ALMOST,
			"wiringPiSetup: Unknown model\n");
	}

	initialiseEpoch ();

	libwiring.mode = MODE_PINS;
	return 0;
}

/*----------------------------------------------------------------------------*/
/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 */
/*----------------------------------------------------------------------------*/
int wiringPiSetupGpio (void)
{
	(void)wiringPiSetup ();

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupGpio called\n") ;

	libwiring.mode = MODE_GPIO;
	return 0 ;
}

/*----------------------------------------------------------------------------*/
/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 */
/*----------------------------------------------------------------------------*/
int wiringPiSetupPhys (void)
{
	(void)wiringPiSetup () ;

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupPhys called\n") ;

	libwiring.mode = MODE_PHYS ;
	return 0 ;
}

/*----------------------------------------------------------------------------*/
/*
 * wiringPiSetupSys:
 *	Must be called once at the start of your program execution.
 *
 * Initialisation (again), however this time we are using the /sys/class/gpio
 *	interface to the GPIO systems - slightly slower, but always usable as
 *	a non-root user, assuming the devices are already exported and setup correctly.
 */
/*----------------------------------------------------------------------------*/
int wiringPiSetupSys (void)
{
	int pin ;
	char fName [128] ;

	(void)wiringPiSetup();

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupSys called\n");

	// Open and scan the directory, looking for exported GPIOs, and pre-open
	//	the 'value' interface to speed things up for later

	for (pin = 0 ; pin < 256 ; ++pin)
	{
		switch (libwiring.model) {
		case	MODEL_ODROID_N1:
		case	MODEL_ODROID_N2:
			sprintf (fName, "/sys/class/gpio/gpio%d/value", pin + libwiring.pinBase);
			break;
		default:
			sprintf (fName, "/sys/class/gpio/gpio%d/value", pin);
			break;
		}

		libwiring.sysFds [pin] = open (fName, O_RDWR);
	}

	initialiseEpoch ();

	libwiring.mode = MODE_GPIO_SYS;
	return 0;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

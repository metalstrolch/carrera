/*
 * CarAttiny25.c
 *
 * Created: 08.01.2018 15:27:24
 * Author : Alexander Zakharyan
 * Author : Stefan Wildemann
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/pgmspace.h>

/* Pinout */
#define TRACK_PIN PB4
#define MOTOR_PIN PB1
#define IRLED_PIN PB0
#define FRONTLIGHT_PIN PB3
#define STOPLIGHT_PIN PB2
#define NU_PIN3 PB5

/* Number all packets are repeated on track in ms */
#define PACKET_PERIOD_MS 75

/* Two clicks of the same controller inside this time window form a doubleclick in ms */
#define DBLCLICK_DELAY_MS 500
#define DBLCLICK_DELAY DBLCLICK_DELAY_MS/PACKET_PERIOD_MS

/* Abort / continue programmng after this time in ms */
#define PROG_TIMEOUT_MS 2000
#define PROG_TIMEOUT PROG_TIMEOUT_MS/PACKET_PERIOD_MS

/* Time a car needs to be stopped to switch the light in ms */
#define STOPBEFORELIGHT_DELAY_MS 3000
#define STOPBEFORELIGHT_DELAY STOPBEFORELIGHT_DELAY_MS/PACKET_PERIOD_MS

#define STOPLIGHT_DELAY_MS 500
#define STOPLIGHT_DELAY STOPLIGHT_DELAY_MS/PACKET_PERIOD_MS

/* track frequency in Hz */
#define TRANSM_FREQ 10000
#define PERIOD_QURT_CICLES F_CPU/TRANSM_FREQ/4

#define PROG_WORD_CHECK 12
#define ACTIV_WORD_CHECK 7
#define CONTROLLER_WORD_CHECK 9

/* special car IDs for ghost and pace car */
#define GHOST_CAR_ID 6
#define PACE_CAR_ID 7

/* maximum settings values */
#define MAX_CAR_SPEED_CURVE 15
#define MAX_CAR_SPEED 15

/* EEProm variables */
uint8_t EEMEM eeprom_carID; 
uint8_t EEMEM eeprom_progInNextPowerOn; 
uint8_t EEMEM eeprom_ghostSpeed; 
uint8_t EEMEM eeprom_speedCurve; 
uint8_t EEMEM eeprom_lightOn;

/* Speed table containing the speed curves. 0 == stop, 255 == full speed */
const uint8_t speedTable[MAX_CAR_SPEED_CURVE + 1][MAX_CAR_SPEED + 1] PROGMEM = {
	{  0,  3,  6, 11, 14, 19, 22, 26, 30, 34, 38, 42, 45, 50, 53, 62},  //
	{  0,  4,  8, 14, 18, 24, 28, 32, 38, 42, 48, 52, 56, 62, 66, 76},  // 1. LED blinks
	{  0,  4, 10, 16, 22, 28, 32, 38, 44, 50, 56, 60, 66, 72, 78, 88},  // 1. LED on
	{  0,  6, 13, 22, 28, 34, 40, 46, 52, 58, 64, 70, 76, 82, 88,100},  // 2. LED blinks
	{  0,  7, 15, 23, 30, 37, 44, 51, 58, 65, 72, 79, 86, 93,100,115},  //
	{  0,  8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96,104,112,126},  // 2. LED on
	{  0,  8, 16, 26, 36, 44, 56, 64, 72, 80, 88, 96,104,113,122,138},  // 3. LED blinks
	{  0,  8, 18, 28, 38, 48, 56, 66, 76, 86, 96,104,114,124,133,152},  // 3. LED on
	{  0,  9, 20, 32, 41, 52, 62, 72, 83, 94,104,114,125,136,147,165},  //
	{  0, 10, 22, 35, 44, 56, 67, 78, 90,101,112,124,136,148,160,176},  // 4. LED blinks
	{  0, 11, 23, 37, 48, 60, 72, 84, 96,108,120,132,144,157,169,190},  //
	{  0, 12, 24, 38, 51, 64, 76, 89,102,115,128,140,152,165,178,202},  // 4. LED on
	{  0, 13, 26, 41, 55, 68, 81, 95,108,122,136,149,162,176,189,215},  //
	{  0, 14, 28, 43, 58, 72, 86,100,114,129,144,158,172,186,200,228},  // 5. LED blinks
	{  0, 15, 30, 46, 61, 76, 91,106,121,137,152,167,182,197,212,240},  //
	{  0, 16, 32, 48, 64, 80, 96,112,128,144,160,176,192,208,224,255},  // 5. LED on
};

uint8_t volatile carID = 255;
uint8_t volatile currentSpeed = 0;
#define PROG_MODE_TO 0x10
#define PROG_MODE_NONE 0
#define PROG_MODE_CONTROLLER 1
#define PROG_MODE_GHOSTCAR 2
#define PROG_MODE_PACECAR 3
#define PROG_MODE_SPEED (PROG_MODE_NONE | 0x80)
#define PROG_MODE_WAIT_FOR_GHOSTCAR (PROG_MODE_NONE | PROG_MODE_TO)
#define PROG_MODE_WAIT_FOR_PACECAR (PROG_MODE_GHOSTCAR | PROG_MODE_TO)
uint8_t volatile progMode = 0; /* 0 normal, 1 controller, 2 ghostcar, 3 pacecar */
uint8_t volatile progTime = 0;
uint8_t volatile ghostRun = 1; /* ghostcar stopped == 0, ghostcar free to run == 1 */
uint8_t volatile ghostSpeed = 0;
uint8_t volatile speedCurve = 0;
uint8_t volatile progSpeedSelecter = 0;
uint8_t volatile lightOn = 0;
uint8_t volatile stopTime = 0;
uint8_t volatile stopLightTime = 0;
uint8_t volatile lastSpeed = 0;
uint8_t volatile anyKeyPressed = 0;

void playTone() {
	cli();
	TCCR0B = (0 << WGM02) | (1 << CS01) | (1 << CS00); // div 64
	OCR0B = 240;
	_delay_ms(150);	
	OCR0B = 255;
	TCCR0B = (0 << WGM02) | (1 << CS00); // no div
	sei();
}

static inline void timersInit() {
	// MOTOR PWM
	TCCR0A = (1 << COM0B0) | (1 << COM0B1) | (3 << WGM00);
	TCCR0B = (0 << WGM02) | (1 << CS00); // no div
	OCR0B = 255;

	// LED PWM
	TCCR1 |= (1 << CTC1) | (1 << PWM1A) | (1 << COM1A0) | (1 << CS12) | (1 << CS10); // div 16
}

void startLEDPWM() {
	OCR1C = 31 + (32 * carID);
	OCR1A = OCR1C - 16;
}

static inline void pinsInit() {
	DDRB = (1 << MOTOR_PIN) | (1 << IRLED_PIN) | (1 << FRONTLIGHT_PIN) | (1 << STOPLIGHT_PIN); //sets as output
	
	PORTB = (1 << TRACK_PIN); //PullUp TRACK
	PORTB |= (1 << NU_PIN3); //PullUp not used pins
}

static inline void interruptsInit() {
	GIMSK = (1 << PCIE); //turn on PCINT
	MCUCR = (1 << ISC01) | (0 << ISC00); // falling front
	PCMSK = (1 << TRACK_PIN); //turn on interrupts only on TRACK
	sei();
}

void setCarSpeed(uint8_t speed) {
	if(speed > MAX_CAR_SPEED)
		speed = MAX_CAR_SPEED;
	/* reset stop timer as long as were driving */
	if(speed > 0) {
		stopTime = STOPBEFORELIGHT_DELAY;
	}
	currentSpeed = speed;
	OCR0B = 255 - pgm_read_byte(&speedTable[speedCurve][speed]);
}

void setCarID(uint8_t newId) {
	carID = newId;
	eeprom_write_byte(&eeprom_carID, newId);
	startLEDPWM();
}

void setLights() {
	PORTB &= ~((1 << FRONTLIGHT_PIN) | (1 << STOPLIGHT_PIN));
	PORTB |= (lightOn & ((1 << FRONTLIGHT_PIN) | (1 << STOPLIGHT_PIN)));
}

//void blinkLights() {
//	PORTB ^= (1 << FRONTLIGHT_PIN);	
//}

static inline void switchFrontLight() {
	lightOn = ~lightOn;
	eeprom_write_byte(&eeprom_lightOn, lightOn);
	setLights();
}

static inline void calcStopLightTime(uint8_t speed) {
	if (lastSpeed - speed > 2) {
		stopLightTime = STOPLIGHT_DELAY;
		setLights();
	} else {
		if (stopLightTime > 0) {
			stopLightTime--;
		}
	}
}

static inline void stopLightMiddleOn() {
	if (lightOn && (stopLightTime == 0)) {
		PORTB ^= (1 << STOPLIGHT_PIN);
	}
}

/* Set prog mode. And store it to eeprom. Intermediate states are equal to
 * main modes on eeprom level as they shouldn't survive power cycle.
 * Start progTime if required for the state */
void setProgMode(uint8_t mode)
{
	/* sanity check for empty flash */
	if(mode == 0xff)
	{
		mode = PROG_MODE_NONE;
	}
	/* save in eeprom if neccesary */
	if(progMode != mode)
	{
		progMode = mode;
		eeprom_write_byte(&eeprom_progInNextPowerOn, mode & 0x0f);
	}
	/* arm or disarm timer */
	if((mode == PROG_MODE_CONTROLLER) || ((mode & PROG_MODE_TO) == PROG_MODE_TO))
	{
		progTime = PROG_TIMEOUT;
	} else
	{
		progTime = 0;
	}
}

/* React to any (unspecific) controller being pressed */
static inline void onAnyKeyPressed() {
	switch(progMode)
	{
		case PROG_MODE_WAIT_FOR_PACECAR:
			/* Driving the car aborts waiting for pacecar programming */
			setProgMode(PROG_MODE_GHOSTCAR);
			break;
		case PROG_MODE_WAIT_FOR_GHOSTCAR:
			/* Pressing a control aborts waiting for the second double click.
			 * Programming is finished then
			 */
			setProgMode(PROG_MODE_NONE);
			break;
		case PROG_MODE_SPEED:
			/* Being in speed programming mode, pressing a control confirms the selected value
			 */
			speedCurve = progSpeedSelecter;
			eeprom_write_byte(&eeprom_speedCurve, speedCurve);
			setProgMode(PROG_MODE_NONE);
			break;
	}
}

/* When the progTime ran out */
static inline void onProgTimeout()
{
	if(progMode == PROG_MODE_WAIT_FOR_PACECAR)
	{
		/* Timeout waiting for pacecar programming double click.
		 * Continue with ghostcar programming
		 */
		setProgMode(PROG_MODE_GHOSTCAR);
	} else {
		/* All other timeouts abort programming. But remember, there can only
		 * be a timeout if a timeout was set. Not all states have timeout
		 */
		setProgMode(PROG_MODE_NONE);
	}
}

/* When any controllers swith button was pressed and released once */
static inline void onClick(uint8_t controllerId)
{
	switch(progMode)
	{
		case PROG_MODE_GHOSTCAR:
		case PROG_MODE_PACECAR:
			/* In case of ghostcar or pacecar programming (driving) pressing the key
			 * of the controller confirms the actual speed to eeprom and sets the ID */
			if((controllerId == carID) && (currentSpeed > 0))
			{
				ghostSpeed = currentSpeed;
				eeprom_write_byte(&eeprom_ghostSpeed, ghostSpeed);
				setCarID((progMode == PROG_MODE_GHOSTCAR) ? GHOST_CAR_ID : PACE_CAR_ID);
			}
			setProgMode(PROG_MODE_NONE);
		break;

		case PROG_MODE_WAIT_FOR_GHOSTCAR:
			/* One additional (3rd) click after confirming the controller brings us to
			 * manual speed programming mode */
			setProgMode(PROG_MODE_SPEED);
		break;

		case PROG_MODE_SPEED:
		{
			/* Were in manual speed prog mode (for 143 red box).
			 * increase speed index on every click. Will be saved
			 * one the car is driven. We try to buzz out the actual value.
			 */
			uint8_t a;
			progSpeedSelecter++;
			if(progSpeedSelecter > MAX_CAR_SPEED_CURVE)
			{
				progSpeedSelecter = 0;
			}
			for(a = 0; a < progSpeedSelecter; a++) {
				_delay_ms(100);
				playTone();
			}
		}
		break;
		case PROG_MODE_NONE:
			/* toggle front light if tanding still long enough */
			if (stopTime == 0)
			{
				switchFrontLight();
			}
		break;
	}
}

/* When a double click of any controller was detected */
static inline void onDoubleClick(uint8_t controllerId)
{
	/* only react on double clicks if vehicle is standing */
	if((!anyKeyPressed) && ((carID >= GHOST_CAR_ID) || (currentSpeed == 0)))
	{
		switch(progMode)
		{
			case PROG_MODE_NONE:
				/* We got a double click. Go might go to programming mode. It's always controller programming,
				 * as the programming type is detected after the power cycle. ProgTime resets this if
				 * no power cycle happenes */
				setProgMode(PROG_MODE_CONTROLLER);
				/* unset prog mode in ram. We don't do anything unless restart */
				progMode = PROG_MODE_NONE;
				/* Wait for reset or timeout */
				break;

			case PROG_MODE_CONTROLLER:
				/* Were in prog mode after a poer cycle and got the correct double click.
				 * So let's bind the carID first and then wait for eventual additional clicks until
				 * progTime or any controller pressing the speed lever */
				setCarID(controllerId);
				playTone();
				setProgMode(PROG_MODE_WAIT_FOR_GHOSTCAR);
				break;

			case PROG_MODE_WAIT_FOR_GHOSTCAR:
				/* We're still waiting for ghost car programming and got another double click in time.
				 * Waiting for pace car programming arms ghostcar programming. Now one can drive as long as 
				 * the controller's switch button is pressed another time. This even survives power cycle
				 */
				playTone();
				setProgMode(PROG_MODE_WAIT_FOR_PACECAR);
				break;

			case PROG_MODE_WAIT_FOR_PACECAR:
				/* another double click detected. We finally reached pacecar programming. now one can drive
				 * as long as the controller's switch button is pressed another time. this even survives power
				 * cycle.
				 */
				playTone();
				setProgMode(PROG_MODE_PACECAR);
				break;
		}
	}
}

static inline void checkDblClick(uint8_t controllerId, uint8_t sw) {
	static volatile uint8_t buttonState = 0;
	static volatile uint8_t lastClick=255;
	static volatile uint8_t clickTimeout=0;
	uint8_t mask = 0x01 << (controllerId & 0x0F);
	if( sw ) {
		if((buttonState & mask) == mask)
		{
			/* button was released */
			if(lastClick == 255) {
				/* first click */
				lastClick = controllerId;
				/* start timer */
				switch(progMode)
				{
					case PROG_MODE_GHOSTCAR:
					case PROG_MODE_PACECAR:
					case PROG_MODE_SPEED:
						/* we don't expect double clicks */
						clickTimeout = 1;
						break;
					default:
						clickTimeout = DBLCLICK_DELAY;
						break;
				}
			}
			else if(lastClick == controllerId)
			{
				/* secondClick */
				onDoubleClick(controllerId);
				/* stop timer */
				clickTimeout = 0;
				/* wait for next */
				lastClick=255;
			}
		}
		/* clear button from button bit mask */
		buttonState &= (~mask);
	} else {
		/* set button in button bit mask */
		buttonState |= mask;
	}
	/* drive the timer */
	if ((controllerId == 2) && (clickTimeout > 0)) {
		clickTimeout --;
		if((clickTimeout == 0) && (lastClick != 255))
		{
			onClick(lastClick);
			lastClick = 255;
		}
	}
}

/* Programming word is coming bit reversed. That's bad as
 * correcting this is quite expensive in processing power
 * and flash space. This functions does so.
 */
static inline int8_t reverseBits(uint8_t byte) {
        uint8_t value = 0;
        if(byte & 0x80) value |= 0x1;
        if(byte & 0x40) value |= 0x2;
        if(byte & 0x20) value |= 0x4;
        if(byte & 0x10) value |= 0x8;
        if(byte & 0x8) value |= 0x10;
        if(byte & 0x4) value |= 0x20;
        if(byte & 0x2) value |= 0x40;
        if(byte & 0x1) value |= 0x80;
        return value;
}

static inline uint16_t reverseBitsW(uint16_t word) {
        uint8_t high = reverseBits((word >> 8) & 0xFF);
        uint8_t low = reverseBits((word) & 0xFF);
        uint16_t value = (((uint16_t) low) << 8) | ((uint16_t) high);
        return value;
}

static inline void onProgramDataWordReceived(uint16_t word) {
	word = reverseBitsW(word);
	uint8_t controllerId = (word >> 13) & 0x07;
	uint8_t command = (word >> 8 ) & 0x1F;
	uint8_t value = (word >> 4) & 0x0F;

	if(controllerId == carID)
	{
		/* seems we got a command */
		switch(command)
		{
			case 0:
				/* Program speed: value 1 to 15 from CU */
				/* no need for boundary check of value. It's 4 bit max */
				if(speedCurve != value)
				{
					speedCurve = value;
					eeprom_write_byte(&eeprom_speedCurve, speedCurve);
				}
				break;
			case 1:
			case 2:
				/* Program brake or fuel level: value 1 to 15 from CU */
				/* no brake or fuel level here */
				while(value > 0)
				{
					_delay_ms(50);
					value --;
					playTone();
				}
				break;
		}
	}
}

static inline void onActiveControllerWordReceived(uint16_t word) {
	uint8_t parity = word & 0xFE;
	parity = parity ^ (parity >> 4);
	parity ^= parity >> 2;
	parity ^= parity >> 1;
	parity &= 0x01;
	if (parity == (word & 0x01)) {
		return;
	}
	
	anyKeyPressed = word & 0x7F;
	if (anyKeyPressed) {
		onAnyKeyPressed();
	}
	
	uint8_t currentKeyPressed = (word << (carID & 0x0F)) & 0x40;
	/* Run the ghostcar in case the CU doesn't send the ghostcar word.
	 * If it does, ghostRun wins*/
	if (carID >= GHOST_CAR_ID) {
		if (ghostRun && anyKeyPressed) {
			setCarSpeed(ghostSpeed);
		}
	} else {
		if (currentKeyPressed) {
			/* We're standing still. but our key was pressed. Prearm motor...*/
			if (currentSpeed == 0) {
				setCarSpeed(1);
			}
		} else {
			/* We're driving. Since our key is not pressed anymore stop motor */
			setCarSpeed(0);
		}
	}
}

static inline void onGhostcarWordReceived(uint16_t word, uint8_t pitlane)
{
	/* decode if shostcar is allowed to run */
	ghostRun = (word >> 3) & 0x01;
	if((ghostRun) && (carID == PACE_CAR_ID))
	{
		/* were a ghost car. Check if were in pit lane */
	        if(pitlane)
		{
		    /* We're in pit lane and requested to stop there. Do so */
		    ghostRun = (word >> 1) & 0x01;
		}
	}
	if(carID >= GHOST_CAR_ID)
	{
		/* we are a ghostcar. Check if we need to stop */
		if(!ghostRun)
		{
			setCarSpeed(0);
		}
		else
		{
			setCarSpeed(ghostSpeed);
		}
	}
}

static inline void driveProgTimer()
{
	if(progTime > 0)
	{

		progTime --;
		if(progTime == 0)
		{
			onProgTimeout();
		}
	}

}

static inline void driveStopTimer()
{

	if(stopTime > 0)
	{
		stopTime --;
	}

}

static inline void onControllerWordReceived(uint16_t word) {
	uint8_t controllerId = (word >> 6) & 0x07;
	uint8_t speed = (word >> 1) & 0x0F;
	uint8_t sw = (word >> 5) & 1;
	/* If our controller, set speed */
	if (controllerId == carID) {
		setCarSpeed(speed);
		calcStopLightTime(speed);
		lastSpeed = speed;
	}
	/* use controller number 1 to drive timers */
	if(controllerId == 1)
	{
		driveProgTimer();
		driveStopTimer();
	}
	/* react on clicks */
	checkDblClick(controllerId, sw);
}

/* Received something from track. Distinguish the messages by their length
 */
static inline void onWordReceived(uint16_t word, uint8_t pitlane) {
	stopLightMiddleOn();
	if ((word >> CONTROLLER_WORD_CHECK) == 1) {
		if ((word >> 6) != 0x0F) {
			onControllerWordReceived(word);
		} else {
			onGhostcarWordReceived(word, pitlane);
		}
	} else
	if ((word >> PROG_WORD_CHECK) == 1) {
		onProgramDataWordReceived(word);
	} else
	if ((word >> ACTIV_WORD_CHECK) == 1) {
		onActiveControllerWordReceived(word);
	}
}

ISR(PCINT0_vect) {
	uint8_t start = TCNT0;
	while ((uint8_t) (TCNT0 - start) < PERIOD_QURT_CICLES);
	start += PERIOD_QURT_CICLES;
	uint8_t firstHalfCycle = 1;
	uint8_t secondHalfCycle = 0;
	uint16_t receivedValue = 0;
	while (firstHalfCycle != secondHalfCycle) {
		receivedValue = (receivedValue << 1) | firstHalfCycle;

		while ((uint8_t) (TCNT0 - start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;
		while ((uint8_t) (TCNT0 - start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;

		firstHalfCycle = (PINB >> TRACK_PIN) & 1;

		while ((uint8_t) (TCNT0 - start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;
		while ((uint8_t) (TCNT0 - start) < PERIOD_QURT_CICLES);
		start += PERIOD_QURT_CICLES;
		
		secondHalfCycle = (PINB >> TRACK_PIN) & 1;
	}
	/* we know that firstHalfCycle == secondHalfCycle. If both are 0, were
	 * in pit lanei, otherwise not. At least if the word is otherwise OK. */
	onWordReceived(receivedValue, (firstHalfCycle==0));
	GIFR = 1 << PCIF;
}


int main(void) {
	carID = eeprom_read_byte(&eeprom_carID);
	if(carID > PACE_CAR_ID)
	{
		carID = 0;
	}
	ghostSpeed = eeprom_read_byte(&eeprom_ghostSpeed);
	speedCurve = eeprom_read_byte(&eeprom_speedCurve);
	if (speedCurve > MAX_CAR_SPEED_CURVE) {
		speedCurve = MAX_CAR_SPEED_CURVE;
	}
	progMode = eeprom_read_byte(&eeprom_progInNextPowerOn);
	setProgMode(progMode); /* to handle timer */

	lightOn = eeprom_read_byte(&eeprom_lightOn);
	
	power_adc_disable();
	power_usi_disable();
	//ADCSRA = 0; //disable the ADC

	pinsInit();
	timersInit();
	startLEDPWM();
	setLights();
	interruptsInit();
	
	set_sleep_mode(SLEEP_MODE_IDLE);
	while (1) {
		sleep_enable(); 
		sleep_cpu(); 
	}
}


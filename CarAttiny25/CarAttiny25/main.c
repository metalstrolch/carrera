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

#define TRACK_PIN PB4
#define MOTOR_PIN PB1
#define IRLED_PIN PB0
#define FRONTLIGHT_PIN PB3
#define STOPLIGHT_PIN PB2
#define NU_PIN3 PB5

#define PACKET_PERIOD_MS 75
#define DBLCLICK_DELAY_MS 250
#define DBLCLICK_DELAY DBLCLICK_DELAY_MS/PACKET_PERIOD_MS
#define PROG_TIMEOUT_MS 2000
#define PROG_TIMEOUT PROG_TIMEOUT_MS/PACKET_PERIOD_MS
#define STOPBEFORELIGHT_DELAY_MS 3000
#define STOPBEFORELIGHT_DELAY STOPBEFORELIGHT_DELAY_MS/PACKET_PERIOD_MS
#define STOPLIGHT_DELAY_MS 500
#define STOPLIGHT_DELAY STOPLIGHT_DELAY_MS/PACKET_PERIOD_MS

#define TRANSM_FREQ 10000
#define PERIOD_QURT_CICLES F_CPU/TRANSM_FREQ/4

#define PROG_WORD_CHECK 12
#define ACTIV_WORD_CHECK 7
#define CONTROLLER_WORD_CHECK 9

#define GHOST_CAR_ID 6
#define PACE_CAR_ID 7

#define SHORT_TONE_MS 100
#define LONG_TONE_MS 200

#define MAX_CAR_SPEED_CURVE 9
#define MAX_CAR_SPEED 15

uint8_t EEMEM eeprom_carID; 
uint8_t EEMEM eeprom_progInNextPowerOn; 
uint8_t EEMEM eeprom_ghostSpeed; 
uint8_t EEMEM eeprom_speedCurve; 
uint8_t EEMEM eeprom_lightOn;

const uint8_t speedTable[MAX_CAR_SPEED_CURVE + 1][MAX_CAR_SPEED + 1] PROGMEM = {
				    {0,  5, 10, 20, 25, 30,  35,  40,  45,  50,  55,  60,  70,  80,  90, 110}, // child 1, very slow
				    {0, 10, 20, 31, 41, 51,  61,  71,  82,  92, 102, 112, 122, 133, 143, 153}, // child 2, medium
				    {0, 38, 46, 56, 65, 77,  85,  94, 103, 114, 124, 133, 142, 151, 161, 179}, // child 3, fast
				    {0,  5, 12, 18, 27, 39,  53,  67,  81,  95, 109, 124, 138, 153, 156, 155},
				    {0,  5, 12, 18, 27, 39,  53,  67,  81,  95, 109, 124, 138, 153, 156, 155},
				    {0,  5, 12, 18, 27, 39,  53,  67,  81,  95, 109, 124, 138, 153, 156, 155},
				    {0,  5, 12, 18, 27, 39,  53,  67,  81,  95, 109, 124, 138, 153, 156, 155},
				    {0,  5, 12, 18, 27, 39,  53,  67,  81,  95, 109, 124, 138, 153, 156, 155},
				    {0, 17, 34, 51, 68, 85, 102, 119, 136, 153, 170, 187, 204, 221, 238, 255}, //full throttle 2, linear
				    {0,  2, 13, 24, 35, 46,  57,  68,  79,  90, 105, 125, 150, 175, 210, 255}, // full throttle 1, flat
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
#define PROG_MODE_WAIT_FOR_PACECAR (PROG_MODE_PACECAR | PROG_MODE_TO)
uint8_t volatile progMode = 0; /* 0 normal, 1 controller, 2 ghostcar, 3 pacecar */
uint8_t volatile progTimer = 0;
uint8_t volatile ghostRun = 1; /* ghostcar stopped == 0, ghostcar free to run == 1 */
uint8_t volatile ghostSpeed = 0;
uint8_t volatile speedCurve = 0;
uint8_t volatile progSpeedSelecter = 0;
uint8_t volatile lightOn = 0;
uint8_t volatile stopTime = 0;
uint8_t volatile stopLightTime = 0;
uint8_t volatile lastSpeed = 0;

void playTone() {
	cli();
	TCCR0B = (0 << WGM02) | (1 << CS01) | (1 << CS00); // div 64
	OCR0B = 240;
	_delay_ms(150);	
	OCR0B = 255;
	TCCR0B = (0 << WGM02) | (1 << CS00); // no div
	sei();
}

void timersInit() {
	// MOTOR PWM
	TCCR0A = (1 << COM0B0) | (1 << COM0B1) | (3 << WGM00);
	TCCR0B = (0 << WGM02) | (1 << CS00); // no div
	OCR0B = 255;

	// LED PWM
	TCCR1 |= (1 << CTC1) | (1 << PWM1A) | (1 << COM1A0) | (1 << CS12) | (1 << CS10); // div 16
}

void startLEDPWM() {
	OCR1C = 32 * (carID + 1);
	OCR1A = OCR1C - 16;
}

void pinsInit() {
	DDRB = (1 << MOTOR_PIN) | (1 << IRLED_PIN) | (1 << FRONTLIGHT_PIN) | (1 << STOPLIGHT_PIN); //sets as output
	
	PORTB = (1 << TRACK_PIN); //PullUp TRACK
	PORTB |= (1 << NU_PIN3); //PullUp not used pins
}

void interruptsInit() {
	GIMSK = (1 << PCIE); //turn on PCINT
	MCUCR = (1 << ISC01) | (0 << ISC00); // falling front
	PCMSK = (1 << TRACK_PIN); //turn on interrupts only on TRACK
	sei();
}

void setCarSpeed(uint8_t speed) {
	if(speed > MAX_CAR_SPEED)
		speed = MAX_CAR_SPEED;
	currentSpeed = speed;
	OCR0B = 255 - pgm_read_byte(&speedTable[speedCurve][speed]);
}

void setCarID(uint8_t newId) {
	carID = newId;
	eeprom_write_byte(&eeprom_carID, newId);
	startLEDPWM();
}

void calcStopTime(uint8_t speed) {
	if (speed == 0) {
		if (stopTime < 255) {
			stopTime++;
		}
	} else {
		stopTime = 0;
	}
}

void setLights() {
	PORTB &= ~((1 << FRONTLIGHT_PIN) | (1 << STOPLIGHT_PIN));
	PORTB |= (lightOn & ((1 << FRONTLIGHT_PIN) | (1 << STOPLIGHT_PIN)));
}

void blinkLights() {
	PORTB ^= (1 << FRONTLIGHT_PIN);	
}

void switchFrontLight() {
	stopTime = STOPBEFORELIGHT_DELAY - DBLCLICK_DELAY;
	lightOn = ~lightOn;
	eeprom_write_byte(&eeprom_lightOn, lightOn);
	setLights();
}

void calcStopLightTime(uint8_t speed) {
	if (lastSpeed - speed > 2) {
		stopLightTime = STOPLIGHT_DELAY;
		setLights();
	} else {
		if (stopLightTime > 0) {
			stopLightTime--;
		}
	}
}

void stopLightMiddleOn() {
	if (lightOn && (stopLightTime == 0)) {
		PORTB ^= (1 << STOPLIGHT_PIN);
	}
}

void setProgMode(uint8_t mode)
{
	if(mode == 0xff)
	{
		mode = PROG_MODE_NONE;
	}
	if(progMode != mode)
	{
		progMode = mode;
		eeprom_write_byte(&eeprom_progInNextPowerOn, mode & 0x0f);
	}
	if((mode == PROG_MODE_CONTROLLER) || ((mode & PROG_MODE_TO) == PROG_MODE_TO))
	{
		progTimer = PROG_TIMEOUT;
	} else
	{
		progTimer = 0;
	}
}

void onAnyKeyPressed() {
	/* stop timeout */
	switch(progMode)
	{
		case PROG_MODE_WAIT_FOR_PACECAR:
			setProgMode(PROG_MODE_GHOSTCAR);
			break;
		case PROG_MODE_WAIT_FOR_GHOSTCAR:
			setProgMode(PROG_MODE_NONE);
			break;
		case PROG_MODE_SPEED:
			speedCurve = progSpeedSelecter;
			eeprom_write_byte(&eeprom_speedCurve, speedCurve);
			setProgMode(PROG_MODE_NONE);
			break;
	}
}

void onProgTimeout()
{
	if(progMode == PROG_MODE_WAIT_FOR_PACECAR)
	{
		setProgMode(PROG_MODE_GHOSTCAR);
	} else {
		setProgMode(PROG_MODE_NONE);
	}
}

void onClick(uint8_t controllerId)
{
	switch(progMode)
	{
		case PROG_MODE_GHOSTCAR:
		case PROG_MODE_PACECAR:
			if(currentSpeed > 0)
			{
				ghostSpeed = currentSpeed;
				eeprom_write_byte(&eeprom_ghostSpeed, ghostSpeed);
				setCarID((progMode == PROG_MODE_GHOSTCAR) ? GHOST_CAR_ID : PACE_CAR_ID);
			}
			setProgMode(PROG_MODE_NONE);
		break;

		case PROG_MODE_WAIT_FOR_GHOSTCAR:
			setProgMode(PROG_MODE_SPEED);
		break;

		case PROG_MODE_SPEED:
		{
			uint8_t a;
			playTone();
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
	}
}

void onDoubleClick(uint8_t controllerId)
{
	/* only react on double clicks if vehicle is standing */
	if(currentSpeed == 0)
	{
		switch(progMode)
		{
			case PROG_MODE_NONE:
				setProgMode(PROG_MODE_CONTROLLER);
				/* unset prog mode in ram. We don't do anything unless restart */
				progMode = PROG_MODE_NONE;
				/* Wait for reset or timeout */
				playTone();
				break;

			case PROG_MODE_CONTROLLER:
				// Got another double click while in prog mode
				setCarID(controllerId);
				playTone();
				setProgMode(PROG_MODE_WAIT_FOR_GHOSTCAR);
				break;

			case PROG_MODE_WAIT_FOR_GHOSTCAR:
				playTone();
				setProgMode(PROG_MODE_WAIT_FOR_PACECAR);
				break;

			case PROG_MODE_WAIT_FOR_PACECAR:
				setProgMode(PROG_MODE_PACECAR);
				break;
		}
	}
}

void checkDblClick(uint8_t controllerId, uint8_t sw) {
	static volatile uint8_t lastClick=255;
	static volatile uint8_t clickTimeout=0;
	if (sw) { // not pressed
		if(lastClick == controllerId) {
			clickTimeout --;
			if(clickTimeout == 0)
			{
				lastClick=255;
				onClick(controllerId);
			}
		}
	} else {
		if(lastClick != controllerId) {
			/* clicked different controller than last time */
			if(lastClick != 255)
			{
				/* The last one was a single click then*/
				onClick(lastClick);
			}
			/* start timeout to wait for decond click */
			lastClick = controllerId;
			clickTimeout = DBLCLICK_DELAY;
		} else {
			if((clickTimeout < DBLCLICK_DELAY) && (clickTimeout > 0))
			{
				lastClick=255;
				clickTimeout=0;
				onDoubleClick(controllerId);
			}
		}
	}
}

int8_t reverseBits(uint8_t byte) {
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

uint16_t reverseBitsW(uint16_t word) {
        uint8_t high = reverseBits((word >> 8) & 0xFF);
        uint8_t low = reverseBits((word) & 0xFF);
        uint16_t value = (((uint16_t) low) << 8) | ((uint16_t) high);
        return value;
}

void onProgramDataWordReceived(uint16_t word) {
	word = reverseBitsW(word);
	uint8_t controllerId = (word >> 13) & 0x07;
	uint8_t command = (word >> 8 ) & 0x1F;
	uint8_t value = (word >> 4) & 0x0F;

	if(controllerId == carID)
	{
		/* seems we got a command */
		if (command == 0) {
			/* Program speed: value 1 to 10 from CU */
			value --; // 0 to 9
			if(value > MAX_CAR_SPEED_CURVE)
			{
				value = MAX_CAR_SPEED_CURVE;
			}
			if(speedCurve != value)
			{
				speedCurve = value;
				eeprom_write_byte(&eeprom_speedCurve, speedCurve);
			}
		}
		if (command == 1) {
			/* Program brake: value 1 to 10 from CU */
			/* no brake here */
			while(value > 0)
			{
				_delay_ms(50);
				value --;
				playTone();
			}
		}
		if (command == 2) {
			/* Program fuel level: value 1 to 10 from CU */
			/* no fuel here */
			while(value > 0)
			{
				_delay_ms(50);
				value --;
				playTone();
			}
		}
	}
}

void onActiveControllerWordReceived(uint16_t word) {
	uint8_t parity = word & 0xFE;
	parity = parity ^ (parity >> 4);
	parity ^= parity >> 2;
	parity ^= parity >> 1;
	parity &= 0x01;
	if (parity == (word & 0x01)) {
		return;
	}
	
	uint8_t anyKeyPressed = word & 0x7F;
	if (anyKeyPressed) {
		onAnyKeyPressed();
	}
	
	uint8_t currentKeyPressed = (word << (carID & 0x0F)) & 0x40;
	/* Run the ghostcar in case the CU doesn't send the ghostcar word.
	 * If it does, ghostRun wins*/
	if ((carID == GHOST_CAR_ID) || (carID == PACE_CAR_ID)) {
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

void onGhostcarWordReceived(uint16_t word)
{
	/* decode if shostcar is allowed to run */
	ghostRun = (word >> 3) & 0x01;
	if(carID == PACE_CAR_ID)
	{
		/* if we are a pacecar, use other bit to determine if we are allowed to run */
		ghostRun = (word >> 1) & 0x01;
	}
	if((carID == GHOST_CAR_ID) || (carID == PACE_CAR_ID))
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
	//uint8_t pacecarActive = (word >> 1) & 0x01;
	//uint8_t pacecarOnTrack = (word >> 2) & 0x01;
}

void onControllerWordReceived(uint16_t word) {
	uint8_t controllerId = (word >> 6) & 0x07;
	uint8_t speed = (word >> 1) & 0x0F;
	uint8_t sw = (word >> 5) & 1;
	if (controllerId == carID) {
		setCarSpeed(speed);
		calcStopTime(speed);
		calcStopLightTime(speed);
		if (!sw && (stopTime > STOPBEFORELIGHT_DELAY) && (progMode == PROG_MODE_NONE)) {
			switchFrontLight();
		}
		lastSpeed = speed;
	}
	if(controllerId == 1)
	{
		if(progTimer > 0)
		{
			progTimer --;
			if(progTimer == 0)
			{
				onProgTimeout();
			}
		}
	}
	checkDblClick(controllerId, sw);
}

void onWordReceived(uint16_t word) {
	stopLightMiddleOn();
	if ((word >> CONTROLLER_WORD_CHECK) == 1) {
		if ((word >> 6) != 0x0F) {
			onControllerWordReceived(word);
		} else {
			onGhostcarWordReceived(word);
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
	onWordReceived(receivedValue);
	GIFR = 1 << PCIF;
}


int main(void) {
	carID = eeprom_read_byte(&eeprom_carID);
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


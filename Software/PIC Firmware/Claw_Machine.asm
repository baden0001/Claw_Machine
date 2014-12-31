;***************************************************
;	Claw_Machine.asm
;***************************************************
;Revision history:
;	V1.00		Program runs and has intermittent problems
;	V1.01		Added debounce on button push (claw drop).  
;					ButtonDbnc = ClawStat,5
;				removed ClawDropTimer (does not appear to be used anywhere in code)
;				Appears to work, ran for 10 minutes and unit did not randomly drop
;				 Claw.  Pushed button after claw was in drop state, no adverse affects.
;				 Button seemed to not be active anymore while claw was in "pick up"
;				 state routine.
;				Randomly the Claw machine will lock up shortly after opening the claw
;				 at the end of the claw home cycle.  No buttons or limits would trigger
;				 a response.  The claw down button would not even create a response.
;				 That said, the interrupt would either not be activated or the pic locked up
;				Next revision should utilize the WDT.  This would allow the program to
;				 reset when a locked machine appeared.
;	V1.02		Added WDT function to kick pic out of locked state if need be
;				 Verified if EnAxis is active that the B port interrupt
;				 is active
;				Modified notes:  added stage 6 to clarify end of claw routine.
;				Cleaned up state machine to use only movlw to avoid
;				 intermediate states that might mess up the interrupt routines.
;				Found out 20 MHz crystal was being used, so the oscillator control
;				 had to be changed from XT to HS

;				 oscillator control change did help some of the problem.  There is still
;				 a problem that is unknown that will cause the machine to try to step
;				 wildly.  Not sure what is causing this.  A reset will cease the
;				 wild stepping but the symptoms still showed when the reset was
;				 released.  It still seems to be an oscillator problem.
;	V1.03		Changed Y control Plus to be on RA3 pin instead of RB6.  This was to allow for 
;				 easier in circuit programming and debugging.
;				Swapped XStep and Xdir to follow schematic
;				Board was also changed during this build and there have not been any
;				 wild stepping happening.  The board design seemed to help
;	V1.04		Add capability to switch between "test" mode and "claw machine" mode
;				 This is set by adjusting the start of the program to save a 1 or 0 to the
;				  TestMode bit
;				 Test mode: allow for button press and hold to change joystick movements into:
;					Forward: Raise claw
;					Back: Lower claw
;					Left: Open claw
;					Right: Close claw
;				Added communication buffering bytes.  The chip can save up to 7 bytes and
;				 only that amount will be saved.  Anything above that will not be retransmitted
;				 back to the user, so the user knows if the buffer is full.	
;				Added Variables:
;				 ClawStat,6 = ProgramMode
;				 ClawStat,7 = TestMode
;				 RXBuf1, RXBuf2, RXBuf3, RXBuf4, RXBuf5, RXBuf6, RXBuf7
;					Used for saving the commands received from the RS232 port
;				 RXBufPointer
;	V1.05		Added Menu structure for setting values with RS232 terminal.  Menu will only
;				 be entered at the end of a claw push button routine (in claw mode) or 
;				 when the push button is released in test mode.
;				 Can set speed for all axis
;				 Change between claw machine operation and test mode via serial
;				 Data currently cannot be saved and the time to drop claw
;				Added Variables:
;				 RXStatus, RXTempBuf, TempSpeed	
;	V1.06		Added saving/retrieving settings from EEPROM
;				Added TimeZ variable being saved/retrieved
;				TimeZ adjusts the time that the claw drops in claw machine mode
;				 This value is multiplied by 5
;				Delay2Sec now requires the WREG to be filled with a count
;				 prior to being called.
;				May need to add code to access PCLATH for calls and gotos 
;				 Rearranged code to push the interrupt @ 0x0400 origin and 
;				 placed PCLATH call for interrupt call.  Code has been rearranged
;				 to push teh interrupt to 0x0400.  Have not added PCLATH
;				 as of yet and the program is running fine.
;				Added Variables:
;				 ADRSpdx, ADRSpdy, ADRSpdz, ADRMode, ADRTimz, TempEE, TempEED, TimeZ
;				Removed Variables:
;				 TimerCountL, TimerCountH
;	V1.07		Added Pop Mode.  This is used to pick up cans and lower the cans down and releasing
;				 them gently, instead of opening claw at the top of the stroke.
;				 The system will have to be out of test mode, active in claw AND pop mode.
;				 Will use Claw mode general operation, but will affect the toy drop routine.
;				 Added into user menu options
;					pop
;				 Added into EEPROM exits save/retreive on startup
;				 Added into main program flow
;				Added Variables:
;				 ClawStat2, SerialTimer 
;				Added Constants:
;				 SerialButton, SerialForward, SerialBack, SerialLeft,
;				 SerialRight, PopMode
;	V1.08		Added quick controls via RS232 port.  
;				 This will allow the claw machine to be controlled via quick commands
;				 sent from computer.  Control forward, back, left, right, and button 
;				 push.
;				I = Forward (0x49)
;				K = Backward (0x4B)
;				L = Right (0x4C)
;				J = Left (0x4A)
;				<spacbar> = Push button (0x20)
;				When the serial port receives in direction controls, it will set the SerialTimerX or SerialTimerY
;				 with a preset time down value of x amount of steps.  Each time the command is received,
;				 the count down timer will reset.  
;				When the serial port receives in the push button <spacebar>, then the debounce
;				 bit will be hijacked to force a claw drop.  This will only work for claw machine
;				 mode.
;				Added into user menu options
;				 srtmrx
;				 srtmry
;				Added into EEPROM exits save/retreive on startup
;				Added Constants:
;				 ADRSTmrX, ADRSTmrY
;				Removed Variables:
;				 SerialTimer
;				Added Variables:
;				 SerialTimerX, SerialTimerY, SerialTimerXSet
;				 SerialTimerYSet
;
;	Crystal: 20 Mhz
;
;	Outputs:
;		Claw_Close		RA2		'0' = open
;								'1' = close
;		ClawDir			RA4		'0' = up
;								'1' = down
;		ClawStep		RA5
;		XMotStp			RC5
;		XMotDir			RC4		'0' = left (minus)
;								'1' = right (plus)
;		YMotStp			RC2
;		YMotDir			RC3		'0' = front (minus)
;								'1' = back	(plus)
;
;	Inputs:
;		XControlMin		RA0		Limit switch input for joystick
;		YControlMin		RA1		Limit switch input for joystick
;		XControlPlus	RB5		Limit switch input for joystick
;		YControlPlus	RA3		Limit switch input for joystick
;		ClawDrop		RB0		Limit switch input for joystick
;		XLimPlus		RB1		Limit
;		XLimMinus		RB2		Limit
;		YLimPlus		RB3		Limit
;		YLimMinus		RB4		Limit
;		ClawUpLim		RC1		Limit switch to know claw is up
;	
;	Communications (Added Version 1.04):
;		RS232 Receive	RC7
;		RS232 Transmit	RC6
;	
;	Notes:	
;
;		Using 20 MHz crystal
;
;
;		Timer2 is used to set a period of .1984 mS
;
;		Timer2 = Interval Time/(Tosc*4*prescaler)
;		 = 0.0001984/(50 ns*4*16) = 62 = 0x3e
;		 This value is placed into PR2 which acts
;		 as a comparator, once Timer2 = PR2
;		 Timer2 resets to zero
;
;		62 = x/(50nS*4*16)
;		x = .0001984 Seconds = .1984 mS
;
;		4MHz crystal
;
;		Timer2 is used to set a period of 1 mS
;
;		Timer2 = Interval Time/(Tosc*4*prescaler)
;		 = 0.001/(250 ns*4*16) = 62 = 0x3e
;		 This value is placed into PR2 which acts
;		 as a comparator, once Timer2 = PR2
;		 Timer2 resets to zero
;***************************************************
	list p=16F870

	include "p16F870.inc"

;	Turn WDT on and start to clear timer.
	LIST
	__config _HS_OSC & _WDT_ON & _LVP_OFF

;	oscillator was set to XT, changed it over to HS for 20 MHz crystal.  This will provide higher
;		gain to keep oscillator alive

;Setup variables
CBLOCK 0x20
StatTemp, PCLTemp, WTemp, Temp, SpeedX, SpeedY
TimerX, TimerY, ClawStat, SpeedClaw, TimerClaw, Delay2, lowDbnc, hiDbnc
Inner, Outer, RXBuf0, RXBuf1, RXBuf2, RXBuf3, RXBuf4, RXBuf5, RXBuf6, RXBuf7, RXBufPointer
RXStatus, RXTempBuf, TempSpeed, TempEE, TempEED, TimeZ, ClawStat2, SerialTimerX, SerialTimerY
SerialTimerXSet, SerialTimerYSet
ENDC

;***************************************************
;	Define addresses for EEPROM locations
ADRSpdx	equ	0
ADRSpdy	equ	1
ADRSpdz	equ	2
ADRMode	equ	3
ADRTimz	equ	4
ADRSTmrX equ 5
ADRSTmrY equ 6

;***************************************************
;	Define outputs (can redefine pins if needed here)
#define Claw_Close		PORTA,2
#define ClawStep		PORTA,5
#define ClawDir			PORTA,4
#define XControlMin		PORTA,0
#define YControlMin		PORTA,1
#define	XControlPlus	PORTB,5
#define YControlPlus	PORTA,3
#define XMotStp			PORTC,5
#define XMotDir			PORTC,4
#define YMotStp			PORTC,2
#define YMotDir			PORTC,3
#define ClawUpLim		PORTC,1
#define ClawDrop		PORTB,0
#define XLimPlus		PORTB,1
#define XLimMinus		PORTB,2
#define YLimPlus		PORTB,3
#define YLimMinus		PORTB,4
#define ButtonPush		ClawStat,0
#define RaiseClaw		ClawStat,1
#define	LowerClaw		ClawStat,2
#define	Home			ClawStat,3
#define EnAxis			ClawStat,4
#define ButtonDbnc		ClawStat,5
#define ProgramMode		ClawStat,6
#define TestMode		ClawStat,7
#define EnterDetected	RXStatus,0
#define	EscapeDetected	RXStatus,1
#define SerialButton	ClawStat2,0
#define SerialForward	ClawStat2,1
#define SerialBack		ClawStat2,2
#define SerialLeft		ClawStat2,3
#define SerialRight		ClawStat2,4
#define PopMode			ClawStat2,5

;***************************************************
;Controls:
;
;Variables:
;	StatTemp, PCLTemp, WTemp	Temps used to save state info
;								 during Interrupts
;	TimerCountL, TimerCountH	Keep Track of timer (no longer used)
;	Temp						Universal variable, needs to be used locally only
;	SpeedX, SpeedY				Holds speed of axis for corresponding axis
;	TimerX, TimerY				Holds step timer count for corresponding axis
;	ClawStat					Holds 8 bits for status of claw machine
;								bit 0 = Button pushed for claw to drop (ButtonPush)
;									1 = Raise Claw up
;									2 = Lower Claw
;									3 = Ignore joystick and home both axis
;									4 = Enable both axis control on joystick
;									5 = Button pushed debounce pass
;									6 = Claw Machine is in Program Mode
;									7 = Claw Machine is in Test Mode
;	ClawStat2					Adds another place to save claw stat data
;									0 = Spacebar pushed via 232 port
;									1 = Forward pushed via 232 port
;									2 = Back pushed via 232 port
;									3 = Left pushed via 232 port
;									4 = Right pushed via 232 port
;									5 = Claw Machine is in Pop Can Mode
;	TimerClaw					HOlds step timer count for claw axis
;	SpeedTimer					Holds speed of axis for corresponding axis					
;	Inner, Outer, Delay2		Used for main program timing
;	lowDbnc, hiDbnc				Used in detecting debounce
;	RXBuf0, RXBuf1, RXBuf2, RXBuf3, RXBuf4, RXBuf5, RXBuf6, RXBuf7
;								Used for saving the commands received from the RS232 port
;	RXBufPointer				Used for keeping track of which byte is empty for
;								 receiving.  This will point to the next byte that is empty
;								 If the pointer is pointing @ 8 then
;								 the buffer is full
;	RXStatus					Holds 8 bits for status of received data
;								bit 0 = '1' <Enter> was detected
;										'0' No <Enter> detected
;									1 = '1' <Escape> detected
;										'0' No <Escape> detected 
;	RXTempBuf					Use to saved RCREG value when it is read, will then transfer
;								 to RXBuf if there is room and the value of it is not
;								 CR or LF 
;	TempSpeed					Used in subExtractRXBufValue to send speed
;								 value back to program
;	ADRSpdx, ADRSpdy, ADRSpdz	EEPROM locations to store the speed of each axis
;	ADRMode						EEPROM location to save either modes:
;									'0' = Claw Mode
;									'1' = Test Mode
;									'2' = Pop Mode
;	ADRTimz						EEPROM location to save time to lower claw
;	TempEE						EEPROM data pointer
;	TempEED						EEPROM data pulled
;	TimeZ						Time to drop claw in claw mode
;	SerialTimerX, SerialTimerY	Timers to keep track of 232 movement commands.  If
;								 timer countsdown before next command is received
;								 then stop movement.  Used on SerialForward, SerialBack,
;								 SerialLeft, and SerialRight.  Spacebar will need
;								 this for test mode but not for claw mode.
;	ADRSTmrX, ADRSTmrY			EEPROM locations to store the timer value used for 
;								 serial control.  These will fill in the values
;								 for SerialTimerX and SerialTimerY
;	SerialTimerXSet, SerialTimerYSet
;								Retains setpoint that was pulled out of the EEPROM on
;								 startup and after changing it via the menu.
;***************************************************

Reset_Vector	code 0x0000
 	goto	Start

Int_Vector		code 0x0004
;	movlw	HIGH IntService  linker file does not allow this to be as large as it is.
;	movwf	PCLATH
	goto	IntService

	code 0x002A
;****************************************
;	Setup Outputs and Timer
;****************************************
Start
;Clear WDT to prevent any bad behaviors during setup
; may want to add code to show that a wdt has caused a reset
; possibly throw into eeprom to save the count that can be looked at later
; during debugging?
	clrwdt
;Initialize variables
;	clrf	TimerCountL		;Clear timer count
;	clrf	TimerCountH
	clrf	ClawStat		;ButtonPush, RaiseClaw, LowerClaw
							;Home, EnAxis, and ButtonDbnc
							; are set to "0"
	clrf	ClawStat2		;232 port commands and Pop Can Mode
							; are set to "0"

;Setup Inputs
	clrf	PORTB
	bsf		STATUS,RP0		;Bank 1 select
	movlw	0xFF
	movwf	TRISB			;Set PORTB as inputs
	bsf		OPTION_REG,6	;RB0 Rising Edge Interrupt
	bcf		STATUS,RP0		;Bank 0 Select

;Setup Outputs
	clrf	PORTA
	clrf	PORTC
	bsf		STATUS,RP0		;Bank 1 select

;May have to add singular bits to turn on certain pins of PORTA to inputs
	movlw 	0x06 			;Configure all pins
	movwf 	ADCON1 			;as digital inputs
	clrf	TRISA			;Set PORTA to outputs
	bsf		TRISA,0			;Set PORTA 0,1,3 to inputs
	bsf		TRISA,1
	bsf		TRISA,3
	clrf	TRISC			;Set PORTC 0,2-7 to outputs
	bsf		TRISC,1			;Set PORTC 1 to input
	bcf		STATUS,RP0		;Bank 0 select
	clrf	PORTA
	clrf	PORTB

;Setup Communication for RS232 port capability
;With a 20 MHz crystal, 9600 baud and BRGH = 1, the following
; will need to be set for the SPBRG register:  129
;Enable transmit interrupt:  TXIE( PIE1<4>)
; This will allow to interrupt operation when the TXREG register has
; sent out data.  TXIF (PIR1<4>) will set high when this happens.  This
; flag will only reset when the TXREG register is filled with new data.
;TXREG holds the data to be transmitted out.

;1. Initialize the SPBRG register for the appropriate
;baud rate. If a high speed baud rate is desired,
;set bit BRGH (Section 9.1).
;2. Enable the asynchronous serial port by clearing
;bit SYNC and setting bit SPEN.
;3. If interrupts are desired, then set enable bit
;RCIE.
;4. If 9-bit reception is desired, then set bit RX9.
;5. Enable the reception by setting bit CREN.
;6. Flag bit RCIF will be set when reception is complete and an interrupt will be generated if enable
;bit RCIE is set.
;7. Read the RCSTA register to get the ninth bit (if
;enabled) and determine if any error occurred
;during reception.
;8. Read the 8-bit received data by reading the
;RCREG register.
;9. If any error occurred, clear the error by clearing
;enable bit CREN.
;10. If using interrupts, ensure that GIE and PEIE
;(bits 7 and 6) of the INTCON register are set.

	bsf		STATUS,RP0		;Bank 1 select
	bsf		TRISC,7			;RX input
	bcf		TRISC,6			;TX input
	movlw	.129			;set baud rate to 9600 when
							; BRGH = 1
	movwf	SPBRG
	movlw	B'00100100'		;Transmit Enable (TXEN = 1), Asynchronous (SYNCH = 0), BRGH = 1, 
	movwf	TXSTA
	bcf		STATUS,RP0		;Bank 0 select
	movlw	B'10000000'		;Serial Port Enable (SPEN = 1), 8-bit, Continuous Receive off (CREN = 0)
	movwf	RCSTA
	bsf		STATUS,RP0		;Bank 1 select
	bsf		PIE1,RCIE		;Enable RCIE to enable interrupt on reception
	bcf		STATUS,RP0		;Bank 0 select
	bsf		RCSTA,4			;Continuous Receive ON (CREN = 1)

;Clear out the receive buffer and pointer
	clrf	RXBuf0
	clrf	RXBuf1
	clrf	RXBuf2
	clrf	RXBuf3
	clrf	RXBuf4
	clrf	RXBuf5
	clrf	RXBuf6
	clrf	RXBuf7
	clrf	RXBufPointer
	clrf	RXStatus
;***********End Serial Setup**************

;Setup Timer2 and interrupt
	bcf		STATUS,RP0		;Bank 0 select
	movlw	B'00000010'		;1:16 Prescale, 1:1 Postscale, timer off
	movwf	T2CON
	clrf	TMR2			;Reset Timer2 count
	bcf		PIR1,1			;Clear Timer2 Flag
	bsf		STATUS,RP0		;Bank 1 Select
	movlw	0x3e			;Load Limit to Timer2
	movwf	PR2				; Calculated above
	bsf		PIE1,1			;Enable Timer2 Interrupt
	bcf		STATUS,RP0		;Bank 0 Select
	bcf		INTCON,4		;Shut off PORTB,0 interrupt
	bsf		T2CON,2			;Turn Timer2 on
	bsf		INTCON,PEIE		;Enable Peripheral Interrupts
	bsf		INTCON,GIE		;Global Interrupt Enable	

;End Setup

	clrf	TimerX			
	clrf	TimerY
	clrf	TimerClaw
	clrf	SerialTimerX
	clrf	SerialTimerY

;Restore saved values from EEPROM
	movlw	ADRSpdx
	movwf	TempEE
	call	ReadEEPROM
	movwf	SpeedX

	movlw	ADRSpdy
	movwf	TempEE
	call	ReadEEPROM
	movwf	SpeedY

	movlw	ADRSpdz
	movwf	TempEE
	call	ReadEEPROM
	movwf	SpeedClaw

	movlw	ADRTimz
	movwf	TempEE
	call	ReadEEPROM
	movwf	TimeZ

	movlw	ADRSTmrX
	movwf	TempEE
	call	ReadEEPROM	
	movwf	SerialTimerXSet

	movlw	ADRSTmrY
	movwf	TempEE
	call	ReadEEPROM	
	movwf	SerialTimerYSet

	movlw	ADRMode
	movwf	TempEE
	call	ReadEEPROM
	movwf	TempEE
	btfsc	TempEE,0
	goto	lblSetRunTestMode
	bcf		TestMode
	btfsc	TempEE,2
	goto	lblSetRunPopMode
	bcf		PopMode
	goto	lblContinueSetup

lblSetRunTestMode
	bsf		TestMode
	goto	lblContinueSetup

lblSetRunPopMode
	bsf		PopMode

;Set speed for motor (pre eeprom programming)

;	movlw	0x06			;place jogging speed into X (printer axis) 0x10 half step
;	movwf	SpeedX
;	movlw	0x15			;place jogging speed into Y (cable axis) 0x15 half step
;	movwf	SpeedY
;	movlw	0x09			;place jogging speed for Claw 0x10 full step
;	movwf	SpeedClaw
;	bsf		TestMode		;enable test mode of the claw machine
;	movlw	B'01111111'
;	movwf	TimeZ

;Added following code to bypass homing sequence.  Allow for testing

;	bsf		EnAxis			;Re enable axis movement
;	goto	Main

;End code addition
lblContinueSetup
	call	HomeAll
	bsf		INTCON,4		;enable PORTB,0 interrupt
	goto	Main

;****************************************
;	Main Program
;	Checks for an enter press when:
;		Claw machine mode has finished the stages of the button push
;		Test mode - button is not pushed.
;****************************************
Main
	btfsc	EnterDetected	;Check if there is a command to process
	goto	lblEnterDetected

	btfss	ButtonPush		;Wait for button press before hijacking
							; control of claw machine
	goto	Main

	btfsc	TestMode		;Test if claw machine is in test mode
	goto	lblTestMode		;Goto Test Mode operation

;Use for future use if a grid is decided to be used.
;	btfsc	PopMode			;Test if claw machine is in Pop Mode
;	goto	lblPopMode		;Goto Pop Mode (currently not addressed)

	goto	lblClawOp		;Goto claw machine game stages

;****************************************
;	Test Mode Operation
;****************************************

lblTestMode
	btfss	ClawDrop		;if push button is released then kick out of button pushed routine
	goto	lblTestMove

	movlw	B'10010000'
	movwf	ClawStat		;Enable Test Mode and Enable Axis
	bsf		EnAxis			;enable joystick movement
	bcf		INTCON,1		;clear RB0 flag
	bsf		INTCON,4		;enable PORTB,0 interrupt (button press)
	goto	Main
	
lblTestMove
	bcf		EnAxis			;Disable X and Y axis movement since
							; the button is pushed

	btfss	XControlMin
	call	subTestXConMin

	btfss	YControlMin
	call	subTestYConMin

	btfsc	YControlMin		;If user returns joystick to home position, then shut off raising claw
	bcf		RaiseClaw
	
	btfss	XControlPlus
	call	subTestXConPlus

	btfss	YControlPlus
	call	subTestYConPlus

	btfsc	YControlPlus	;If user returns joystick to home position, then shut off lowering claw
	bcf		LowerClaw

	goto	lblTestMode

;****************************************
;		End of Test Mode Operation
;****************************************

;****************************************
;	Enter Detected/Program Mode
;	This section will be used for
;	 processing the buffer data and 
;	 acting on commands
;	Structured for sequential flow into 
;	 menu structure
;	PROG
;		SPDX,SPDY,SPDZ
;			ENTER:
;		CLAW
;		TEST
;		POP
;		TIMZ
;			ENTER:
;		SRTMRX, SRTMRY
;			ENTER:
;		EXIT
;		EXITS
;	Algorithm checks that the correct
;	 values are in the correct buffer location
;	 and that the buffer pointer is correct
;	Once the enter button is pushed Receive
;	 buffer is locked out and no more
;	 data wil be saved to the buffer
;	 until EnterMode bit is cleared
;****************************************
lblEnterDetected
;Detect if command "prog" is detected
;	ASCII	p = 70
;			r = 72
;			o = 6F
;			g = 67
	movlw	0x70
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblMenuExit
	movlw	0x72
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblMenuExit
	movlw	0x6F
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblMenuExit
	movlw	0x67
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblMenuExit
	movlw	0x04			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblMenuExit

	call	subClearBuffer
	bsf		ProgramMode		;Shift chip into programming mode which will
							; suspend movement of the system from the Joystick
	
;In program mode, display that the command was accepted
	call	subReturn
	call	subWaitDisplay
	bcf		EnterDetected	;Program command has been processed, reset enter

;Once the claw machine detects it is in programming mode, maintain control
; in the menu structure.
lblEnterMenu1
	btfss	EnterDetected	;Check if there is a command to process
	goto	lblEnterMenu1

;Detect if command "spdx", "spdy", "spdz" is detected
;	ASCII	s = 73
;			p = 70
;			d = 64
;			x = 78
;			y = 79
;			z = 7A	
lblSpdDetect
	movlw	0x73
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblClawDetect
	movlw	0x70
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblClawDetect
	movlw	0x64
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblClawDetect
	movlw	0x04			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblClawDetect

;**************X Axis*************************

lblSpdXDetect
	movlw	0x78
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblSpdYDetect

	call	subReturn
	call	subEnterDisplay
	call	subClearBuffer
	bcf		EnterDetected

lblEnterMenuX
;Sub Menu
	btfss	EnterDetected	;Check if the user has entered a value
	goto	lblEnterMenuX

	movlw	0x03			;Make sure only 3 bytes are present
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblFailSPDXMenu

	call	subExtractRXBufValue
	movf	TempSpeed,W
	btfsc	STATUS,Z		;Check if speed is zero, if it is, then fail entry
	goto	lblFailSPDXMenu
	movwf	SpeedX			;Move entered value into speedX
	goto	lblSpdConfigured

lblFailSPDXMenu
	call	subReturn
	call	subFailDisplay
	call	subReturn
	call	subEnterDisplay

lblResetSPDXMenu
	call	subClearBuffer
	bcf		EnterDetected
	goto	lblEnterMenuX

;**************Y Axis*************************

lblSpdYDetect
	movlw	0x79
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblSpdZDetect

	call	subReturn
	call	subEnterDisplay
	call	subClearBuffer
	bcf		EnterDetected

lblEnterMenuY
;Sub Menu
	btfss	EnterDetected	;Check if the user has entered a value
	goto	lblEnterMenuY

	movlw	0x03			;Make sure only 3 bytes are present
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblFailSPDYMenu

	call	subExtractRXBufValue
	movf	TempSpeed,W
	btfsc	STATUS,Z		;Check if speed is zero, if it is, then fail entry
	goto	lblFailSPDYMenu
	movwf	SpeedY			;Move entered value into speedY
	goto	lblSpdConfigured

lblFailSPDYMenu
	call	subReturn
	call	subFailDisplay
	call	subReturn
	call	subEnterDisplay

lblResetSPDYMenu
	call	subClearBuffer
	bcf		EnterDetected
	goto	lblEnterMenuY

;**************Z Axis*************************
lblSpdZDetect
	movlw	0x7A
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblClawDetect

	call	subReturn
	call	subEnterDisplay
	call	subClearBuffer
	bcf		EnterDetected

lblEnterMenuZ
;Sub Menu
	btfss	EnterDetected	;Check if the user has entered a value
	goto	lblEnterMenuZ

	movlw	0x03			;Make sure only 3 bytes are present
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblFailSPDZMenu

	call	subExtractRXBufValue
	movf	TempSpeed,W
	btfsc	STATUS,Z		;Check if speed is zero, if it is, then fail entry
	goto	lblFailSPDZMenu
	movwf	SpeedClaw		;Move entered value into speedX
	goto	lblSpdConfigured

lblFailSPDZMenu
	call	subReturn
	call	subFailDisplay
	call	subReturn
	call	subEnterDisplay

lblResetSPDZMenu
	call	subClearBuffer
	bcf		EnterDetected
	goto	lblEnterMenuZ

lblSpdConfigured
	call	subReturn
	call	subDoneDisplay
	call	subReturn
	call	subWaitDisplay
	goto	lblMenu1Reset

;Detect if command "claw" is detected
;	ASCII	c = 63
;			l = 6C
;			a = 61
;			w = 77	
;Affects:
;	TestMode
;	PopMode
lblClawDetect
	movlw	0x63
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblTestDetect
	movlw	0x6C
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblTestDetect
	movlw	0x61
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblTestDetect
	movlw	0x77
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblTestDetect
	movlw	0x04			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblTestDetect

	bcf		TestMode
	bcf		PopMode
	call	subReturn
	call	subDoneDisplay
	call	subReturn
	call	subWaitDisplay
	goto	lblMenu1Reset

;Detect if command "test" is detected
;	ASCII	t = 74
;			e = 65
;			s = 73
;			t = 74	
;Affects:
;	TestMode
;	PopMode
lblTestDetect
	movlw	0x74
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblPopDetect
	movlw	0x65
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblPopDetect
	movlw	0x73
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblPopDetect
	movlw	0x74
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblPopDetect
	movlw	0x04			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblPopDetect

	bsf		TestMode
	bcf		PopMode
	call	subReturn
	call	subDoneDisplay
	call	subReturn
	call	subWaitDisplay
	goto	lblMenu1Reset

;Detect if command "pop" is detected
;	ASCII	p = 70
;			o = 6F
;			p = 70
;Affects:
;	TestMode
;	PopMode	
lblPopDetect
	movlw	0x70
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblTimzDetect
	movlw	0x6F
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblTimzDetect
	movlw	0x70
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblTimzDetect
	movlw	0x03			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblTimzDetect

	bsf		PopMode
	bcf		TestMode
	call	subReturn
	call	subDoneDisplay
	call	subReturn
	call	subWaitDisplay
	goto	lblMenu1Reset

;Detect if command "timz" is detected
;	ASCII	t = 74
;			i = 69
;			m = 6D
;			z = 7A	
lblTimzDetect
	movlw	0x74
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblSerialSpeedDetect
	movlw	0x69
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSerialSpeedDetect
	movlw	0x6D
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSerialSpeedDetect
	movlw	0x7A
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblSerialSpeedDetect
	movlw	0x04			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblSerialSpeedDetect

	call	subReturn
	call	subEnterDisplay
	call	subClearBuffer
	bcf		EnterDetected

lblEnterMenuTimz
;Sub Menu
	btfss	EnterDetected	;Check if the user has entered a value
	goto	lblEnterMenuTimz

	movlw	0x03			;Make sure only 3 bytes are present
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblFailTIMZMenu

	call	subExtractRXBufValue
	movf	TempSpeed,W
	btfsc	STATUS,Z		;Check if speed is zero, if it is, then fail entry
	goto	lblFailTIMZMenu
	movwf	TimeZ			;Move entered value into speedX
	goto	lblTimzConfigured

lblFailTIMZMenu
	call	subReturn
	call	subFailDisplay
	call	subReturn
	call	subEnterDisplay

lblResetTIMZMenu
	call	subClearBuffer
	bcf		EnterDetected
	goto	lblEnterMenuTimz

lblTimzConfigured
	call	subReturn
	call	subDoneDisplay
	call	subReturn
	call	subWaitDisplay
	goto	lblMenu1Reset

;Detect if command "srtmrx", "srtmry" is detected
;	ASCII	s = 73
;			r = 72
;			t = 74
;			m = 6D
;			r = 72
;			x = 78
;			y = 79
lblSerialSpeedDetect
	movlw	0x73
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblExitDetect
	movlw	0x72
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblExitDetect
	movlw	0x74
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblExitDetect
	movlw	0x6D
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblExitDetect
	movlw	0x72
	subwf	RXBuf4,W
	btfss	STATUS,Z
	goto	lblExitDetect

	movlw	0x06			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblExitDetect

;**************X Axis*************************

lblSerialTimerXDetect
	movlw	0x78
	subwf	RXBuf5,W
	btfss	STATUS,Z
	goto	lblSerialTimerYDetect

	call	subReturn
	call	subEnterDisplay
	call	subClearBuffer
	bcf		EnterDetected

lblEnterSerialTimerMenuX
;Sub Menu
	btfss	EnterDetected	;Check if the user has entered a value
	goto	lblEnterSerialTimerMenuX

	movlw	0x03			;Make sure only 3 bytes are present
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblFailSerialTimerXMenu

	call	subExtractRXBufValue
	movf	TempSpeed,W
	btfsc	STATUS,Z		;Check if speed is zero, if it is, then fail entry
	goto	lblFailSerialTimerXMenu
	movwf	SerialTimerXSet	;Move entered value into speedX
	goto	lblSerialTimerConfigured

lblFailSerialTimerXMenu
	call	subReturn
	call	subFailDisplay
	call	subReturn
	call	subEnterDisplay

lblResetSerialTimerXMenu
	call	subClearBuffer
	bcf		EnterDetected
	goto	lblEnterSerialTimerMenuX

;**************Y Axis*************************

lblSerialTimerYDetect
	movlw	0x79
	subwf	RXBuf5,W
	btfss	STATUS,Z
	goto	lblExitDetect

	call	subReturn
	call	subEnterDisplay
	call	subClearBuffer
	bcf		EnterDetected

lblEnterSerialTimerMenuY
;Sub Menu
	btfss	EnterDetected	;Check if the user has entered a value
	goto	lblEnterSerialTimerMenuY

	movlw	0x03			;Make sure only 3 bytes are present
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblFailSerialTimerYMenu

	call	subExtractRXBufValue
	movf	TempSpeed,W
	btfsc	STATUS,Z		;Check if speed is zero, if it is, then fail entry
	goto	lblFailSerialTimerYMenu
	movwf	SerialTimerYSet	;Move entered value into speedY
	goto	lblSerialTimerConfigured

lblFailSerialTimerYMenu
	call	subReturn
	call	subFailDisplay
	call	subReturn
	call	subEnterDisplay

lblResetSerialTimerYMenu
	call	subClearBuffer
	bcf		EnterDetected
	goto	lblEnterSerialTimerMenuY

lblSerialTimerConfigured
	call	subReturn
	call	subDoneDisplay
	call	subReturn
	call	subWaitDisplay
	goto	lblMenu1Reset

;Detect if command "exit" or "exits" is detected
;	ASCII	e = 65
;			x = 78
;			i = 69
;			t = 74
;			s = 73	
lblExitDetect
	movlw	0x65
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblMenuFail
	movlw	0x78
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblMenuFail
	movlw	0x69
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblMenuFail
	movlw	0x74
	subwf	RXBuf3,W
	btfss	STATUS,Z
	goto	lblMenuFail
	movlw	0x04			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblExitSDetect

	call	subReturn
	call	subDoneDisplay
	call	subReturn
	goto	lblMenuExit

lblExitSDetect
	movlw	0x73
	subwf	RXBuf4,W
	btfss	STATUS,Z
	goto	lblMenuFail
	movlw	0x05			;Check if pointer is at correct location
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblMenuFail

;Save settings to EEPROM
	movlw	ADRSpdx
	movwf	TempEE
	movf	SpeedX, W
	movwf	TempEED
	call	WriteEEPROM

	movlw	ADRSpdy
	movwf	TempEE
	movf	SpeedY, W
	movwf	TempEED
	call	WriteEEPROM

	movlw	ADRSpdz
	movwf	TempEE
	movf	SpeedClaw, W
	movwf	TempEED
	call	WriteEEPROM

	movlw	ADRSTmrX
	movwf	TempEE
	movf	SerialTimerXSet, W
	movwf	TempEED
	call	WriteEEPROM

	movlw	ADRSTmrY
	movwf	TempEE
	movf	SerialTimerYSet, W
	movwf	TempEED
	call	WriteEEPROM

	movlw	ADRMode
	movwf	TempEE
	btfsc	TestMode
	goto	lblSetTestModeEEPROM
	btfsc	PopMode
	goto	lblSetPopModeEEPROM
	movlw	0x00
	goto	lblSaveModeEEPROM

lblSetTestModeEEPROM
	movlw	0x01
	goto	lblSaveModeEEPROM

lblSetPopModeEEPROM
	movlw	0x04

lblSaveModeEEPROM
	movwf	TempEED
	call	WriteEEPROM

	movlw	ADRTimz
	movwf	TempEE
	movf	TimeZ, W
	movwf	TempEED
	call	WriteEEPROM

	call	subReturn
	call	subDoneDisplay
	call	subReturn
	goto	lblMenuExit

lblMenuFail
	call	subReturn
	call	subFailDisplay
	call	subReturn
	call	subWaitDisplay

lblMenu1Reset
	call	subClearBuffer
	bcf		EnterDetected
	goto	lblEnterMenu1

lblMenuExit
	call	subClearBuffer
	bcf		ProgramMode		;Since the menu has been kicked out
							; drop out of programming mode
	bcf		EnterDetected	;Allow for data to be received again
	goto	Main

;****************************************
;	End Enter/Program Mode
;****************************************

;****************************************
;	Stage 1:  Open claw and drop claw to floor for 
;				predetermined amount of time
;****************************************
;claw drop button press has been detected, start pickup toy routine
lblClawOp
	movlw	B'00000101'		;Enable LowerClaw and keep ButtonPush active
	movwf	ClawStat
	bcf		Claw_Close		;make sure claw is open
	
;TimeZ will be multiplied by 5 for timing of the claw drop

	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec

;****************************************
;	Stage 2:  Close Claw for 
;				predetermined amount of time
;****************************************
	movlw	B'00000001'		;clear all of ClawStat and keep ButtonPush active
	movwf	ClawStat
	bsf		Claw_Close		;Close the claw

	movlw	B'01111111'
	call	Delay2Sec
	movlw	B'01111111'
	call	Delay2Sec

;****************************************
;	Stage 3:  Raise Claw until claw limit
;				switch is depressed
;****************************************
	movlw	B'00000011'		;Enable RaiseClaw and keep ButtonPush active
	movwf	ClawStat	

lblRaise
	btfsc	ClawUpLim
	goto	lblRaise

;****************************************
;	Stage 4:  Home the Claw Machine
;****************************************
	movlw	B'00001001'		;Enable Home and keep ButtonPush active
	movwf	ClawStat

lblHomeClaw
	btfsc	XLimMinus		;Need to add debounce on this circuit?  No problems with noise yet. 1/7/14
	goto	lblHomeClaw
	btfsc	YLimMinus	
	goto	lblHomeClaw

;****************************************
;	Stage 5:  Open the claw to drop prize
;****************************************
	movlw	B'00000001'		;clear all of ClawStat except ButtonPush
	movwf	ClawStat
	btfsc	PopMode			;Test if in PopMode
	goto	lblStageFivePopMode
	bcf		Claw_Close		;open claw
	movlw	B'01111111'
	call	Delay2Sec
	movlw	B'01111111'
	call	Delay2Sec
	goto	lblResetStage

;****************************************
;	Stage 5/Pop Mode:  Lower claw down
;	 then open claw. Retract claw.
;****************************************
;Lower Claw
lblStageFivePopMode
	movlw	B'01111111'		;Delay for claw to stop oscillating
	call	Delay2Sec
	movlw	B'01111111'
	call	Delay2Sec

	movlw	B'00000101'		;Enable LowerClaw and keep ButtonPush active
	movwf	ClawStat
	
;TimeZ will be multiplied by 5 for timing of the claw drop

	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec
	movf	TimeZ,W
	call	Delay2Sec
	
;****************************************
;	Stage 6/Pop Mode:  Open Claw for 
;		predetermined amount of time
;****************************************
	movlw	B'00000001'		;clear all of ClawStat and keep ButtonPush active
	movwf	ClawStat
	bcf		Claw_Close		;Open the claw

	movlw	B'01111111'
	call	Delay2Sec
	movlw	B'01111111'
	call	Delay2Sec

;****************************************
;	Stage 7/Pop Mode:  Raise Claw until claw limit
;				switch is depressed
;****************************************
	movlw	B'00000011'		;Enable RaiseClaw and keep ButtonPush active
	movwf	ClawStat	

lblPopRaise
	btfsc	ClawUpLim
	goto	lblPopRaise

	movlw	B'01111111'		;Delay for short period of time
	call	Delay2Sec
	movlw	B'01111111'
	call	Delay2Sec

;****************************************
;	Reset Stage:  Reset claw routine
;****************************************
lblResetStage
;at end of claw home routine, then re-enable port b interrupt
	movlw	B'00010000'		;Enable Joystick control 'EnAxis' and zero out to beginning state
	movwf	ClawStat
	bcf		INTCON,1		;Clear RB0 Interrupt Flag
	bsf		INTCON,4		;enable PORTB,0 interrupt (button press)
	goto	Main			;Restart process over again

;****************************************
;	End of Claw Mode Game Operation
;****************************************

;****************************************
;
;	HomeAll subroutine
;
;	Variables affected:
;		ClawStat		Will be zeroed out: 
;						ButtonPush 		= 0
;						RaiseClaw 		= 0
;						LowerClaw 		= 0
;						Home 			= 0
;						EnAxis 			= 0
;						ButtonDbnc 		= 0
;						Test Mode 		= 0
;						Program Mode 	= 0
;						 and then
;						 EnAxis will be enabled when 
;						 machine has finished homing
;****************************************
;Home will bring claw up and to front left
HomeAll

;open claw and retract claw up
	bcf		Claw_Close
	bsf		RaiseClaw

	btfsc	ClawUpLim
	goto	HomeAll

	movf	ClawStat,W		;Clear out status but allow test to pass through
	andlw	B'10000000'
	movwf	ClawStat

;Force claw to near left
	bcf		YMotDir			;push claw front
	bcf		XMotDir			;push claw left
	bsf		Home			;enable homing of axis

HomeXY
	btfsc	XLimMinus
	goto	HomeXY

	btfsc	YLimMinus	
	goto	HomeXY

	movf	ClawStat,W		;Clear out status but allow test to pass through
	andlw	B'10000000'
	movwf	ClawStat
	
	bsf		EnAxis			;Enable joystick control
	return

;************************************************
;
;		Test Mode button pushed subroutines
;
;	subTestXConMin = Debounce/Detect Claw Opening
;	subTestXConPlus = Debounce/Detect Claw Closing
;	subTestYConMin = Debounce/Detect Raise Claw
;	subTestYConPlus = Debounce/Detect Lower Claw
;
;************************************************

subTestXConMin
	movlw	0x7F
	call	Delay20ms		;A form of debounce
	btfsc	XControlMin
	return
	bcf		Claw_Close
	return

subTestXConPlus
	movlw	0x7F
	call	Delay20ms		;A form of debounce
	btfsc	XControlPlus
	return
	bsf		Claw_Close
	return

subTestYConMin
	movlw	0x7F
	call	Delay20ms		;A form of debounce
	btfsc	YControlMin
	return
	bcf		LowerClaw
	bsf		RaiseClaw
	return

subTestYConPlus
	movlw	0x7F
	call	Delay20ms		;A form of debounce
	btfsc	YControlPlus
	return
	bsf		LowerClaw
	bcf		RaiseClaw
	return

;*************************************************
;	Clear Buffer
;	 Clears out the receive buffer
;	 Use after a menu command is correctly 
;	 received in
;	If there is data that comes in during this
;	 time, there is potential that a value
;	 just saved will be erased, but user
;	 will not know this happened
;*************************************************

subClearBuffer

	clrf	RXBuf0
	clrf	RXBuf1
	clrf	RXBuf2
	clrf	RXBuf3
	clrf	RXBuf4
	clrf	RXBuf5
	clrf	RXBuf6
	clrf	RXBuf7
	clrf	RXBufPointer
	return

;*************************************************
;	Return
;	 Send CR LF to device to act as a return
;*************************************************
subReturn
	movlw	0x7F
	call	Delay20ms
lblReturnPause
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblReturnPause
	movlw	0x0D		;CR
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblReturn
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblReturn
	movlw	0x0A		;LF
	movwf	TXREG
	return

;*************************************************
;	Wait 
;	 Send > to user to represent a command can
;	 be entered
;*************************************************
subWaitDisplay
	movlw	0x7F
	call	Delay20ms
lblWaitDisplayPause
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblWaitDisplayPause
	movlw	0x3E		;>
	movwf	TXREG
	return

;*************************************************
;	Done Display
;	 Send DONE to the user
;	 ASCII 	D = 44
;			O = 4F
;			N = 4E
;			E = 45
;*************************************************
subDoneDisplay
	movlw	0x7F
	call	Delay20ms
lblDoneDisplayPause
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblDoneDisplayPause
	movlw	0x44		;D
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblDone1
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblDone1
	movlw	0x4F		;O
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblDone2
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblDone2
	movlw	0x4E		;N
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblDone3
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblDone3
	movlw	0x45		;E
	movwf	TXREG

	return

;*************************************************
;	Fail Display
;	 Send FAIL to the user
;	 ASCII 	F = 46
;			A = 41
;			I = 49
;			L = 4C
;*************************************************
subFailDisplay
	movlw	0x7F
	call	Delay20ms
lblFailDisplayPause
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblFailDisplayPause
	movlw	0x46		;F
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblFail1
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblFail1
	movlw	0x41		;A
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblFail2
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblFail2
	movlw	0x49		;I
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblFail3
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblFail3
	movlw	0x4C		;L
	movwf	TXREG

	return

;*************************************************
;	Enter:_ Display
;	 Send ENTER:<space> to the user
;	 ASCII 	E = 45
;			N = 4E
;			T = 54
;			E = 45
;			R = 52
;			: = 3A
;		space = 20
;*************************************************
subEnterDisplay
	movlw	0x7F
	call	Delay20ms
lblEnterDisplayPause
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblEnterDisplayPause
	movlw	0x45		;E
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblEnter1
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblEnter1
	movlw	0x4E		;N
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblEnter2
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblEnter2
	movlw	0x54		;T
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblEnter3
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblEnter3
	movlw	0x45		;E
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblEnter4
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblEnter4
	movlw	0x52		;R
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblEnter5
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblEnter5
	movlw	0x3A		;:
	movwf	TXREG
	movlw	0x7F
	call	Delay20ms

lblEnter6
	btfss	PIR1,4		;Check if it is ok to transmit
	goto	lblEnter6
	movlw	0x20		;<space>
	movwf	TXREG

	return


;*********************************
;EEPROM File Read subroutine
;	TempEE points to desired 
;	EEPROM address
;	Value placed into WREG
;*********************************
ReadEEPROM
	BSF		STATUS, RP1		;
	BCF		STATUS, RP0		;Bank 2
	MOVF	TempEE, W			;Write address
	MOVWF	EEADR			;to read from
	BSF		STATUS, RP0		;Bank 3
	BCF		EECON1, EEPGD	;Point to Data memory
	BSF		EECON1, RD		;Start read operation
	BCF		STATUS, RP0		;Bank 2
	MOVF	EEDATA, W		;W = EEDATA
	BCF		STATUS, RP1		;Bank 0
	return

;*********************************
;EEPROM File Write subroutine
;	TempEE points to desired 
;	 EEPROM address
;	TempEED is filled with Data
;	 to be saved
;*********************************
WriteEEPROM
	BSF		STATUS, RP1		;
	BSF		STATUS, RP0		;Bank 3
	BTFSC	EECON1, WR		;Wait for
	GOTO	$-1				;write to finish
	BCF		STATUS, RP0		;Bank 2
	MOVF	TempEE, W			;Address to
	MOVWF	EEADR			;write to
	MOVF	TempEED, W		;Data to
	MOVWF	EEDATA			;write
	BSF		STATUS, RP0		;Bank 3
	BCF		EECON1, EEPGD	;Point to Data memory
	BSF		EECON1, WREN	;Enable writes
							;Only disable interrupts
	MOVLW	0x55			;Write 55h to
	MOVWF	EECON2			;EECON2
	MOVLW	0xAA			;Write AAh to
	MOVWF	EECON2			;EECON2
	BSF		EECON1, WR		;Start write operation
							;Only enable interrupts
	BCF		EECON1, WREN	;Disable writes
	BCF		STATUS, RP1		;
	BCF		STATUS, RP0		;Bank 0
	return

;*********************************
;	Delay2Sec subroutine is used
;		for general purpose
;		stalling
;	move timer value into WREG
;*********************************
Delay2Sec
;	movlw	B'01111111'
	movwf	Delay2
Delay2SecA	
	movf	Delay2,w
	call	Delay20ms
	decfsz	Delay2
	goto	Delay2SecA
	return

;*********************************
;	Delay20ms subroutine 
;*********************************
Delay20ms
	movlw	.26				;Set outer loop to 26
	movwf	Outer
D1
	clrf	Inner			;Clear inner loop
D2
	decfsz	Inner		
	goto	D2
	decfsz	Outer
	goto	D1
	return

;********************************************
;	Extract RX Buf Value
;	 Pulls numeric value from RXBuf0, RXBuf1, RXBuf2
;	 and places value into TempSpeed
;	Checks if value is numeric and places different
;	 binary weight onto value read
;	Value can range from 1 to 255
;	ASCII	0 = 30
;			9 = 39
;	Variables:
;		TempSpeed, RXBuf0-9
;	Notes:
;		This does not detect if the buffer is
;		4 or more digits long, instead it 
;		will zero out
;		A value of 0 represents a failed
;		data set point.
;********************************************

subExtractRXBufValue
	clrf	TempSpeed		;Start by clearing out passing variable 

;Check least significant digit first
	movlw	0x30
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd1
	goto	lblSpdTens

lblSpd1
	movlw	0x31			;Add 1
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd2
	movlw	D'1'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd2
	movlw	0x32			;Add 2
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd3
	movlw	D'2'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd3
	movlw	0x33			;Add 3
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd4
	movlw	D'3'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd4
	movlw	0x34			;Add 4
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd5
	movlw	D'4'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd5
	movlw	0x35			;Add 5
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd6
	movlw	D'5'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd6
	movlw	0x36			;Add 6
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd7
	movlw	D'6'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd7
	movlw	0x37			;Add 7
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd8
	movlw	D'7'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd8
	movlw	0x38			;Add 8
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpd9
	movlw	D'8'
	addwf	TempSpeed,F
	goto	lblSpdTens

lblSpd9
	movlw	0x39			;Add 9
	subwf	RXBuf2,W
	btfss	STATUS,Z
	goto	lblSpdFail		;Kick out due to invalid data
	movlw	D'9'
	addwf	TempSpeed,F

;Check second digit next
lblSpdTens
	movlw	0x30
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd10
	goto	lblSpdHundreds

lblSpd10
	movlw	0x31			;Add 10
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd20
	movlw	D'10'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd20
	movlw	0x32			;Add 20
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd30
	movlw	D'20'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd30
	movlw	0x33			;Add 30
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd40
	movlw	D'30'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd40
	movlw	0x34			;Add 40
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd50
	movlw	D'40'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd50
	movlw	0x35			;Add 50
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd60
	movlw	D'50'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd60
	movlw	0x36			;Add 60
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd70
	movlw	D'60'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd70
	movlw	0x37			;Add 70
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd80
	movlw	D'70'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd80
	movlw	0x38			;Add 80
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpd90
	movlw	D'80'
	addwf	TempSpeed,F
	goto	lblSpdHundreds

lblSpd90
	movlw	0x39			;Add 90
	subwf	RXBuf1,W
	btfss	STATUS,Z
	goto	lblSpdFail		;Kick out due to invalid data
	movlw	D'90'
	addwf	TempSpeed,F

;Check most significant digit first
lblSpdHundreds
	movlw	0x30
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblSpd100
	goto	lblSpdExit

lblSpd100
	movlw	0x31			;Add 100
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblSpd200
	movlw	D'100'
	addwf	TempSpeed,F
	goto	lblSpdExit

lblSpd200
	movlw	0x32			;Add 200
	subwf	RXBuf0,W
	btfss	STATUS,Z
	goto	lblSpdFail		;Kick out of detection due to invalid data
	movlw	D'200'
	addwf	TempSpeed,F
	btfss	STATUS,C		;If carry happens, then the number is too large
	goto	lblSpdExit

lblSpdFail
	clrf	TempSpeed		;zero out speed due to invalid number

lblSpdExit
	return

;****************************************
;	Interrupt Subroutine
;	  
;****************************************
org 0x0500
IntService
	MOVWF	WTemp			;Copy W to TEMP register
	SWAPF	STATUS,W		;Swap status to be saved into W
	CLRF	STATUS			;bank 0, regardless of current bank, Clears IRP,RP1,RP0
	MOVWF	StatTemp		;Save status to bank zero STATUS_TEMP register
	MOVF	PCLATH, W		;Only required if using pages 1, 2 and/or 3
	MOVWF	PCLTemp			;Save PCLATH into W
	CLRF	PCLATH			;Page zero, regardless of current page (should this really happen?)
	BCF		STATUS, RP1		;
	BCF		STATUS, RP0		;Bank 0 Select

;*******************************************************
;	RS232 data check
;		This section of the interrupt will fill the 
;		 RXBuf full of data (8 bytes) that can be used for 
;		 commands
;		Saves data to RXBuf
;		Detects if Enter has been pressed
;		Not designed to check if the commands are correct or 
;		 to push chip into program mode.  Only meant to 
;		 quickly save the receive buffer.
;		Have main program detect if the command is good
;		 and act on it after an enter press
;		This will only WRITE TO the RXBuf, it will not 
;		 erase the data.
;		Added quick command control from 232 port: 
;				I = Forward (0x49)
;				K = Backward (0x4B)
;				L = Right (0x4C)
;				J = Left (0x4A)
;				<spacbar> = Push button (0x20)
;
;	Possible issues:
;		If there is multiple bytes in the RXREG (2)
;		 then this section will run multiple times
;		If the buffer is full, then this will run faster
;		 than if there was data to be saved
;		May have to add code to check the TXREG flags
;		 making sure it is ready for another byte
;	Variables:
;		RXTempBuf, RXBufPointer, RXBufx (all 8 numbers)
;*******************************************************

ReceiveChk
	btfss	PIR1,RCIF		;Check if RXREG has data.
	goto	lblCheckProgramMode

	movf	RCREG,W			;Save value read in to temporary variable
	movwf	RXTempBuf

;Check if the value is to quick control the machine:
; Commands recognized:
;	I = Forward (0x49)
;	K = Backward (0x4B)
;	L = Right (0x4C)
;	J = Left (0x4A)
;	<spacebar> = Push button (0x20)
; When a command is recognized, then fill a serial timer that will count
;	down to zero.  If it is above zero, allow for the machine to move
;	If it zeroes then reset the bit that will be used for controlling the machine
	movlw	0x49			;ASCII value for I
	subwf	RXTempBuf,W
	btfss	STATUS,Z
	goto	lblSerialBackCheck

	movf	SerialTimerYSet,W	;Sets the amount of steps the Y axis will go 
	movwf	SerialTimerY	; before movement stops.
	bsf		SerialForward
	bcf		SerialBack
	goto	lblCheckProgramMode

lblSerialBackCheck
	movlw	0x4B			;ASCII value for K
	subwf	RXTempBuf,W
	btfss	STATUS,Z
	goto	lblSerialLeftCheck

	movf	SerialTimerYSet,W	;Sets the amount of steps the Y axis will go 
	movwf	SerialTimerY	; before movement stops.
	bsf		SerialBack
	bcf		SerialForward
	goto	lblCheckProgramMode

lblSerialLeftCheck
	movlw	0x4A			;ASCII value for J
	subwf	RXTempBuf,W
	btfss	STATUS,Z
	goto	lblSerialRightCheck

	movf	SerialTimerXSet,W	;Sets the amount of steps the X axis will go 
	movwf	SerialTimerX	; before movement stops.
	bsf		SerialLeft
	bcf		SerialRight
	goto	lblCheckProgramMode

lblSerialRightCheck
	movlw	0x4C			;ASCII value for L
	subwf	RXTempBuf,W
	btfss	STATUS,Z
	goto	lblSerialButtonCheck

	movf	SerialTimerXSet,W	;Sets the amount of steps the X axis will go 
	movwf	SerialTimerX	; before movement stops.
	bsf		SerialRight
	bcf		SerialLeft
	goto	lblCheckProgramMode

lblSerialButtonCheck
	movlw	0x20
	subwf	RXTempBuf,W
	btfss	STATUS,Z
	goto	lblEnterCheck

	btfsc	ButtonPush		;Avoid causing a second button push, if already detected
	goto	lblCheckProgramMode

	bsf		SerialButton	;may need to create OR and AND function to set at once.
	bcf		SerialForward
	bcf		SerialBack
	bcf		SerialRight
	bcf		SerialLeft
	goto	lblCheckProgramMode

lblEnterCheck
;Check if the value read in is either CR or LF.  If it is not, save value to
;	buffer.  If it is, then let Main Program know that enter was pushed.
	movlw	0x0A			;ASCII value for LF
	subwf	RXTempBuf,W
	btfsc	STATUS,Z
	goto	lblRXEnterSet	;If LF is detected, then set the enter detected bit

	movlw	0x0D			;ASCII value for CR
	subwf	RXTempBuf,W
	btfsc	STATUS,Z
	goto	lblRXEnterSet	;If CR is detected, then set the enter detected bit

;Check if the value is the escape button.  This can be used to exit out of commands.
;	Do not save this value to the buffer
	movlw	0x1B
	subwf	RXTempBuf,W
	btfsc	STATUS,Z
	goto	lblRXEscapeSet

;Check if backspace was used and backup pointer if it is not zero
	movlw	0x7F
	subwf	RXTempBuf,W
	btfsc	STATUS,Z
	goto	lblBackspace

;Check if enter was pushed.  If it was then do not allow the buffer to be filled up 
;	with more data,  Check to see if there are errors for the Receive
	btfsc	EnterDetected
	goto	lblCheckErrors

;Check if receive buffer is full
	movlw	0x08
	subwf	RXBufPointer,W
	btfsc	STATUS,Z
	goto	lblCheckErrors

;Fill up receive buffer with data input into serial port.
; Will increment up through the 8 buffers, filling data that
; comes in.
;Check which buffer location to place data.
	movf	RXBufPointer,W	;Check if the buffer pointer is empty
	btfss	STATUS,Z
	goto	lblRXBuf1Check
	movf	RXTempBuf,w
	movwf	RXBuf0
	movwf	TXREG			;Transmit out to user that the data was received
	goto	lblIncRXPointer

lblRXBuf1Check
	movlw	0x01
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblRXBuf2Check
	movf	RXTempBuf,W
	movwf	RXBuf1
	movwf	TXREG			;Transmit out to user that the data was received
	goto	lblIncRXPointer

lblRXBuf2Check
	movlw	0x02
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblRXBuf3Check
	movf	RXTempBuf,W
	movwf	RXBuf2
	movwf	TXREG			;Transmit out to user that the data was received
	goto	lblIncRXPointer

lblRXBuf3Check
	movlw	0x03
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblRXBuf4Check
	movf	RXTempBuf,W
	movwf	RXBuf3
	movwf	TXREG			;Transmit out to user that the data was received
	goto	lblIncRXPointer

lblRXBuf4Check
	movlw	0x04
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblRXBuf5Check
	movf	RXTempBuf,W
	movwf	RXBuf4
	movwf	TXREG			;Transmit out to user that the data was received
	goto	lblIncRXPointer

lblRXBuf5Check
	movlw	0x05
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblRXBuf6Check
	movf	RXTempBuf,W
	movwf	RXBuf5
	movwf	TXREG			;Transmit out to user that the data was received
	goto	lblIncRXPointer

lblRXBuf6Check
	movlw	0x06
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblRXBuf7Check
	movf	RXTempBuf,W
	movwf	RXBuf6
	movwf	TXREG			;Transmit out to user that the data was received
	goto	lblIncRXPointer

lblRXBuf7Check
	movlw	0x07
	subwf	RXBufPointer,W
	btfss	STATUS,Z
	goto	lblCheckErrors
	movf	RXTempBuf,W
	movwf	RXBuf7
	movwf	TXREG			;Transmit out to user that the data was received

lblIncRXPointer
	incf	RXBufPointer	;increment pointer to next available byte

lblCheckErrors
	btfss	RCSTA,OERR		;Check if there is multiple bytes in queue and a
							; possible missed byte, bank 0
	goto	lblCheckProgramMode

	bcf		RCSTA,CREN		;Reset Receive enable 
	bsf		RCSTA,CREN

	btfss	PIR1,RCIF		;Check if RXREG has another packet
	goto	lblCheckProgramMode
	goto	ReceiveChk		;Go through the saving of the byte a second time.
							; This will add some time to this interrupt and may skew
							; the stepping pulses for a short period.
lblRXEnterSet
	bsf		EnterDetected	;Save that enter has been detected
	movf	RXTempBuf,W
	movwf	TXREG
	goto	lblCheckProgramMode

lblRXEscapeSet
	bsf		EscapeDetected
	goto	lblCheckProgramMode

lblBackspace
	movf	RXBufPointer,W	;Check if zero
	btfsc	STATUS,Z
	goto	lblCheckProgramMode
	decf	RXBufPointer,1	;push pointer back, store back in register
	movf	RXTempBuf,W
	movwf	TXREG			;push delete back to interface

;*************************************************
;	End of Serial interrupt routine
;*************************************************

;Check if the system is in program mode.  If it is then do not
;	allow joystick to control machine
;	Reset interrupts as the rests will then be bypassed
;	Watch dog timer will be reset whenever there is 
;	 a portb, RX, or timer2 interrupt.  Could add
;	 further detection if needed

lblCheckProgramMode
	btfss	ProgramMode
	goto	BIntCheck
	clrwdt					;Clear out watch dog timer when in program mode
	bcf		INTCON,1		;Clear out RB0 flag
	bcf		PIR1,1			;Clear out Timer 2 Interrupt
	goto	IntExit

;End program mode bypass

BIntCheck
	btfss	INTCON,1		;Check for claw down button press
	goto	Timer2Chk

	btfsc	ButtonPush		;make sure we do not reset button press routine
	goto	Timer2Chk
	
	btfsc	ButtonDbnc		;Make sure we do not have a debounce check going on
	goto	Timer2Chk

;claw drop button press has been detected, start debounce check.	
	bcf		INTCON,4		;Disable PortB,0 interrupt
	bcf		INTCON,1		;Clear RB0 Interrupt Flag
	bcf		ButtonPush		;Reset Button Push just in case this was active
	bsf		ButtonDbnc		;Start to check if this is a real button press or noise
	movlw	0xFE			;load debounce bytes for debounce detect with the first one low
	movwf	lowDbnc
	movwf	hiDbnc

Timer2Chk
	btfss	PIR1,1			;Check Timer2 Flag
	goto	IntExit			; Exit Interrupt
	bcf		PIR1,1			;Reset Timer2 Flag

;Clear WDT every mS.  This will hopefully help to avoid any lockups from keeping the game
; going
;May need to add some sort of "health check" to see if things are ok before resetting the
; wdt.  Possibly check the state of the machine and verify that we are in a prescribed state
;Check for stable run state:
;	EnAxis = 1
;	B Interrupt is enabled
;	All other bits in ClawStat are zero.
;	If this state is not correct then do not reset watchdog timer.


;This may have to be changed if the Test Mode or Program Mode is active.
; If the Test Mode is Active, there might be a chance that the axis
; is enabled, but PORTB,0 interrupt is not



;Add different states that would allow for a WDT to be reset.

lblRunState
	btfsc	TestMode		;Check if system is in test mode.  If it is, bypass other tests
	goto	lblClrWDT		; and clear wdt.  Helps solve random program resets.

	btfss	EnAxis			;Check if axis is enabled
	goto	lblClrWDT

	btfss	INTCON,4		;If enabled, verify the button interrupt is active
	goto	lblDbncRout	

lblClrWDT
	clrwdt

;****************************************
;	Debounce Routine
;****************************************

lblDbncRout
	btfsc	SerialButton
	goto	lblSerialButtonPushedInterrupt

;Debounce routine will use 1 mS timer to check button push until there is no more bounce
	btfss	ButtonDbnc
	goto	lblStep
	
	rlf		lowDbnc			;shift debounce data left
	rlf		hiDbnc
	
	btfss	PORTB,0			;Check if we need to set/reset the first bit of the xxDbnc
	goto	lblDbncLow
	
	bsf		lowDbnc,0	
	goto	lblStep			;If we set the incoming bit, then the dbnc wont be zeroed out,
							;	go check stepper pulses
	
lblDbncLow
	bcf		lowDbnc,0

;Check if there is no more bounce
	movf	lowDbnc,1		;Check if all zero
	btfss	STATUS,Z
	goto	lblDbncFail
	
	movf	hiDbnc,1		;Check if all zero
	btfss	STATUS,Z
	goto	lblStep
	goto	lblButtonPushDetected

lblSerialButtonPushedInterrupt	
;claw drop button press has been detected, start debounce check.	
	bcf		INTCON,4		;Disable PortB,0 interrupt
	bcf		INTCON,1		;Clear RB0 Interrupt Flag
	bcf		SerialButton	;Reset Serial button push detection

lblButtonPushDetected
	;All zeros (3.174 mS of no bouncing (20 MHz crystal)).  Set ButtonPush bit and reset debounce
	bsf		ButtonPush
	bcf		ButtonDbnc
;	bcf		Claw_Close		;make sure claw is open -->This is opening the claw for test mode
							; Disabled for test mode.  Removing this did not affect Claw Game operation
	
	goto	lblStep
	
lblDbncFail
	movlw	0xFF			;Check if all ones, if they are then reset the button push routine
	subwf	lowDbnc,0		; and interrupt timer.
	btfss	STATUS,Z
	goto	lblStep
	
	movlw	0xFF
	subwf	hiDbnc,0
	btfss	STATUS,Z
	goto	lblStep

;Reset debounce and reset interrupt timer for button press
	bcf		ButtonDbnc		;Reset the button push/debounce functions
	bcf		ButtonPush
	bcf		INTCON,1		;Clear RB0 Interrupt Flag
	bsf		INTCON,4		;enable PORTB,0 interrupt (button press)

;****************************************
;	End of Debounce Routine
;****************************************
		
lblStep	
;Reset Step bits for all axis.  Stepper Driver needs to see only 1 uS step
;	pulse.  Timer2 is setup for .1984 mS timing pulses
	bcf		XMotStp
	bcf		YMotStp
	bcf		ClawStep                                                                                                                                        

;**********************************************************
;	Claw Raise/Lower routine
;		Use to raise or lower the claw.
;		Claw uses stepper motor
;		Exits out of interrupts after raise or lower due to 
;		x and y axis being locked out anyways
;	Variables used:
;		RaiseClaw
;		LowerClaw
;	Set a value of '1' to either RaiseClaw or LowerClaw
;	 to activate this section.  If both are set to '1'
;	 then it will raise claw until RaiseClaw goes low.
;	This is not affected by variable EnAxis.  
;**********************************************************

lblRaiseClaw
	btfss	RaiseClaw
	goto	lblLowerClaw

	bcf		ClawDir			;Raise claw up
	btfss	ClawUpLim		;Check if claw is up against limit
	goto	IntExit			;Exit out of interrupts
	incf	TimerClaw		;increment timer for speed of pulses
	movf	TimerClaw,0		;move timer value into WREG
	subwf	SpeedClaw,0		;Compare to speed that is set
	btfss	STATUS,2
	goto	IntExit			;Exit out of interrupts
	clrf	TimerClaw		;Clear Timer
	bsf		ClawStep		;Send pulse to driver
	goto	IntExit			;Exit out of interrupts


lblLowerClaw
	btfss	LowerClaw
	goto	lblXHome
	
	bsf		ClawDir			;Lower claw down
	incf	TimerClaw		;increment timer for speed of pulses
	movf	TimerClaw,0		;move timer value into WREG
	subwf	SpeedClaw,0		;Compare to speed that is set
	btfss	STATUS,2
	goto	IntExit			;Exit out of interrupts
	clrf	TimerClaw		;Clear Timer
	bsf		ClawStep		;Send pulse to driver
	goto	IntExit			;Exit out of interrupts


;**********************************************************
;	No Joystick control routines
;		If user has pushed the claw down button, 
;		then use these routines to not allow full control
;		Exits out of interrupt routines if they are used
;	Variables used:
;		Home
;**********************************************************

lblXHome
	btfss	Home
	goto	lblXConMin
	
	bcf		XMotDir			;push claw to drop corner (-)
	btfss	XLimMinus		;Check if unit is up against limit
	goto	lblYHome		;Up against limit, jump to y axis

	incf	TimerX			;Increment timer for speed of pulses
	movf	TimerX,0		;Move timer value into WREG
	subwf	SpeedX,0		;Compare to speed that is set
	btfss	STATUS,2
	goto	lblYHome		;jump to y axis
	clrf	TimerX			;Clear Timer
	bsf		XMotStp			;Send pulse to driver

lblYHome
	bcf		YMotDir			;push claw to drop corner (-)
	btfss	YLimMinus		;Check if unit is up against limit
	goto	IntExit			;Up against limit

	incf	TimerY			;Increment timer for speed of pulses
	movf	TimerY,0		;Move timer value into WREG
	subwf	SpeedY,0		;Compare to speed that is set
	btfss	STATUS,2
	goto	IntExit			;jump to interrupt exit
	clrf	TimerY			;Clear Timer
	bsf		YMotStp			;Send pulse to driver
	goto	IntExit			;jump to interrupt exit

;**********************************************************
;	Joystick full control routines
;		If user has not pushed the claw down button, 
;		 then use these routines to allow for full control
;		The serial port can also be used to control the
;		 X and Y axis of the claw machine.  It will
;		 force movement as long as the appropriate serial
;		 control bit is used and SerialTimerX or
;		 SerialTimerY has not counted down to zero
;	Variables used:
;		EnAxis
;	Control Inputs used:
;		XControlMin
;		XControlPlus
;		YControlMin
;		YControlPlus
;	Serial Control Inputs used:
;		SerialForward
;		SerialBack
;		SerialLeft
;		SerialRight
;**********************************************************

lblXConMin
	btfss	EnAxis			;Check if joystick can be used for control
	goto	IntExit

	btfsc	SerialLeft
	goto	lblXConMinActive
	btfsc	XControlMin
	goto	lblXConPlus

lblXConMinActive
	bcf		XMotDir			;Change motor directions
	btfss	XLimMinus		;Check if unit is up against limit
	goto	lblYConMin		;Up against limit and joystick is in minus location
							;May allow serial timer to be retained until it counts down

	incf	TimerX			;Increment timer for speed of pulses
	movf	TimerX,0		;Move timer value into WREG
	subwf	SpeedX,0		;Compare to speed that is set in homing sequence
	btfss	STATUS,2
	goto	lblYConMin		;jump to y axis

;If the SerialTimerX has timed down, then stop movement initiated by serial
	movf	SerialTimerX,W	;Check if SerialTimerX is zero
	btfsc	STATUS,Z
	goto	lblXConMinStep	;jump to stepping axis

	decfsz	SerialTimerX
	goto	lblXConMinStep
	bcf		SerialLeft

lblXConMinStep
	clrf	TimerX			;Clear Timer
	bsf		XMotStp			;Send pulse to driver
	goto	lblYConMin

lblXConPlus
	btfsc	SerialRight
	goto	lblXConPlusActive
	btfsc	XControlPlus
	goto	lblYConMin

lblXConPlusActive
	bsf		XMotDir			;Change motor directions
	btfss	XLimPlus		;Check if unit is up against limit
	goto	lblYConMin		;Up against limit and joystick is in plus location

	incf	TimerX			;Increment timer for speed of pulses
	movf	TimerX,0		;Move timer value into WREG
	subwf	SpeedX,0		;Compare to speed that is set in homing sequence
	btfss	STATUS,2
	goto	lblYConMin		;jump to y axis

;If the SerialTimerX has timed down, then stop movement initiated by serial
	movf	SerialTimerX,W	;Check if SerialTimerX is zero
	btfsc	STATUS,Z
	goto	lblXConPlusStep	;jump to stepping axis

	decfsz	SerialTimerX
	goto	lblXConPlusStep
	bcf		SerialRight

lblXConPlusStep
	clrf	TimerX			;Clear Timer
	bsf		XMotStp			;Send pulse to driver

lblYConMin
	btfsc	SerialBack
	goto	lblYConMinActive
	btfsc	YControlMin
	goto	lblYConPlus

lblYConMinActive
	bcf		YMotDir			;Change motor directions
	btfss	YLimMinus		;Check if unit is up against limit
	goto	IntExit			;Up against limit and joystick is in plus location

	incf	TimerY			;Increment timer for speed of pulses
	movf	TimerY,0		;Move timer value into WREG
	subwf	SpeedY,0		;Compare to speed that is set in homing sequence
	btfss	STATUS,2
	goto	IntExit			;jump to interrupt exit

;If the SerialTimerY has timed down, then stop movement initiated by serial
	movf	SerialTimerY,W	;Check if SerialTimerY is zero
	btfsc	STATUS,Z
	goto	lblYConMinStep	;jump to stepping axis

	decfsz	SerialTimerY
	goto	lblYConMinStep
	bcf		SerialBack

lblYConMinStep
	clrf	TimerY			;Clear Timer
	bsf		YMotStp			;Send pulse to driver
	goto	IntExit

lblYConPlus
	btfsc	SerialForward
	goto	lblYConPlusActive
	btfsc	YControlPlus
	goto	IntExit

lblYConPlusActive
	bsf		YMotDir			;Change motor directions
	btfss	YLimPlus		;Check if unit is up against limit
	goto	IntExit			;Up against limit and joystick is in plus location

	incf	TimerY			;Increment timer for speed of pulses
	movf	TimerY,0		;Move timer value into WREG
	subwf	SpeedY,0		;Compare to speed that is set in homing sequence
	btfss	STATUS,2
	goto	IntExit			;jump to interrupt exit

;If the SerialTimerY has timed down, then stop movement initiated by serial
	movf	SerialTimerY,W	;Check if SerialTimerX is zero
	btfsc	STATUS,Z
	goto	lblYConPlusStep	;jump to stepping axis

	decfsz	SerialTimerY
	goto	lblYConPlusStep
	bcf		SerialForward

lblYConPlusStep
	clrf	TimerY			;Clear Timer
	bsf		YMotStp			;Send pulse to driver
	goto	IntExit

;Shut Down Timer2 and interrupt
;	bsf		STATUS,RP0		;Bank 1 Select
;	bcf		PIE1,1			;Disable Timer2 Interrupt
;	bcf		STATUS,RP0		;Bank 0 Select
;	bcf		T2CON,2			;Turn Timer2 off
;	bcf		INTCON,PEIE		;Disable Peripheral Interrupts
;	bcf		INTCON,GIE		;Global Interrupt Disable	
;End Setup

IntExit
	bcf		STATUS,RP0		;Bank 0 Select
	MOVF	PCLTemp, W		;Restore PCLATH
	MOVWF	PCLATH			;Move W into PCLATH
	SWAPF	StatTemp,W		;Swap STATUS_TEMP register into W
							;(sets bank to original state)
	MOVWF	STATUS			;Move W into STATUS register
	SWAPF	WTemp,F			;Swap W_TEMP
	SWAPF	WTemp,W			;Swap W_TEMP into W	
	retfie
;****************************************
;	End Interrupt Routine
;****************************************



	end
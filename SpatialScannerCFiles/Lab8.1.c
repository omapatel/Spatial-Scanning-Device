/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/

/*	Demo 2 - 2DX3 (April 5th, 2024)

		Modified code by Om Patel, patelo7, 400378073
*/

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h>

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3

  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

// Flash LED D4 (D4 is connected to PF0)
void FlashLEDF(int count) {
		while(count--) {
			GPIO_PORTF_DATA_R ^= 0b00000001; 								// hello world!
			SysTick_Wait10ms(20);														// 0.2s delay
			GPIO_PORTF_DATA_R ^= 0b00000001;			
		}
}

//Flash LED D2 (D2 is connected to PN0)
void ToggleLEDN(int count) {
		while(count--) {
			GPIO_PORTN_DATA_R ^= 0b00000001; 			
		}
}

void FlashLEDN(int count){
	while(count--){
		GPIO_PORTN_DATA_R ^= 0b00000001;
		SysTick_Wait10ms(10);														// 0.1s delay
		GPIO_PORTN_DATA_R ^= 0b00000001;                // hello world!
	}
}

// Give clock to Port J and initalize PJ1 as Digital Input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000FF;	 								//  Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//  Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor on PJ1
}

// Initialize onboard LED D2 and PN2
void PortN_Init(void){
	//Use PortN onboard LED
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// Activate clock for Port N
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x05;        								// Make PN0 output (Built-in LED: D2 (PN0))
  GPIO_PORTN_AFSEL_R &= ~0x05;     								// Disable alt funct on PN0
  GPIO_PORTN_DEN_R |= 0x05;        								// Enable digital I/O on PN0
																									
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x05;     								// Disable analog functionality on PN0
	FlashLEDN(1);
	return;
}

void PortM_Init(void){
	//Use PortM motor control
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// Activate clock for Port M
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x0F;        								// Make PM0-3 output
  GPIO_PORTM_AFSEL_R &= ~0x0F;     								// Disable alt funct on PM0-3
  GPIO_PORTM_DEN_R |= 0x0F;        								// Enable digital I/O on PM0-3
																									
  //GPIO_PORTL_PCTL_R = (GPIO_PORTL_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTM_AMSEL_R &= ~0x0F;     								// Disable analog functionality on PM0-3
	return;
}

// Give clock to Port F and initalize PF1 as Digital Input GPIO
void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;					// Activate clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0x01;												  // Make PF0 output
	GPIO_PORTF_AFSEL_R &= ~0x01;     								  // Disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x01;     										// Enable digital I/O on PF0
	
	GPIO_PORTF_AMSEL_R &= ~0x01;											//  Disable analog functionality on PF0		
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
		GPIO_PORTJ_IS_R = 0x0;      						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0x0;     						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0x0;    						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x03;       					// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x03;      					// 					Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R =   0x00080000;          					// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000; 									// (Step 4) Set interrupt priority to 5

		EnableInt();																	// (Step 3) Enable Global Interrupt. lets go!
}

//	(Step 5) IRQ Handler (Interrupt Service Routine).  
//  				This must be included and match interrupt naming convention in startup_msp432e401y_uvision.s 
//					(Note - not the same as Valvano textbook).
volatile int pressed = 0;
volatile int pressed2 = 1;
volatile int flag = 0;
volatile int flag2 = 0;
void GPIOJ_IRQHandler(void){
	FlashLEDF(1);										
	if (GPIO_PORTJ_RIS_R == 0x01){ //PJ0 to start spinning for the first time or terminate
		if (flag2 == 0)
		{
			FlashLEDF(1);
			flag2 = 1;
			flag = 1;
			pressed = 1;
		}
		else
		{
			flag2 = 0;
			pressed2 = 0;
		}
		GPIO_PORTJ_ICR_R = 0x01;
	}
	else if (GPIO_PORTJ_RIS_R == 0x02){ //PJ1 to signal to spin again
		if (pressed == 0){ //checks to see if the button has already been pressed, if it hasn't yet then that means the motor is not spinning, signaling to turn it on
			flag = 1;
			pressed = 1;
		}
		else if (flag == 1){ //if the button has already been pressed, then this is the signal to turn it off
			flag = 0;
			pressed = 0;
		}
		GPIO_PORTJ_ICR_R = 0x02;     					// Acknowledge flag by setting proper bit in ICR register
	}
}

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

void Milestone1(void)
{

	uint8_t byteData,sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
	uint16_t wordData;

	status = VL53L1_RdByte(dev, 0x010F, &byteData); //for model ID (0xEA)
	SysTick_Wait10ms(10);
	
	sprintf(printf_buffer,"Model ID: 0x%x\r\n", byteData);
	UART_printf(printf_buffer);
	
  status = VL53L1_RdByte(dev, 0x0110, &byteData); //for module type (0xCC)
	SysTick_Wait10ms(10);
	
	sprintf(printf_buffer,"Module Type: 0x%x\r\n", byteData);
	UART_printf(printf_buffer);
	
  status = VL53L1_RdWord(dev, 0x010F, &wordData); //for both model ID and type
	SysTick_Wait10ms(10);
	
	sprintf(printf_buffer,"Model ID and Type: %x\r\n", wordData);
	UART_printf(printf_buffer);
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
volatile int returnHome = 0;
volatile int skipping = 0;
void home(){
	uint32_t delay = 200;
	for(int i=0; i<returnHome; i++){												// 512 * 4 (full step) = 2048 steps
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10us(delay);											// using us instead of ms
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10us(delay);
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10us(delay);
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10us(delay);
	}
	if (skipping == 1){
		sprintf(printf_buffer,"");
		char skip[4];
		sprintf(skip, "sk\r");
		UART_printf(skip);
		status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
	}
	returnHome = 0;
}

//volatile int blink = 64; //45 degrees
volatile int blink = 16; //11.25 degrees
void demo2(){
	uint8_t byteData,sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
	uint16_t wordData;

	uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
		// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	int count = 0;
	int errorHandler = 0;
	uint32_t delay = 200;															// minimum value is SysTick_Wait10us(150)
	float theta = 3.14159265358979323846/16; // divide by 16 for 11.25 deg, divide by 4 for 45 deg

	for(int i=0; i<512; i++){												// 512 * 4 (full step) = 2048 steps
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10us(delay);											// using us instead of ms
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10us(delay);
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10us(delay);
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10us(delay);
		count++;
		returnHome++;
		if (flag == 0)
		{
			skipping = 1;
			home();
			break;
		}
		if (count == blink && skipping == 0){ // 11.25 deg/64 ratio -> 0.1758 -> 45 deg/0.1758 = 256 steps -> each loop has 4 steps repeated 512 times -> 256/4 = 64 cycles
			SysTick_Wait10ms(1);
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
        FlashLED3(1);
				VL53L1_WaitMs(dev, 5);
			}
			uint16_t average = 0;
			for (int j = 0; j < 5; j++)
			{
				errorHandler = 0;
				status = VL53L1X_GetDistance(dev, &Distance);
				while (RangeStatus != 0) //filters out errors
				{
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
					status = VL53L1X_GetDistance(dev, &Distance);
					SysTick_Wait10ms(1);
					if (RangeStatus == 4 || RangeStatus == 2) // out of bounds error
					{
						Distance = 4000;
						RangeStatus = 0;
						errorHandler = 0;
						status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
						break;
					}
			  }
				average += Distance;
				status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
				SysTick_Wait10ms(1);
			}
			average /= 5;
			FlashLED4(1);

			status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
		
			// print the resulted readings to UART
			
			signed int y = sin(theta)*average;
			signed int x = cos(theta)*average;
			
			sprintf(printf_buffer,"%d\n", x);
			UART_printf(printf_buffer);
			sprintf(printf_buffer,"%d\n", y);
			UART_printf(printf_buffer);
			SysTick_Wait10ms(50);
			status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
			
			count = 0;
			theta = theta + 3.14159265358979323846/16;
		}
	}
	//spin back
	if (skipping == 0)
		home();
	skipping = 0;
}

//Flash LED D2 (D2 is connected to PN0 and PN2 (to measure using AD2 if needed))
void LEDDutyCycle(int percent) {
		int percent_off = 100-percent;
		GPIO_PORTN_DATA_R ^= 0b00000101;
		SysTick_Wait10ms(percent);
		GPIO_PORTN_DATA_R ^= 0b00000101;
		SysTick_Wait10ms(percent_off);
}

// Assumes 120MHz bus, bus period = 1/(120*10^6) = 8.33 ns
// If bus clock = timer clock, then max interrupt period = 1/(120*10^6) * 2^32 = 35.8s
// Since we multiply the period by 120 to match the units of 1 us, the period will be in 1us units
// We want to set the interrupt period to be 0.25 sec. Calculate the value that must be placed in period to achieve this:
// 0.25s = 250000 us (Fill in the blank)
void Timer3_Init(void){

	uint32_t period = 1000000; 								// 32-bit value in 1us increments
	
	// Step 1: Activate timer
	SYSCTL_RCGCTIMER_R = 0x08;							// (Step 1)Activate timer 
	SysTick_Wait10ms(1);							// Wait for the timer module to turn on
	
	
	// Step 2: Arm and Configure Timer Module
	TIMER3_CTL_R = 0x0;								// (Step 2-1) Disable Timer3 during setup (Timer stops counting)
	TIMER3_CFG_R = 0x0;								// (Step 2-2) Configure for 32-bit timer mode   
	TIMER3_TAMR_R = 0x02;							// (Step 2-3) Configure for periodic mode   
	TIMER3_TAPR_R = 0x0;	    				// (Step 2-4) Set prescale value to 0; i.e. Timer3 works with Maximum Freq = bus clock freq (120MHz)  
	TIMER3_TAILR_R = (period*60)-1; 	// (Step 2-5) Reload value (we multiply the period by 120 to match the units of 1 us)  
	TIMER3_ICR_R = 0x01;		 						// (Step 2-6) Acknowledge the timeout interrupt (Clear timeout flag of Timer3)
	TIMER3_IMR_R = 0x01;								// (Step 2-7) Arm timeout interrupt   
	
	
	// Step 3: Enable Interrupt at Processor side
	NVIC_EN1_R = 0x08;									// Enable IRQ 35 in NVIC 
	NVIC_PRI8_R = 0x40000000;								// Set Interrupt Priority to 2 
																
	EnableInt();															// Global Interrupt Enable
	
	
	// Step 4: Enable the Timer to start counting
	TIMER3_CTL_R = 0x01;								// Enable Timer3
} 
volatile int done = 0;
void TIMER3A_IRQHandler(void){ 
	if (done == 1){
		LEDDutyCycle(100);															// insert int from 0-100 to indicate duty cycle percent
	}
	TIMER3_ICR_R = 0x01;								// Acknowledge Timer3 timeout
}

int main(void) {

	//initialize
	PLL_Init();	
	PortG_Init();
	PortM_Init();
	SysTick_Init();
	//onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortN_Init();					// Initialize the onboard LED on port N
	PortF_Init();					// Initialize led PF0
	PortJ_Init();					// Onboard push button
	PortJ_Interrupt_Init();		// Initalize and configure the Interrupt on Port J
	Timer3_Init();					  // Initalize and configure the Timer3 Interrupt

	int input = 0;
	
	uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	// run the mcu and wait for python input before starting to collect data, loop the input check
	input = UART_InChar(); //read python thing first
	UART_printf("ToF Chip Booted!\r Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
	/* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	while(done == 0)
	{
		if (flag == 1 && input == 's'){ //if the interrupt is successful, the flag should be set to 1, running the motor
			demo2();
			pressed = 0; //reset the pressed flag to 0
			flag = 0;
		}
		else if (pressed2 == 0)
		{
			char stop[3];
  		sprintf(stop, "t\r");
			GPIO_PORTM_DATA_R = 0b00000000;
			UART_printf(stop);
			status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
			done = 1;
			break;
		}
	}
	while(1){
		;
	}
}
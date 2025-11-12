
#include "xil_printf.h"
#include "xil_exception.h"
#include "xil_types.h"
#include "xintc.h"
#include "xparameters.h"
#include "xtmrctr.h"
#include "xgpio.h"

/*-------------------------------------------------------------------
MEMORY MAPPINGS
-------------------------------------------------------------------*/

#define AXI_GPIO_0_BASE_ADDR 0x40000000
#define AXI_GPIO_1_BASE_ADDR 0x40010000

#define GREEN_LEDS_BASE_ADDR (AXI_GPIO_0_BASE_ADDR)             /* Not used */
#define PUSH_BTNS_BASE_ADDR (AXI_GPIO_0_BASE_ADDR + 8)
#define GREEN_LEDS_REG (unsigned *)(GREEN_LEDS_BASE_ADDR)       /* Not used */
#define PUSH_BTNS_REG (unsigned *)(PUSH_BTNS_BASE_ADDR)
#define RGB_LEDS_BASE_ADDR (AXI_GPIO_1_BASE_ADDR)
#define DIP_SWITCHES_BASE_ADDR (AXI_GPIO_1_BASE_ADDR + 8)       /* Not used */

/* RGB LEDS */
// Bitfields for manipulating individual LEDs
#define RGB_LD0 0b000000000111
#define RGB_LD1 0b000000111000
#define RGB_LD2 0b000111000000
#define RGB_LD3 0b111000000000

#define RGB_LED_ALL_RED         0b100100100100
#define RGB_LED_ALL_GREEN       0b010010010010
#define RGB_LED_ALL_OFF         0b000000000000

#define INT_CONTROLLER_BASE_ADDR    0x41200000
#define INTC_DEVICE_ID              0
#define TMRCTR_DEVICE_ID            0
#define TMRCTR_INTERRUPT_ID         XPAR_FABRIC_XTMRCTR_0_INTR
#define INTC_GPIO_INT_ID            XPAR_FABRIC_XGPIO_0_INTR
#define TIMER_0                     XTC_TIMER_0

/* TIMERS */
#define TIM_BASE_ADDR           0x41C00000
#define TIM_TCSR0_REG           (TIM_BASE_ADDR + 0x0)   // Control and Status Register
#define TIM_TLR0_REG            (TIM_BASE_ADDR + 0x4)   // Load Register
#define TIM_TCR0_REG            (TIM_BASE_ADDR + 0x8)   // Counter Register

#define TIM_TCSR1_REG           (TIM_BASE_ADDR + 0x10)  // Control and Status Register
#define TIM_TLR1_REG            (TIM_BASE_ADDR + 0x14)  // Load Register
#define TIM_TCR1_REG            (TIM_BASE_ADDR + 0x18)  // Counter Register

#define TIM_TCSR_LOAD_BIT       0b000000100000
#define TIM_TCSR_ENABLE_BIT     0b000010000000
#define TIM_TCSR_INT_BIT        0b000100000000

// Timer register pointers
unsigned *tim0_tcsr = TIM_TCSR0_REG;
unsigned *tim0_tlr = TIM_TLR0_REG;
unsigned *tim0_tcr = TIM_TCR0_REG;

// NEED A SECOND TIMER
// PWM FREQUENCY: 2kHz
// NEED a much higher timer frequency to get good pwm resolution
// SET UP TIM1

//PMOD MAXSONAR:
// CAN RECEIVE DATA EITHER VIA UART, PWM, OR ANALOG
// FIGURE OUT WHICH. PROBABLY UART?

//PMOD LS1:
// SENDS A DIGITAL 1 WHEN THE LIGHT IS ABOVE A CERTAIN THRESHOLD.
// EZPZ, JUST MAKE IT AN INTERRUPT

typedef enum 
{ENUMERATE_ME} State; 

// Create state variable
State state;

// Data and tristate register ptrs for the 4 push buttons on the Arty board
//unsigned *buttonsData = PUSH_BTNS_REG;
//unsigned *buttonsTri = PUSH_BTNS_REG + 1;
unsigned *rgbLEDsData = RGB_LEDS_BASE_ADDR;
unsigned *rgbLEDsTri = RGB_LEDS_BASE_ADDR + 1;

static unsigned int doOnce = 0;
static unsigned long int globalTimestamp = 0;       // in ms
static unsigned long int redStateTimeout = 0;
static unsigned long int flashingRedStateTimeout = 0;
static int flashingRedStateIterator = 0; 

static _Bool buttonPressed = 0;
static unsigned long int buttonPressDbnceTimeout = 0;

void executionFailed();

void setupInterrupts();
void setupTimer();

XIntc InterruptController;  // Create an instance of the interrupt controller
XTmrCtr TimerCounter;       // Create an instance of the Timer Counter
XGpio gpio;                 // Create an instance of the gpio driver

/*---------------------------------------------------------------------------------------
    gpioIntISR()

        Interrupt service routine that is called whenever a button press is detected
        
---------------------------------------------------------------------------------------*/
void gpioIntISR(void *CallbackRef) {
    // Clear the interrupt
    XGpio_InterruptClear(&gpio, 0xFF);
    
    // If a button press (not a button release) triggered this interrupt
    if (XGpio_DiscreteRead(&gpio, 2) > 0) {
        xil_printf("Button press detected.\n");
        buttonPressed = 1;
    }
    // Else this interrupt is a button release
    else {
        buttonPressed = 0;
    }

}


/*---------------------------------------------------------------------------------------
    TimerCounterISR()

        Interrupt Service routine that is called once per millisecond

---------------------------------------------------------------------------------------*/

void TimerCounterISR(void *CallBackRef, u8 TmrCtrNumber) {
  XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

  if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber)) {
    globalTimestamp++;
  }
}


/*---------------------------------------------------------------------------------------
    FSM_tick()

        Manages the Finite State machine for the robot

---------------------------------------------------------------------------------------*/
void FSM_tick() 
{ 
  switch(state) 
  { 
    case 0: 
      break; 
  } 
}


/*---------------------------------------------------------------------------------------
    main()

        Program entry point

---------------------------------------------------------------------------------------*/

int main(void)
{
    // Set the tri-state direction of the AXI_GPIO_0 pins.
    // Buttons = inputs, LEDs = outputs
    *rgbLEDsTri = 0x0;

    setupInterrupts();
    setupTimer();

    state = 0;
    doOnce = 0;

    // Infinite loop
    while(1)
    {
        // cycle through traffic light pattern 
        FSM_tick(); 

    }
    return 0;
}

/*---------------------------------------------------------------------------------------
    executionFailed()

        Failure handler function

---------------------------------------------------------------------------------------*/

void executionFailed() {
  *rgbLEDsData = 04444; // display all red LEDs if fail state occurs

  // trap the program in an infinite loop
  while (1)
    ;
}

/*---------------------------------------------------------------------------------------
    setupInterrupts()

        setup function that prepares interrupt controller and registers ISR functions

---------------------------------------------------------------------------------------*/

void setupInterrupts() {

    // Initialize the timer counter instance
    int status = XTmrCtr_Initialize(&TimerCounter, TMRCTR_DEVICE_ID);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to initialize the timer! Execution stopped.\n");
        executionFailed();
    }

    // Initialize the gpio instance
    status = XGpio_Initialize(&gpio, AXI_GPIO_0_BASE_ADDR);
    if (status != XST_SUCCESS) {
        xil_printf("Gpio Initialization Failed\r\n");
        executionFailed();
    }

    // Set buttons as input
    XGpio_SetDataDirection(&gpio, 2, 0xF);    
    

    // Initialize the interrupt controller instance
    status = XIntc_Initialize(&InterruptController, INTC_DEVICE_ID);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to initialize the interrupt controller! Execution stopped.\n");
        executionFailed();
    }

    // Connect the gpio interrupt handler funciton
    status = XIntc_Connect(&InterruptController, INTC_GPIO_INT_ID,
                         (XInterruptHandler)gpioIntISR,
                         (void *)0);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to connect gpio int handler! Execution stopped.\n");
        executionFailed();
    }

    // Enable the GPIO interrupt channel
    XIntc_Enable(&InterruptController, INTC_GPIO_INT_ID);

    // Verifies the specified timer is setup correctly in hardware/software
    status = XTmrCtr_SelfTest(&TimerCounter, 1);
    if (status != XST_SUCCESS) {
        xil_printf("Testing timer operation failed! Execution stopped.\n");
        executionFailed();
    }


    // Connect the timer's interrupt handler function
    status = XIntc_Connect(&InterruptController, TMRCTR_INTERRUPT_ID,
                         (XInterruptHandler)XTmrCtr_InterruptHandler,
                         (void *)&TimerCounter);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to connect timer handler! Execution stopped.\n");
        executionFailed();
    }

    
    // Start the interrupt controller
    status = XIntc_Start(&InterruptController, XIN_REAL_MODE);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to start interrupt controller! Execution stopped.\n");
        executionFailed();
    }

    XGpio_InterruptEnable(&gpio, 0xF);
    XGpio_InterruptGlobalEnable(&gpio);
    // Enable interrupts and the exception table
    XIntc_Enable(&InterruptController, TMRCTR_INTERRUPT_ID);
    Xil_ExceptionInit();
    // Register the interrupt controller handler with the exception table.
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                (Xil_ExceptionHandler)XIntc_InterruptHandler,
                                &InterruptController);
    Xil_ExceptionRegisterHandler(INTC_GPIO_INT_ID,
                                (Xil_ExceptionHandler)gpioIntISR,
                                0);
    Xil_ExceptionEnable();
    // Register the TimerCounterISR function
    XTmrCtr_SetHandler(&TimerCounter, (XTmrCtr_Handler)TimerCounterISR,
                     &TimerCounter);

}


/*---------------------------------------------------------------------------------------
    setupTimer()

        Sets up the 1ms reload timer that triggers TimerCounterISR() interrupt

---------------------------------------------------------------------------------------*/

void setupTimer() {
    // Timer options setup
    u32 options =
      XTC_INT_MODE_OPTION | XTC_DOWN_COUNT_OPTION | XTC_AUTO_RELOAD_OPTION;
    XTmrCtr_SetOptions(&TimerCounter, TIMER_0, options);

    // For 81.247969MHz clock, count time is 1ms
    // This means the clock should reset every 1.0000003815ms
    // Set what value the timer should reset/init to (setting TLR0 indirectly)
    XTmrCtr_SetResetValue(&TimerCounter, TIMER_0, 81248);

    // Start the timer
    XTmrCtr_Start(&TimerCounter, TIMER_0);
}
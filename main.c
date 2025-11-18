
#include "xil_printf.h"
#include "xil_exception.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xintc.h"
#include "xparameters.h"
#include "xtmrctr.h"
#include "xgpio.h"

#include "Pmod_Dual_MAXSONAR.h"
#include "Pmod_DHB1.h"
#include "MotorFeedback.h"
#include "PWM.h"
#include <stdint.h>

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

/*-------------------------------------------------------------------
MAIN CLOCK FREQUENCY
-------------------------------------------------------------------*/
#define CLK_FREQ_HZ                 166666670

/*-------------------------------------------------------------------
SONAR
-------------------------------------------------------------------*/
#define PMOD_MAXSONAR_BASEADDR      0x44A00000

#define SONAR_THRESHOLD_VALUE       68000

PMOD_DUAL_MAXSONAR Sonar;

/*-------------------------------------------------------------------
INFRARED SENSORS
-------------------------------------------------------------------*/
#define INFRARED_SENSOR_BASE_ADDR   0x40020000

#define LEFT_SENSOR                 0b01
#define RIGHT_SENSOR                0b10

/*-------------------------------------------------------------------
DHB1 MOTOR CONTROLLER
-------------------------------------------------------------------*/
#define DHB1_GPIO                   0x44A10000
#define DHB1_MOTOR_FB               0x44A20000
#define DHB1_PWM                    0x44A30000

#define PWM_PER                     2
#define SENSOR_EDGES_PER_REV        4
#define GEARBOX_RATIO               48

PmodDHB1 pmodDHB1;
MotorFeedback motorFeedback;

/*-------------------------------------------------------------------
RGB LEDS
-------------------------------------------------------------------*/
// Bitfields for manipulating individual LEDs
#define RGB_LD0                     0b000000000111
#define RGB_LD1                     0b000000111000
#define RGB_LD2                     0b000111000000
#define RGB_LD3                     0b111000000000

#define RGB_LED_ALL_RED             0b100100100100
#define RGB_LED_ALL_GREEN           0b010010010010
#define RGB_LED_ALL_BLUE            0b001001001001
#define RGB_LED_ALL_OFF             0b000000000000

unsigned *rgbLEDsData = RGB_LEDS_BASE_ADDR;
unsigned *rgbLEDsTri = RGB_LEDS_BASE_ADDR + 1;

/*-------------------------------------------------------------------
INTERRUPTS
-------------------------------------------------------------------*/
#define INT_CONTROLLER_BASE_ADDR    0x41200000
#define INTC_DEVICE_ID              0
#define TMRCTR_DEVICE_ID            0
#define TMRCTR_INTERRUPT_ID         XPAR_FABRIC_XTMRCTR_0_INTR
#define INTC_GPIO_INT_ID            XPAR_FABRIC_XGPIO_0_INTR
#define TIMER_0                     XTC_TIMER_0

/*-------------------------------------------------------------------
TIMERS
-------------------------------------------------------------------*/
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

/*-------------------------------------------------------------------
SYSTEM STATE
-------------------------------------------------------------------*/
typedef enum {
    STARTUP,
    CENTER_FINDING,
    LINE_FOLLOWING,
    SONAR_STOPPED
} State; 

// Create state variable
State state = STARTUP;

// Data and tristate register ptrs for the 4 push buttons on the Arty board
//unsigned *buttonsData = PUSH_BTNS_REG;
//unsigned *buttonsTri = PUSH_BTNS_REG + 1;

static unsigned int doOnce = 0;
static unsigned long int globalTimestamp = 0;       // in ms

enum {
    NO_LINE_DETECTED = 0,
    LEFT_LINE_DETECTED = 1,
    RIGHT_LINE_DETECTED = 2,
    BOTH_LINES_DETECTED = 3
}
static uint8_t lineDetectionState = NO_LINE_DETECTED;

static _Bool buttonPressed = 0;
static unsigned long int buttonPressDbnceTimeout = 0;

// SONAR DISTANCE SENSORS
uint32_t sonarDist;

// INFRARED LINE SENSORS
_Bool leftSensor = FALSE;
_Bool rightSensor = FALSE;

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
    delayMs()

        Waits until the specified number of milliseconds have passed
        
---------------------------------------------------------------------------------------*/
void delayMs(unsigned long int delayTimeMs) {
    unsigned long int targetTime = globalTimestamp + delayTimeMs;
    while (globalTimestamp < targetTime) {
        // wait
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
    getSonarDistance()

        gets the raw sensor value from the two sonar sensors, averages it, then puts that
        average into a simple moving average for filtering.

---------------------------------------------------------------------------------------*/

uint32_t getSonarDistance() {
    static uint32_t movingAvgData[4] = {0};
    static uint32_t staleIndex = 0;
    static _Bool isFull = FALSE;

    uint32_t dist = MAXSONAR_getDistance(&Sonar, 1);
    uint32_t dist2 = MAXSONAR_getDistance(&Sonar, 2);

    movingAvgData[staleIndex] = ((dist + dist2) / 2);
    staleIndex = (staleIndex + 1) % 4;

    if (staleIndex == 0) {
        isFull = TRUE;
    }

    if (!isFull) {
        return (SONAR_THRESHOLD_VALUE + 1000);
    }
    else {
        uint32_t sma = 0;
        for (int i = 0; i < 4; i++) {
            sma += movingAvgData[i];
        }
        sma = sma / 4;
        return sma;
    }
}

/*---------------------------------------------------------------------------------------
    readInfraredSensor()
        Returns TRUE if the input sensor is detecting a reflective surface. Returns FALSE
        otherwise.

        Parameters:
            sensor - either LEFT_SENSOR or RIGHT_SENSOR
        

---------------------------------------------------------------------------------------*/
_Bool readInfraredSensor(uint8_t sensor) {
    return ( *(volatile u32 *)(INFRARED_SENSOR_BASE_ADDR) & sensor );
}


/*---------------------------------------------------------------------------------------
    FSM_tick()

        Manages the Finite State machine for the line following robot

---------------------------------------------------------------------------------------*/
void FSM_tick() 
{ 

    static unsigned long int lineFallingEdge_Ms = 0;
    static _Bool prevLineReading = true;

  switch(state) 
  { 
    case STARTUP:
        if (!doOnce) {
            // Dwell for 5 seconds on startup;
            while (globalTimestamp < 5000) {
                // Make a LED countdown to robot start. Think like the countdown light
                // in a drag race
                if (globalTimestamp < 2500) {
                    // Set LD2 Red
                    *rgbLEDsData = (RGB_LD2 & RGB_LED_ALL_RED);
                }
                else {
                    // Set LD1 Yellow
                    *rgbLEDsData = (RGB_LD1 & (RGB_LED_ALL_RED | RGB_LED_ALL_GREEN));
                }
                
            }
            // Set LEDs all blue to indicate centering
            *rgbLEDsData = RGB_LED_ALL_BLUE;
            doOnce = true;
        }

        leftSensor = readInfraredSensor(LEFT_SENSOR);
        rightSensor = readInfraredSensor(RIGHT_SENSOR);
        
        if (leftSensor && rightSensor) {
            lineDetectionState = BOTH_LINES_DETECTED;
            executionFailed(); // Error state
        }
        else if (leftSensor) {
            lineDetectionState = LEFT_LINE_DETECTED;
        }
        else if (rightSensor) {
            lineDetectionState = RIGHT_LINE_DETECTED;
        }
        else {
            lineDetectionState = NO_LINE_DETECTED;
        }

        state = CENTER_FINDING;
        doOnce = 0;
        // Start with one sensor on the line.
        // Rotate the opposite direction until no longer reading the line, start timer.
        // Time until the other sensor reads the line.
        // Divide that time by two, and turn back for half that time.
        // Robot should now be 'straight' on the line

        break;

    case CENTER_FINDING:

        if (!doOnce) {
            if (lineDetectionState == LEFT_LINE_DETECTED) {
                DHB1_setDirs(&pmodDHB1, 1, 1); // left turn
            }
            else if (lineDetectionState == RIGHT_LINE_DETECTED) {
                DHB1_setDirs(&pmodDHB1, 0, 0); // right turn
            }
            else {
                executionFailed(); // Error state
            }

            delayMs(30);
            DHB1_setMotorSpeeds(&pmodDHB1, 35, 35);

            doOnce = 1;
        }

        switch (lineDetectionState) {
            case LEFT_LINE_DETECTED:
                // Rotate left until no longer detecting line
                while(!readInfraredSensor(LEFT_SENSOR)) {
                    
                }
                if (!readInfraredSensor(LEFT_SENSOR)) { // A way to get a simple debounce
                    break;
                }
                // Stop
                DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
                delayMs(50);
                // Start turning right until right sensor detects line
                DHB1_setDirs(&pmodDHB1, 0, 0);
                delayMs(30);
                DHB1_setMotorSpeeds(&pmodDHB1, 35, 35);
                // Start timer
                unsigned long int startTime = globalTimestamp;
                
                while (!readInfraredSensor(RIGHT_SENSOR)) {
                    // wait
                }
                unsigned long int elapsedTime = globalTimestamp - startTime;

                // Stop
                DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
                // Rotate left for half that time
                delayMs(50);
                DHB1_setDirs(&pmodDHB1, 1, 1);
                delayMs(30);
                DHB1_setMotorSpeeds(&pmodDHB1, 35, 35);
                unsigned long int targetTime = globalTimestamp + (elapsedTime / 2);
                while (globalTimestamp < targetTime) {
                    // wait
                }
                DHB1_setMotorSpeeds(&pmodDHB1, 0, 0); // stop
                delayMs(50);
                DHB1_setDirs(&pmodDHB1, 0, 1); // set forward
                *rgbLEDsData = RGB_LED_ALL_GREEN;
                delayMs(200);
                break;

            case RIGHT_LINE_DETECTED:
                // Rotate right until no longer detecting line
                while(!readInfraredSensor(RIGHT_SENSOR)) {
                    
                }
                if (!readInfraredSensor(RIGHT_SENSOR)) { // A way to get a simple debounce
                    break;
                }
                // Stop
                DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
                delayMs(50);
                // Start turning left until left sensor detects line
                DHB1_setDirs(&pmodDHB1, 1, 1);
                delayMs(30);
                DHB1_setMotorSpeeds(&pmodDHB1, 35, 35);
                // Start timer
                unsigned long int startTime = globalTimestamp;
                
                while (!readInfraredSensor(LEFT_SENSOR)) {
                    // wait
                }
                unsigned long int elapsedTime = globalTimestamp - startTime;

                // Stop
                DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
                // Rotate right for half that time
                delayMs(50);
                DHB1_setDirs(&pmodDHB1, 0, 0);
                delayMs(30);
                DHB1_setMotorSpeeds(&pmodDHB1, 35, 35);
                unsigned long int targetTime = globalTimestamp + (elapsedTime / 2);
                while (globalTimestamp < targetTime) {
                    // wait
                }
                DHB1_setMotorSpeeds(&pmodDHB1, 0, 0); // stop
                delayMs(50);
                DHB1_setDirs(&pmodDHB1, 0, 1); // set forward
                *rgbLEDsData = RGB_LED_ALL_GREEN;
                delayMs(200);
                break;

            default:
                executionFailed(); // Error state
                break;
        }

        doOnce = 1;
        state = LINE_FOLLOWING;
        }
        break;

    case LINE_FOLLOWING:
        if (!doOnce) {
            DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);
            doOnce = 1;
        }

        // Read infrared sensors
        leftSensor = readInfraredSensor(LEFT_SENSOR);
        rightSensor = readInfraredSensor(RIGHT_SENSOR);

        // Left turn if left sensor detects line
        if (leftSensor && !rightSensor) {
            DHB1_setMotorSpeeds(&pmodDHB1, 30, 50);
            *rgbLEDsData = (RGB_LD0 & RGB_LED_ALL_BLUE);
        }
        // Right turn if right sensor detects line
        else if (!leftSensor && rightSensor) {
            DHB1_setMotorSpeeds(&pmodDHB1, 50, 30);
            *rgbLEDsData = (RGB_LD3 & RGB_LED_ALL_BLUE);
        }
        // If neither sensor detects line, go straight
        else if (!leftSensor && !rightSensor) {
            // Both sensors detect line, go straight
            DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);
            *rgbLEDsData = ((RGB_LD0 | RGB_LD3) & RGB_LED_ALL_GREEN);
        }
        // If both sensors detect line, set LEDs to yellow
        else {
            // No line detected, stop
            //DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
            *rgbLEDsData = (RGB_LED_ALL_RED | RGB_LED_ALL_GREEN);
        }

        sonarDist = getSonarDistance();
        if (sonarDist < SONAR_THRESHOLD_VALUE) {
            state = SONAR_STOPPED;
            doOnce = 0;
        }
        delayMs(5); // small delay to avoid excessive looping
        break;

    case SONAR_STOPPED:
        // Stop the motors
        DHB1_setMotorSpeeds(&pmodDHB1, 0, 0);
        // Set LEDs to all red
        *rgbLEDsData = RGB_LED_ALL_RED;

        executionFailed(); // Stop execution for now
        break;
    
    default:
        break;

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

    state = STARTUP;
    doOnce = 0;
    MAXSONAR_begin(&Sonar, PMOD_MAXSONAR_BASEADDR, CLK_FREQ_HZ);

    DHB1_begin(&pmodDHB1, DHB1_GPIO, DHB1_PWM, CLK_FREQ_HZ, PWM_PER);
    MotorFeedback_init(&motorFeedback, DHB1_MOTOR_FB, CLK_FREQ_HZ,
         SENSOR_EDGES_PER_REV, GEARBOX_RATIO);
    DHB1_motorEnable(&pmodDHB1);

    DHB1_setDirs(&pmodDHB1, 0, 1); // Set direction forward

    // Infinite loop
    //DHB1_setMotorSpeeds(&pmodDHB1, 50, 50);
    while(1)
    {
        FSM_tick(); 

        /*-------------------------------------------------------------------
        DRIVER TESTS
        -------------------------------------------------------------------*/
        sonarDist = getSonarDistance();
        if (sonarDist < SONAR_THRESHOLD_VALUE) {
            *rgbLEDsData = 04444;
        }

        leftSensor = readInfraredSensor(LEFT_SENSOR);
        //if (leftSensor) { xil_printf("left: TRUE  ");} else { xil_printf("left: FALSE  ");}

        rightSensor = readInfraredSensor(RIGHT_SENSOR);
        //if (rightSensor) { xil_printf("right: TRUE\r\n");} else { xil_printf("right: FALSE\r\n");}
        
        /* TODO: TEST THAT TIMER IS APPROXIMATELY CORRECT */

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

    // TODO!!!!: CHECK IF THE CLOCK IS GETTING THE NEW 166.667MHz clock
    // If so, we need to call:
    // XTmrCtr_SetResetValue(&TimerCounter, TIMER_0, 166667);

    // Start the timer
    XTmrCtr_Start(&TimerCounter, TIMER_0);
}

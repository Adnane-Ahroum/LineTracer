

//this sourcefile has all the work

Test Function
int fibo (int num) {
    if (num <= 1) return 0;
    else if (num == 2) return 1;
    return fibo(num-1) + fibo(num-2);
    }

Register Control
int main(void) {
    Clock_Init48Mhz();
    //Setup P2 0~2 bit & P2 0 bit as GPIO
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P1->SEL0 &= ~0x01;
    P1->SEL1 &= ~0x01;
    // Setup P2 0~2 bit & P2 0 bit as OUTPUT
    P2->DIR |= 0x07;
    P2->DIR |= 0x01;
    //Turn off all the LEDs initially
    P2->OUT &= ~0x07;
    P2->OUT &= ~0x01;
    // Turn on Red LED
    P2->OUT |= ~0x1;
    P1->OUT |= ~0x1;
    }


Turn on Red & Blue: LED Control
// Turn on all Red LED
    P2->OUT |= 0x1;
    P1->OUT |= 0x1;

// Turn on red & blue LED
    P2->OUT |= 0x1;
    P2->OUT |= 0x4;
    
    // The LED connected to Port1.0
    // can only emit red light
    P1->OUT |= 0x1;

Make Colours in order Red, Green, Blue
 Clock_Delay1ms(1000); //Red
        P2->OUT &= ~0x07;
        P2->OUT &= ~0x01;
        P2->OUT |= 0x01;
        Clock_Delay1ms(1000); //Green
        P2->OUT &= ~0x01;
        P2->OUT &= ~0x02;
        P2->OUT |= 0x02;
        Clock_Delay1ms(1000); //Blue
        P2->OUT &= ~0x07;
        P2->OUT &= ~0x04;
        P2->OUT |= 0x04;
    }
Basic Switch Control
//Enable pull resistors
    P1->REN |= 0x12;
    
    // Now Pull-up
    P1->OUT |= 0x12;
while(1){
        int sw1;
        
        sw1 = P1->IN & 0x02;
        if (!sw1) {
            P2->OUT |= 0x01;
        } else {
            P2->OUT &= ~0x07;
        }
    }

Control Colours LED with Switch
int led_num = 0;
    int prev_state = 0;
    while(1){
        int sw1;
        sw1 = P1->IN & 0x02;
        if (!sw1 && prev_state ==0) {
            if (!led_num) led_num = 1;
            else led_num <<= 1;
            led_num %= 8;
            prev_state = 1;
        } else of (sw1) {
            prev_state = 0;
        }
        P2->OUT &= ~0x07;
        P2->OUT |= led_num;
    }
}

Basic IR Sensor Control
// 0, 2, 4, 6 IR Emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08; //GPIO
    P5->DIR |= 0x08; // OUTPUT
    P5->OUT &= ~0x08;  // turn off 4 even IR LEDS
    // 1,3,5,7 IR Emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04; //GPIO
    P9->DIR |= 0x04; // OUTPUT
    P9->OUT &= ~0x04; // turn off 4 odd IR LEDs
    // 0~7 IR Sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF; //GPIO
    P7->DIR &= ~0xFF; //INPUT

while(1){
        // Turn on IR LEDs
        P5->OUT |= 0X08;
        P9->OUT |= 0X04;
        // Make P7.0-P7.7 as output
        P7->DIR = 0xFF;
        //Charges a Capacitor
        P7->OUT = 0xFF;
        // Wait to be fully charged
        Clock_Delay1us(10);
        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        Clock_Delay1us(1000);
        int sensor;
        // Read P7.7 - P7.0 Input
        // white: 0 , Black ; 1
        sensor = P7->IN & 0x10;
        if (sensor) {
            P2->OUT |= 0x01;
        } else {
            P2->OUT &= ~0x07;
        }
        // Turn off IR LEDs
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;
        Clock_Delay1ms(10);
    }
Edit to make 7.3 and 7.4 read and turn on LED red
while(1){
        // Turn on IR LEDs
        P5->OUT |= 0X08;
        P9->OUT |= 0X04;
        // Make P7.0-P7.7 as output
        P7->DIR = 0xFF;
        //Charges a Capacitor
        P7->OUT = 0xFF;
        // Wait to be fully charged
        Clock_Delay1us(10);
        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        Clock_Delay1us(1000);
        int sensor;
               // Read P7.7 - P7.0 Input
               // white: 0 , Black ; 1
               sensor = P7->IN & 0x18;
               if (sensor == 0x18) {
                   P2->OUT |= 0x01;
               } else {
                   P2->OUT &= ~0x07;
               }
               // Turn off IR LEDs
               P5->OUT &= ~0x08;
               P9->OUT &= ~0x04;
               Clock_Delay1ms(10);
    }

Time change from White to Black
// Start Timer
start_time = get_current_time();
// Make P7.0-P7.7 as input
P7->DIR = 0x00;
// Continuously monitor phototransistor input
while(1) {
    int current_state = P7->IN & 0xFF;
    
    // Detect transition from white to black
    if (current_state != previous_state && current_state == BLACK_STATE) {
        // Stop Timer
        stop_time = get_current_time();
        // Calculate Time Difference
        time_taken = stop_time - start_time;
        // Print or store time_taken as needed
        break; // Exit loop once transition is detected
    }
    // Update previous state
    previous_state = current_state;
}
// Helper function to get current time (time in milliseconds since program start)
uint32_t get_current_time() {
    // Implement this function based on your microcontroller's timer or system clock
}

Simple SysTick Program
#define LED_RED (1 << 0)
#define LED_GREEN (1 << 1)
#define LED_BLUE (1 << 2)
void SysTick_Init(void) {
    SysTick ->LOAD = 0x00FFFFFF;
    SysTick ->CTRL = 0x00000005;
}
void Led_Init(void) {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x7;
    P2->OUT &= ~0x07;
}
void TurnOn_Led(int color) {
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}
void TurnOff_Led(void) {
    P2->OUT &= ~0x07;
}
void SysTick_Wait1ms() {
    SysTick->LOAD = 48000 - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1s() {
    int i, count = 1000;
    for (i=0; i < count; i++){
        SysTick_Wait1ms();
    }
    printf("1s passed...\n");
}
int main(void)
{
    Clock_Init48MHz();
    SysTick_Init();
    Led_Init();
    while(1){
        TurnOn_Led(LED_RED);
        SysTick_Wait1s();
        TurnOn_Led(LED_BLUE);
        SysTick_Wait1s();
    }
}


Simple PWM Program (speed)
LED blinking
int main(void)
{
    // 0 < speed <10000
    int speed = 5000;
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    Led_Init();
    
    while(1){
        TurnOn_Led(RED);
        SysTick_Wait1msDelay(100);
        TurnOff_Led();
        SysTick_Wait1msDelay(100);
    }
    
}
LED No blinking
Delay change from 100 to 1

PWM Brightness
int main(void)
{
    // 0 < speed <10000
    int speed = 5000;
    int delay = 1;
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    Led_Init();
    while(1){
        if (delay >= 10000) delay = 1;
        TurnOn_Led(LED_RED);
        SysTick_Wait1us(10000 - delay);
        TurnOff_Led();
        SysTick_Wait1us(delay);
        delay += 100;
    }
}


Motor Initialization
#define LED_RED (1 << 0)
#define LED_GREEN (1 << 1)
#define LED_BLUE (1 << 2)
void SysTick_Init(void) {
    SysTick ->LOAD = 0x00FFFFFF;
    SysTick ->CTRL = 0x00000005;
}
void Led_Init(void) {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x7;
    P2->OUT &= ~0x07;
}
void TurnOn_Led(int color) {
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}
void TurnOff_Led(void) {
    P2->OUT &= ~0x07;
}
void SysTick_Wait1ms() {
    SysTick->LOAD = 48000 - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1s() {
    int i, count = 1000;
    for (i=0; i < count; i++){
        SysTick_Wait1ms();
    }
    printf("1s passed...\n");
}
void Motor_Init(void){
    P3->SEL0 &= ~0xC0; // 1) Configure nSLPR & nSLPL as GPIO
    P3->SEL1 &= ~0xC0; // 2) make nSLPR & nSLPL as output
    P3->DIR |= 0xC0; // 3) output LOW
    P3->OUT &= ~0xC0;
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;
}
int main(void)
{
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    while(1){
        // Move Forward
        P5->OUT &= ~0x30; // PH =0 (yes)
        P2->OUT |= 0xC0; // EN = 1
        P3->OUT |= 0xC0; //nSleep=1
        SysTick_Wait1s();
        // Stop
        P2->OUT &= ~0xC0; //EN =0
        SysTick_Wait1s();
    }
}


Simple Motor Speed Control
void Motor_Init(void){
    P3->SEL0 &= ~0xC0; // 1) Configure nSLPR & nSLPL as GPIO
    P3->SEL1 &= ~0xC0; // 2) make nSLPR & nSLPL as output
    P3->DIR |= 0xC0; // 3) output LOW
    P3->OUT &= ~0xC0;
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;
}
int main(void)
{
    // 0 < speed <10000
    int speed = 1000;
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    while(1){
        // Move Forward
        P5->OUT &= ~0x30; // PH =0 (yes)
        P2->OUT |= 0xC0; // EN = 1
        P3->OUT |= 0xC0; //nSleep=1
        SysTick_Wait1us(speed);
        // Stop
        P2->OUT &= ~0xC0; //EN =0
        SysTick_Wait1us(10000-speed);
    }
}

Moves forward


       
        //Move Forward
        P5->OUT &= ~0x30; // PH =0 (yes)
        P2->OUT |= 0xC0; // EN = 1
        P3->OUT |= 0xC0; //nSleep=1
        SysTick_Wait1us(speed);
        // Stop
        P2->OUT &= ~0xC0; //EN =0
        SysTick_Wait1us(10000-speed);
        }
Modified for pins 7.3, 7.4 reading
while(1){
        // Turn on IR LEDs
        P5->OUT |= 0X08;
        P9->OUT |= 0X04;
        // Make P7.0-P7.7 as output
        P7->DIR = 0xFF;
        //Charges a Capacitor
        P7->OUT = 0xFF;
        // Wait to be fully charged
        Clock_Delay1us(10);
        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        Clock_Delay1us(1000);
        int sensor;
               // Read P7.7 - P7.0 Input
               // white: 0 , Black ; 1
       sensor = P7->IN & 0x18; //change 0x18 to read for all

       if (sensor == 0x18) {
           P2->OUT |= 0x01;
           Move_Forward(speed);
       } else {
           P2->OUT &= ~0x07;
       }
       // Turn off IR LEDs
       P5->OUT &= ~0x08;
       P9->OUT &= ~0x04;
       Clock_Delay1ms(10);
    }


Turn Left or Right

Starting Line
// Check for the starting line before entering the while loop or set a flag that doesn’t check for the starting line once initialized. Need to do the latter and don’t check for the starting line anymore once the robot is running. Or wait at the starting line for 1 second before starting. Then when you encounter the starting line again it ends. Ok lets deal with this later. No need to do this so that you can store variables easier
// Only enter this case if starting_line_flag is not set
if (sensor == 0xFF && !starting_line_flag){
           starting_line_flag = 1;
           Clock_Delay1ms(1000); //Delay for 1 second before starting.
           
           // Record this in an array
           movement_array[0] = "Starting Line";

Initialize IR LEDs
void IR_Init(void){
    // Turn on IR LEDs
    P5->OUT |= 0X08;
    P9->OUT |= 0X04;
    // Make P7.0-P7.7 as output
    P7->DIR = 0xFF;
    //Charges a Capacitor
    P7->OUT = 0xFF;
    // Wait to be fully charged
    Clock_Delay1us(10);
    // Make P7.0-P7.7 as input
    P7->DIR = 0x00;
    // Wait for a while
    Clock_Delay1us(1000);
}

Check for Finishing Line
 else if (sensor == 0xFF && starting_line_flag)
           ending_line_flag = 1;
           movement_array[array_counter] = "Ending Line"



Save 
#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#define LED_RED (1 << 0)
#define LED_GREEN (1 << 1)
#define LED_BLUE (1 << 2)
void SysTick_Init(void) {
    SysTick ->LOAD = 0x00FFFFFF;
    SysTick ->CTRL = 0x00000005;
}
void Led_Init(void) {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x7;
    P2->OUT &= ~0x07;
}
void TurnOn_Led(int color) {
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}
void TurnOff_Led(void) {
    P2->OUT &= ~0x07;
}
void SysTick_Wait1us(int speed) {
    uint32_t ticks = speed * 48;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1ms() {
    SysTick->LOAD = 48000 - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1s() {
    int i, count = 1000;
    for (i=0; i < count; i++){
        SysTick_Wait1ms();
    }
    printf("1s passed...\n");
}
void Motor_Init(void){
    P3->SEL0 &= ~0xC0; // 1) Configure nSLPR & nSLPL as GPIO
    P3->SEL1 &= ~0xC0; // 2) make nSLPR & nSLPL as output
    P3->DIR |= 0xC0; // 3) output LOW
    P3->OUT &= ~0xC0;
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;
}
void Move_Forward(int speed){
    //Move Forward
   P5->OUT &= ~0x30; // PH =0
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
  P2->OUT &= ~0xC0; //EN =0
  SysTick_Wait1us(10000-speed);
}
void Move_Backward(int speed){
    //Move Backward
   P5->OUT &= 0x30; // PH = 1
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Left(int speed){
    //Turn Left
   P5->OUT &= ~0x30; // PH = 1 for left wheel, PH = 0 for right wheel
   P5->OUT &= 0x10; // left DIR
   P2->OUT |= 0xC0; // EN = 0 for left wheel, EN = 1 for right wheel
   P2->OUT &= ~0x80; // left PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Right(int speed){
    //Turn Right
   P5->OUT &= ~0x30; // PH = 0 for left wheel, PH = 1 for right wheel
   P5->OUT &= 0x20; // Right DIR
   P2->OUT |= 0xC0; // EN = 1 for left wheel, EN = 0 for right wheel
   P2->OUT &= ~0x40; // Right PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
    }
int main(void)
{
    // 0 < speed <10000
    int speed = 5000;
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    Led_Init();
    // 0, 2, 4, 6 IR Emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08; //GPIO
    P5->DIR |= 0x08; // OUTPUT
    P5->OUT &= ~0x08;  // turn off 4 even IR LEDS
    // 1,3,5,7 IR Emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04; //GPIO
    P9->DIR |= 0x04; // OUTPUT
    P9->OUT &= ~0x04; // turn off 4 odd IR LEDs
    // 0~7 IR Sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF; //GPIO
    P7->DIR &= ~0xFF; //INPUT
    while(1){
        // Turn on IR LEDs
        P5->OUT |= 0X08;
        P9->OUT |= 0X04;
        // Make P7.0-P7.7 as output
        P7->DIR = 0xFF;
        //Charges a Capacitor
        P7->OUT = 0xFF;
        // Wait to be fully charged
        Clock_Delay1us(10);
        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        Clock_Delay1us(1000);
        int sensor;
               // Read P7.7 - P7.0 Input
               // white: 0 , Black ; 1
       sensor = P7->IN & 0xFF;
       if (sensor == 0x18) {
           //move forward
           P2->OUT |= 0x01;
           Move_Forward(speed);
       } else if ((sensor == 0x10) | (sensor ==0x30  )) {
           //move left
           P2->OUT |= 0x01;
           Turn_Left(speed);
       } else if ((sensor == 0x08) | (sensor == 0x0C )){
           //move right
           P2->OUT |= 0x01;
           Turn_Right(speed);
       } else {
           P2->OUT &= ~0x07; // Off, don't move
       }
       // Turn off IR LEDs
       P5->OUT &= ~0x08;
       P9->OUT &= ~0x04;
       Clock_Delay1ms(10);
    }
}

Timer Initialization
void (*TimerA2Task)(void);
void TimerA2_Init(void(*task)(void), uint16_t period) {
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF0)|0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}
void TA2_0_IRQHandler(void){
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}
void task(){
    printf("interrupt occurs!\n)");
}
int main(void) {
    Clock_Init48MHz();
    TimerA2_Init(&task, 50000);
    while(1){};
}


PWM Using Timer
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
    TIMER_A0->CCTL[0] = 0X800;
    TIMER_A0->CCR[0] = period;
    TIMER_A0->EX0 = 0x0000;
    TIMER_A0->CCTL[3] = 0X0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0X0040;
    TIMER_A0->CCR[4] = duty4;
    TIMER_A0->CTL = 0x02F0;
} // Set Motor's PWM Pin as output && Timer0A instead of GPIO

Set TA0.3 as Toggle/Reset Mode
Set TA0.3 as Up/Down Mode

My Own Implementation
void Move_Forward(int speed){
    //Move Forward
   P5->OUT &= ~0x30; // PH =0
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
  P2->OUT &= ~0xC0; //EN =0
  SysTick_Wait1us(10000-speed);
}
void Move_Backward(int speed){
    //Move Backward
   P5->OUT &= 0x30; // PH = 1
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Left(int speed){
    //Turn Left
   P5->OUT &= ~0x30; // PH = 1 for left wheel, PH = 0 for right wheel
   P5->OUT &= 0x10; // left DIR
   P2->OUT |= 0xC0; // EN = 0 for left wheel, EN = 1 for right wheel
   P2->OUT &= ~0x80; // left PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Right(int speed){
    //Turn Right
   P5->OUT &= ~0x30; // PH = 0 for left wheel, PH = 1 for right wheel
   P5->OUT &= 0x20; // Right DIR
   P2->OUT |= 0xC0; // EN = 1 for left wheel, EN = 0 for right wheel
   P2->OUT &= ~0x40; // Right PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
    }
int main(void)
{
    // 0 < speed <10000
    int speed = 5000;
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    Led_Init();
    // 0, 2, 4, 6 IR Emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08; //GPIO
    P5->DIR |= 0x08; // OUTPUT
    P5->OUT &= ~0x08;  // turn off 4 even IR LEDS
    // 1,3,5,7 IR Emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04; //GPIO
    P9->DIR |= 0x04; // OUTPUT
    P9->OUT &= ~0x04; // turn off 4 odd IR LEDs
    // 0~7 IR Sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF; //GPIO
    P7->DIR &= ~0xFF; //INPUT
    while(1){
        // Turn on IR LEDs
        P5->OUT |= 0X08;
        P9->OUT |= 0X04;
        // Make P7.0-P7.7 as output
        P7->DIR = 0xFF;
        //Charges a Capacitor
        P7->OUT = 0xFF;
        // Wait to be fully charged
        Clock_Delay1us(10);
        // Make P7.0-P7.7 as input
        P7->DIR = 0x00;
        // Wait for a while
        Clock_Delay1us(1000);
        int sensor;
               // Read P7.7 - P7.0 Input
               // white: 0 , Black ; 1
       sensor = P7->IN & 0xFF;
       if (sensor == 0x18) {
           //move forward
           P2->OUT |= 0x01;
           Move_Forward(speed);
       } else if ((sensor == 0x10) | (sensor ==0x30  )) {
           //move left
           P2->OUT |= 0x01;
           Turn_Left(speed);
       } else if ((sensor == 0x08) | (sensor == 0x0C )){
           //move right
           P2->OUT |= 0x01;
           Turn_Right(speed);
       } else {
           P2->OUT &= ~0x07; // Off, don't move
       }
       // Turn off IR LEDs
       P5->OUT &= ~0x08;
       P9->OUT &= ~0x04;
       Clock_Delay1ms(10);
    }
}

Motor Initialization (rev)
void SysTick_Init(void) {
    SysTick ->LOAD = 0x00FFFFFF;
    SysTick ->CTRL = 0x00000005;
}
void Led_Init(void) {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x7;
    P2->OUT &= ~0x07;
}
void TurnOn_Led(int color) {
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}
void TurnOff_Led(void) {
    P2->OUT &= ~0x07;
}
void SysTick_Wait1us(int speed) {
    uint32_t ticks = speed * 48;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1ms(int speed) {
    SysTick->LOAD = (48000 * speed) - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1s() {
    int i, count = 1000;
    for (i=0; i < count; i++){
        SysTick_Wait1ms(1);
    }
    printf("1s passed...\n");
}
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
    TIMER_A0->CCTL[0] = 0X800;
    TIMER_A0->CCR[0] = period;
    TIMER_A0->EX0 = 0x0000;
    TIMER_A0->CCTL[3] = 0X0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0X0040;
    TIMER_A0->CCR[4] = duty4;
    TIMER_A0->CTL = 0x02F0;
} // Set Motor's PWM Pin as output && Timer0A instead of GPIO
void Motor_Init(void){
    P3->SEL0 &= ~0xC0; // 1) Configure nSLPR & nSLPL as GPIO
    P3->SEL1 &= ~0xC0; // 2) make nSLPR & nSLPL as output
    P3->DIR |= 0xC0; // 3) output LOW
    P3->OUT &= ~0xC0;
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;
    // Added PWM_Init for lab 3
    PWM_Init34(15000, 0, 0);
}
void PWM_Duty3(uint16_t duty3){
    TIMER_A0->CCR[3] = duty3;
}
void PWM_Duty4(uint16_t duty4){
    TIMER_A0->CCR[4] = duty4;
}
void Move(uint16_t leftDuty, uint16_t rightDuty){
    P3->OUT |= 0xC0;
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}
void Left_Forward(){
    P5->OUT &= ~0x10;
}
void Left_Backward(){
    P5->OUT |= 0x10;
}
void Right_Forward(){
    P5->OUT &= ~0x20;
}
void Right_Backward(){
    P5->OUT |= 0x20;
}
void Move_Forward(int speed){
    //Move Forward
   P5->OUT &= ~0x30; // PH =0
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
  P2->OUT &= ~0xC0; //EN =0
  SysTick_Wait1us(10000-speed);
}
void Move_Backward(int speed){
    //Move Backward
   P5->OUT &= 0x30; // PH = 1
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Left(int speed){
    //Turn Left
   P5->OUT &= ~0x30; // PH = 1 for left wheel, PH = 0 for right wheel
   P5->OUT &= 0x10; // left DIR
   P2->OUT |= 0xC0; // EN = 0 for left wheel, EN = 1 for right wheel
   P2->OUT &= ~0x80; // left PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Right(int speed){
    //Turn Right
   P5->OUT &= ~0x30; // PH = 0 for left wheel, PH = 1 for right wheel
   P5->OUT &= 0x20; // Right DIR
   P2->OUT |= 0xC0; // EN = 1 for left wheel, EN = 0 for right wheel
   P2->OUT &= ~0x40; // Right PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
    }
void (*TimerA2Task)(void);
void TimerA2_Init(void(*task)(void), uint16_t period) {
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF0)|0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}
void TA2_0_IRQHandler(void){
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}
void task(){
    printf("interrupt occurs!\n");
}
int main(void) {
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
     while(1){
        // Move Forward
        Left_Forward();
        Right_Forward();
        Move(2000, 2000);
        Clock_Delay1ms(2000);
        printf("change side F \n");
        // Move Backward
        Left_Backward();
        Right_Backward();
        Move(2000, 2000);
        Clock_Delay1ms(2000);
        printf("change side B \n");
        // Rotate
        Left_Forward();
        Right_Backward();
        Move(1500, 1500);
        Clock_Delay1ms(2000);
        printf("change side R \n");
    }

Calculate Rotation Duty
Is_Running_Led();
        //Rotate Right/Clockwise
        Rotate_CW(rotation_angle, rotation_duty); // in 1 second
        Clock_Delay1ms(1000);
        Move(0,0);
        Clock_Delay1ms(2000);

// 1040 ms is used for 10000 rotation duty

State that works

    while(1){
	
       array_counter += 1;
       Charge_capacitor();
       int sensor;
       // Read P7.7 - P7.0 Input
       // white: 0 , Black ; 1
       sensor = P7->IN & 0xFF;
       if (sensor == 0xFF && !starting_line_flag){
           printf("At Starting Line \n");
           // Record this in an array
           movement_array[0] = "Starting Line";
           // Begin the first movement
           //Move forward
          Is_Running_Led();
          movement_array[array_counter] = "Forward";
       }
       if (sensor == 0x18) {
           printf("Move Forward \n");
           Is_Running_Led();
           Move_Forward_New(frame_rate, forward_duty);
           // determine distance moved later on **
           Clock_Delay1ms(frame_rate);
           movement_array[array_counter] = "Forward";
       } else if ((sensor == 0x10) | (sensor ==0x30  )) {
           // Alignment
           //Turn left
           printf("Turn Left \n");
           Is_Running_Led();
           // can change to duty later
           Turn_Left_New();
           // need to add it into the system later
       } else if ((sensor == 0x08) | (sensor == 0x0C )){
           // Alignment
           //Turn right
           printf("Turn Right \n");
           Is_Running_Led();
           Turn_Right_New();
           // need to add it into the system later
       } else if (sensor == ~0xFF) {
           printf("Rotating \n");
           //Rotate When sensors detect nothing (Not implemented)
           Is_Running_Led();
           //Rotate Right/Clockwise
           Rotate_CW(rotation_angle, rotation_duty);
           // figure out what rotation_duty I'm using and stay fixed
           Clock_Delay1ms(rotation_angle);
           movement_array[array_counter] = "Rotate 135";
           // ** Need to move back to original position prior to rotating
       } else {
           P2->OUT &= ~0x07; // Off, don't move
           printf("Stopped");
           Move(0,0);
       }
// ------------- Code for finishing Line ------------
       // Finishing Line after delay
       if (sensor == 0xFF && starting_line_flag && (array_counter > 15)) {
                  printf("Finish Line Reached \n");
                  ending_line_flag = 1;
                  movement_array[array_counter] = "Ending Line";
       }
       if (ending_line_flag) {
           // print file
           // *Have to be careful if ending_line_flag is set even if we off the robot
           printf("Movement Array:\n");
           int i = 0;
           for (i; i < array_counter + 1; i++) {
               printf("%s\n", movement_array[i]);
           }
           //return 0;
               }
       // -----------------End of Loops-----------------
       // turn off IR Leds to rest
       IR_off();
    }





// Tasks To be done
Finish detecting Starting Line
Determine determine duty for its angle to turn 360 // 1040 ms for 10000 duty
When sensor detects nothing (then u rotate)
Determine Distance moved // can be stored in seconds and s * duty = distance
We will use 0.5ms 
Set alignment if robots doesn’t drive straight 
Make it turn back and rotate 135 degrees
Path finding algorithm
Either remember the vertices or map the robot path in its mind checking if the coordinate has been around been reached or just hard code it once x amount of rotations are completed.
Implementing graph might be harder, can do the easier one first
Storage of path traversed
Iteration of path found based on command, store it in a variable or output txt.
Possible: speed up then slow down at the end.


Remove starting line flags first
//
           if (sensor == 0xFF && !starting_line_flag){
               printf("At Starting Line \n");
               // Record this in an array
               movement_array[0] = "Starting Line";
               // Begin the first movement
               //Move forward
              Is_Running_Led();
              movement_array[array_counter] = "Forward";
           }

Remove ending line flag phase 3
// Finishing Line after delay
           if (sensor == 0xFF && starting_line_flag && (array_counter > 15)) {
                      printf("Finish Line Reached \n");
                      ending_line_flag = 1;
                      movement_array[array_counter] = "Ending Line";
           }
           // print file
           if (ending_line_flag) {
               // *Have to be careful if ending_line_flag is set even if we off the robot
               printf("Movement Array:\n");
               int i = 0;
               for (i; i < array_counter + 1; i++) {
                   printf("%s\n", movement_array[i]);
               }
               //return 0;
                   }

Everything
(sensor != 0x18) && (sensor != 0x10) && (sensor != 0x30) && (sensor != 0x60) && (sensor != 0x40) && (sensor != 0x08) && (sensor != 0x0C ) && (sensor != 0x06) && (sensor != 0x02) && (sensor != 0x80) && (sensor != 0xC0) && (sensor != 0x01) && (sensor != 0x03)

Turn left
(sensor == 0x10) || (sensor ==0x30) || (sensor == 0x60) || (sensor == 0x40) || (sensor == 0x80) || (sensor == 0xC0) 

Turn Right
(sensor == 0x08) || (sensor == 0x0C ) || (sensor == 0x06) || (sensor == 0x02) || (sensor == 0x01)|| (sensor == 0x03)){

if (flag){
                       //--------Move Forward to reset sensors------//
              printf("Move Forward! \n");
              Move_Forward_New(forward_duty); // 0.05 seconds
              printf("problem 1");
              SysTick_Wait1ms(500);
              printf("problem 2");
              array_counter += 1;
              movement_array[array_counter] = "Move Forward (s)";
              printf("problem 3");
              Stop();
              stopped = 0;
              flag = 0;
                       }


Rotating Count
if (rotating_count >= 7){
               printf('Phase 2 Reached');
               SysTick_Wait1ms(1000);;        //Delay to chill out & signal movement change
               //------------ 2a. First Vertex ------//
               if (stopped) {
                                  //--------Check Sensor--------//
              Clock_Delay1ms(500);
              Charge_capacitor();
              sensor = P7->IN & 0xFF;
              int bms1 ;
              int v1_flag = 0;
              bms1 = sensor & 0xFF;
              if (bms1 == 0)
              {                                     //Change if buggy
                                  //--------Move back------//
              printf("Move Back");
              Move_Backward_New(forward_duty); // 0.25 seconds
              SysTick_Wait1ms(500);
              Stop();
              movement_array[array_counter] = "Move Back (x)";
                                  //--------Rotate 90 degrees CW------//
              if (v1_flag == 0) {
              printf("Rotating 90 \n");
              Rotate_CW(rotation_duty);
              Clock_Delay1ms(210);
              Stop();
              movement_array[array_counter] = "Rotate CW 90";
              v1_flag = 1;
              stopped = 0;
              }
              //--------Rotate 45 degrees CW------//
              if (v1_flag == 1){
                printf("Rotating 45 \n");
                Rotate_CW(rotation_duty);
                Clock_Delay1ms(95);
                Stop();
                stopped = 0;
              }
              }
              IR_off();
           }
        }
Move back

                     printf("Move Back");
                    Move_Backward_New(forward_duty); //
                    Clock_Delay1ms(500);
                    Stop();
                    movement_array[array_counter] = "Move Back (s)";
	
// Moveback 200
                        printf("Move Back");
                        Move_Backward_New(forward_duty); //500
                        Clock_Delay1ms(200);
                        Stop();
                        array_counter += 1
                        movement_array[array_counter] = "Back200";




Moving Straight
if ((bm1 || bm2 )) {
       printf("Move Forward \n");
       Is_Running_Led();
       Move_Forward_New(forward_duty);
       P2->OUT &= ~0x01; //Off LED
       return 1;
   } else if ((sensor == 0x10) || bm3 ){
       // Alignment
       //Turn left
       printf("Turn Left \n");
       Is_Running_Led();
       Turn_Left_New();
       P2->OUT &= ~0x01; //Off LED
       // need to add it into the system later
       return 1;
   } else if ((sensor == 0x08) || bm4 ){
       // Alignment
       //Turn right
       printf("Turn Right \n");
       Is_Running_Led();
       Turn_Right_New();
       P2->OUT &= ~0x01; //Off LED
       // need to add it into the system later
       return 1;
   } else {
    return 0;}
}
Rotating
if ((rotating_count > 7) && stopped){
               printf("Phase 2, rotating_count: %d\n", rotating_count);
             Charge_capacitor();
             sensor = P7->IN & 0xFF;
             int bmx1, bmx2;
             bmx1 = sensor & 0xFF;
             bmx2 = sensor & 0xFF;
             if (bmx1 == 0){
                 if (v1_flag == 0) {
                     printf("Move Back");
                   Move_Backward_New(forward_duty); //
                   Clock_Delay1ms(200);
                   Stop();
                   movement_array[array_counter] = "Move Back (s)";
                  printf("Rotating 90 \n");
                  Rotate_CW(rotation_duty);
                  Clock_Delay1ms(90);
                  movement_array[array_counter] = "Rotate CW 90";
                  v1_flag = 1;
                  stopped = 0;
                  }
             }
             if (bmx2 == 0){
                 if (v1_flag == 1){
                     printf("Rotating 45 \n");
                     Rotate_CW(rotation_duty);
                     Clock_Delay1ms(55);
                     Stop();
                     stopped = 0;
                               }
                 IR_off();
             }


Save State (12.06.24)
#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#include <string.h>
#define LED_RED (1 << 0)
#define LED_GREEN (1 << 1)
#define LED_BLUE (1 << 2)
#define TEST_PHASE 0
void SysTick_Init(void) {
    SysTick ->LOAD = 0x00FFFFFF;
    SysTick ->CTRL = 0x00000005;
}
void Led_Init(void) {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x7;
    P2->OUT &= ~0x07;
}
void TurnOn_Led(int color) {
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}
void TurnOff_Led(void) {
    P2->OUT &= ~0x07;
}
void SysTick_Wait1us(int speed) {
    uint32_t ticks = speed * 48;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1ms(int speed) {
    uint32_t ticks = speed * 48000;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1s() {
    int i, count = 1000;
    for (i=0; i < count; i++){
        SysTick_Wait1ms(1);
    }
    printf("1s passed...\n");
}
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
    TIMER_A0->CCTL[0] = 0X800;
    TIMER_A0->CCR[0] = period;
    TIMER_A0->EX0 = 0x0000;
    TIMER_A0->CCTL[3] = 0X0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0X0040;
    TIMER_A0->CCR[4] = duty4;
    TIMER_A0->CTL = 0x02F0;
} // Set Motor's PWM Pin as output && Timer0A instead of GPIO
void Motor_Init(void){
    P3->SEL0 &= ~0xC0; // 1) Configure nSLPR & nSLPL as GPIO
    P3->SEL1 &= ~0xC0; // 2) make nSLPR & nSLPL as output
    P3->DIR |= 0xC0; // 3) output LOW
    P3->OUT &= ~0xC0;
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;
    // Added PWM_Init for lab 3
    PWM_Init34(15000, 0, 0);
}
void PWM_Duty3(uint16_t duty3){
    TIMER_A0->CCR[3] = duty3;
}
void PWM_Duty4(uint16_t duty4){
    TIMER_A0->CCR[4] = duty4;
}
void Move(uint16_t leftDuty, uint16_t rightDuty){
    P3->OUT |= 0xC0;
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}
void Left_Forward(){
    P5->OUT &= ~0x10;
}
void Left_Backward(){
    P5->OUT |= 0x10;
}
void Right_Forward(){
    P5->OUT &= ~0x20;
}
void Right_Backward(){
    P5->OUT |= 0x20;
}
void Move_Forward(int speed){
    //Move Forward
   P5->OUT &= ~0x30; // PH =0
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
  P2->OUT &= ~0xC0; //EN =0
  SysTick_Wait1us(10000-speed);
}
void Move_Backward(int speed){
    //Move Backward
   P5->OUT &= 0x30; // PH = 1
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Left(int speed){
    //Turn Left
   P5->OUT &= ~0x30; // PH = 1 for left wheel, PH = 0 for right wheel
   P5->OUT &= 0x10; // left DIR
   P2->OUT |= 0xC0; // EN = 0 for left wheel, EN = 1 for right wheel
   P2->OUT &= ~0x80; // left PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Right(int speed){
    //Turn Right
   P5->OUT &= ~0x30; // PH = 0 for left wheel, PH = 1 for right wheel
   P5->OUT &= 0x20; // Right DIR
   P2->OUT |= 0xC0; // EN = 1 for left wheel, EN = 0 for right wheel
   P2->OUT &= ~0x40; // Right PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
    }
void (*TimerA2Task)(void);
void TimerA2_Init(void(*task)(void), uint16_t period) {
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF0)|0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}
void TA2_0_IRQHandler(void){
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}
void task(){
    printf("interrupt occurs!\n");
}
void Charge_capacitor(void){
    // Turn on IR LEDs
    P5->OUT |= 0X08;
    P9->OUT |= 0X04;
    // Make P7.0-P7.7 as output
    P7->DIR = 0xFF;
    //Charges a Capacitor
    P7->OUT = 0xFF;
    // Wait to be fully charged
    Clock_Delay1us(10);
    // Make P7.0-P7.7 as input
    P7->DIR = 0x00;
    // Wait for a while
    Clock_Delay1us(1000);
}
void IR_off(void){
// Turn off IR LEDs
      P5->OUT &= ~0x08;
      P9->OUT &= ~0x04;
      Clock_Delay1ms(10);
}
void IR_Init(void){
    // 0, 2, 4, 6 IR Emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08; //GPIO
    P5->DIR |= 0x08; // OUTPUT
    P5->OUT &= ~0x08;  // turn off 4 even IR LEDS
    // 1,3,5,7 IR Emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04; //GPIO
    P9->DIR |= 0x04; // OUTPUT
    P9->OUT &= ~0x04; // turn off 4 odd IR LEDs
    // 0~7 IR Sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF; //GPIO
    P7->DIR &= ~0xFF; //INPUT
}
void Is_Running_Led(void){
    P2->OUT |= 0x01;
}
void Rotate_CW( int rotation_duty){
    Left_Forward();
    Right_Backward();
    Move(rotation_duty, rotation_duty);
}
void Move_Forward_New(int forward_duty){
    Left_Forward();
    Right_Forward();
    Move(forward_duty, forward_duty); // determine distance moved later on **
}
void Move_Backward_New(int forward_duty){
    Left_Backward();
    Right_Backward();
    Move(forward_duty, forward_duty); // determine distance moved later on **
}
void Turn_Left_New(void){
    Left_Forward();
    Right_Forward();
    Move(5000, 5500); // determine distance moved later on ** 6500 6000, 5000, 5400
     //50
}
void Turn_Right_New(void){
    Left_Forward();
    Right_Forward();
    Move(5500,5000); // determine distance moved later on ** //init 2000;
}
int Moving_Straight_Sequence(int sensor, int forward_duty){
    int bm1, bm2, bm3, bm4, bm5, bm6;
    bm1 = sensor & 0x10;
    bm2 = sensor & 0x08;
    bm3 = sensor & 0x20;
    bm4 = sensor & 0x04;
    bm5 = sensor & 0x60;
    bm6 = sensor & 0x06;
    if ((bm1 && bm2 )) {
       printf("Move Forward \n");
       Is_Running_Led();
       Move_Forward_New(forward_duty);
       P2->OUT &= ~0x01; //Off LED
       return 1;
   } else if (bm1 || bm3 || bm5 ){
       printf("Turn Left \n");
       Is_Running_Led();
               Turn_Left_New();
               Clock_Delay1ms(100);
       P2->OUT &= ~0x01; //Off LED
       return 1;
   } else if (bm2 || bm4 || bm6){
       printf("Turn Right \n");
       Is_Running_Led();
        Turn_Right_New();
        Clock_Delay1ms(100);
       P2->OUT &= ~0x01; //Off LED
       return 1;
   }
    else {
    return 0;}
}
void Stop(void) {
    P2->OUT &= ~0x07; // Off LED
    // Set the enable pins to low
    P2->OUT &= ~0xC0; // Assuming P2.6 and P2.7 are the enable pins
    // Set PWM duty cycles to zero
    PWM_Duty3(0);
    PWM_Duty4(0);
    printf("Stopped\n");
    }
// ---------------  Create an Output File with our collected data ------------------------
void CreateOutputFolder() {
    struct stat st = {0};
    if (stat("output", &st) == -1) {
        mkdir("output", 0700);
    }
}
void WriteMovementArrayToFile() {
    // Open the file for writing
    FILE *file = fopen("output/movement_log.txt", "w");
    if (file == NULL) {
        printf("Error opening file!\n");
        return;
    }
    // Write each element of the movement_array to the file
    for (int i = 0; i < array_counter; i++) {
        if (movement_array[i] != NULL) {
            fprintf(file, "%s\n", movement_array[i]);
        }
    }
    // Close the file
    fclose(file);
    printf("Movement log has been written to output/movement_log.txt\n");
}
int main() {
    // Populate the movement_array with some test data for demonstration
    movement_array[array_counter++] = "Starting Line";
    movement_array[array_counter++] = "Move Forward";
    movement_array[array_counter++] = "Turn Left";
    movement_array[array_counter++] = "Move Forward";
    movement_array[array_counter++] = "Finish Line";
    // Create output folder
    CreateOutputFolder();
    // Assuming the robot finishes executing here
    WriteMovementArrayToFile();
    return 0;
}
int main(void) {
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    IR_Init();
    Led_Init();
    int start_line_flag = 0;
    int ending_line_flag = 0;
    int array_counter = 0; // 0 is allocated to starting line but counter is called once per loop
    int forward_duty = 5000;  // 0 to 65535 // init 2000
    int rotation_duty = 10000; // init 1500
    int frame_rate = 50; // in ms, e.g. 500 ms = 0.5 seconds, 250ms = 0.25 seconds
    int rotation_angle = 520; // in ms, precalculated based on duty X 1040
    int rotating_count = 0;
    int stopped;
    int v1_flag = 0;
    const char *movement_array[10000]; //Static array for simple tests
    while(1){
           //printf("----\n");
           array_counter += 1;
           Charge_capacitor();
           int sensor;
           sensor = P7->IN & 0xFF; // Read P7.7 - P7.0 Input // white: 0 , Black ; 1
           //------ Phase 0: Implement starting Line flag------//
           // Have to implement pause if not the following function will start rotating
           // ** problem im facing dragging identify where the pause is from
                   //---------Robot on starting line for 2.5 seconds--------
           if (start_line_flag == 0){       //Once set, don't enter
               if (sensor == 0xFF){
                   IR_off();
                   printf("? \n");
                   Clock_Delay1ms(2000);
                   printf("checking... ...");
                   Charge_capacitor();
                   sensor = P7->IN & 0xFF;
                            if (sensor == 0xFF){
                                printf("Starting line\n");
                                movement_array[0] = "Starting Line";
                                start_line_flag = 1;
                                // ---- Move Forward after confirming it is true ---//
                                printf("Moving Forward\n");
                                Move_Forward_New(5000);
                                array_counter += 1;
                                movement_array[array_counter] = "Move Forward (sl)";
                                Clock_Delay1ms(500);
                                printf("Pausing...\n");
                                Stop();
                            }
                   }
           }
           //------ Phase 1: Go Forward/ Keep Alignment/ Rotate------//
                       //---------- Execute once "Starting Line" exists ------//
           if (start_line_flag){ //                                                     //strcmp(movement_array[0], "Starting Line") == 0
                       //---------- 1a. Go forward & Keep Alignment ----------//
           Charge_capacitor();
          int sensor;
          sensor = P7->IN & 0xFF;
           int Is_Running;
           Is_Running = Moving_Straight_Sequence( sensor, forward_duty); // Will terminate when it detects nothing
           if (Is_Running){ // checks if 0 or 1
               movement_array[array_counter] = "Forward";
               Clock_Delay1ms(140); //200
               stopped = 0;
           } else {
               Stop();
               stopped = 1;
           }
           IR_off();
                       //---------- 1b. Rotate ----------//
           if (rotating_count < 8){
           if (stopped) {
               printf("rotating_count: %d\n", rotating_count);
                                   //--------Check Sensor--------//
               Clock_Delay1ms(500);
               Charge_capacitor();
               sensor = P7->IN & 0xFF;
               int bms1 ;
               bms1 = sensor & 0xFF;
               if (bms1 == 0)
               {                                     //Change if buggy
                                   //--------Move back------//
                                   //--------Rotate 135 degrees CW------//
               printf("Rotating \n");
               printf("Array Counter : %d\n", array_counter);
               Rotate_CW(rotation_duty);
               Clock_Delay1ms(335); //490
               Stop();
               rotating_count += 1;
               array_counter += 1;
               movement_array[array_counter] = "Rotate CW 135";
               stopped = 0;
               printf("Turn Right!!");
               Turn_Right_New(); //
               Clock_Delay1ms(505);
               Stop();
               movement_array[array_counter] = "Move Forward (s)";
                    }
               IR_off();
               }
           }else {
                   // -------------------Phase 2: When rotation hits 7, start tracing the polygon-----------------
                  printf("Phase 2, rotating_count: %d\n", rotating_count);
                    Charge_capacitor();
                    sensor = P7->IN & 0xFF;
                    int bmx1, bmx2;
                    bmx1 = sensor & 0xFF;
                    bmx2 = sensor & 0xFF;
                    if (rotating_count == 8){
                        // Moveback 200
                        printf("Move Back");
                        Move_Backward_New(forward_duty); //
                        Clock_Delay1ms(800);
                        Stop();
                        movement_array[array_counter] = "Move Back (s)";
                         printf("Rotating 90 \n");
                         Rotate_CW(rotation_duty);
                         Clock_Delay1ms(40);
                         movement_array[array_counter] = "Rotate CW 90";
                         stopped = 0;
                    } else if (rotating_count == 15){
                        printf("Rotating 270 \n");
                                                 Rotate_CW(rotation_duty);
                                                 Clock_Delay1ms(40);
                                                 movement_array[array_counter] = "Rotate CW 90";
                                                 stopped = 0;
                    } else {
                        if (bmx1){
                            printf("Rotating 45 \n");
                         Rotate_CW(rotation_duty);
                         Clock_Delay1ms(55);
                         movement_array[array_counter] = "Rotate CW 45";
                         stopped = 0;
                    }}
                    rotating_count += 1;
                    }
           // -------------------Phase 3: Reach Finish Line-----------------//
               IR_off();
           }
            // *
//------ end of code-------
    }
}


Save State Final
#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <string.h>
#define LED_RED (1 << 0)
#define LED_GREEN (1 << 1)
#define LED_BLUE (1 << 2)
#define TEST_PHASE 0
void SysTick_Init(void) {
    SysTick ->LOAD = 0x00FFFFFF;
    SysTick ->CTRL = 0x00000005;
}
void Led_Init(void) {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x7;
    P2->OUT &= ~0x07;
}
void TurnOn_Led(int color) {
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}
void TurnOff_Led(void) {
    P2->OUT &= ~0x07;
}
void SysTick_Wait1us(int speed) {
    uint32_t ticks = speed * 48;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1ms(int speed) {
    uint32_t ticks = speed * 48000;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000)==0){};
}
void SysTick_Wait1s() {
    int i, count = 1000;
    for (i=0; i < count; i++){
        SysTick_Wait1ms(1);
    }
    printf("1s passed...\n");
}
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
    TIMER_A0->CCTL[0] = 0X800;
    TIMER_A0->CCR[0] = period;
    TIMER_A0->EX0 = 0x0000;
    TIMER_A0->CCTL[3] = 0X0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0X0040;
    TIMER_A0->CCR[4] = duty4;
    TIMER_A0->CTL = 0x02F0;
} // Set Motor's PWM Pin as output && Timer0A instead of GPIO
void Motor_Init(void){
    P3->SEL0 &= ~0xC0; // 1) Configure nSLPR & nSLPL as GPIO
    P3->SEL1 &= ~0xC0; // 2) make nSLPR & nSLPL as output
    P3->DIR |= 0xC0; // 3) output LOW
    P3->OUT &= ~0xC0;
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;
    // Added PWM_Init for lab 3
    PWM_Init34(15000, 0, 0);
}
void PWM_Duty3(uint16_t duty3){
    TIMER_A0->CCR[3] = duty3;
}
void PWM_Duty4(uint16_t duty4){
    TIMER_A0->CCR[4] = duty4;
}
void Move(uint16_t leftDuty, uint16_t rightDuty){
    P3->OUT |= 0xC0;
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}
void Left_Forward(){
    P5->OUT &= ~0x10;
}
void Left_Backward(){
    P5->OUT |= 0x10;
}
void Right_Forward(){
    P5->OUT &= ~0x20;
}
void Right_Backward(){
    P5->OUT |= 0x20;
}
void Move_Forward(int speed){
    //Move Forward
   P5->OUT &= ~0x30; // PH =0
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
  P2->OUT &= ~0xC0; //EN =0
  SysTick_Wait1us(10000-speed);
}
void Move_Backward(int speed){
    //Move Backward
   P5->OUT &= 0x30; // PH = 1
   P2->OUT |= 0xC0; // EN = 1
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Left(int speed){
    //Turn Left
   P5->OUT &= ~0x30; // PH = 1 for left wheel, PH = 0 for right wheel
   P5->OUT &= 0x10; // left DIR
   P2->OUT |= 0xC0; // EN = 0 for left wheel, EN = 1 for right wheel
   P2->OUT &= ~0x80; // left PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
}
void Turn_Right(int speed){
    //Turn Right
   P5->OUT &= ~0x30; // PH = 0 for left wheel, PH = 1 for right wheel
   P5->OUT &= 0x20; // Right DIR
   P2->OUT |= 0xC0; // EN = 1 for left wheel, EN = 0 for right wheel
   P2->OUT &= ~0x40; // Right PWM
   P3->OUT |= 0xC0; //nSleep=1
   SysTick_Wait1us(speed);
   // Stop
   P2->OUT &= ~0xC0; //EN =0
   SysTick_Wait1us(10000-speed);
    }
void (*TimerA2Task)(void);
void TimerA2_Init(void(*task)(void), uint16_t period) {
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF0)|0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}
void TA2_0_IRQHandler(void){
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}
void task(){
    printf("interrupt occurs!\n");
}
void Charge_capacitor(void){
    // Turn on IR LEDs
    P5->OUT |= 0X08;
    P9->OUT |= 0X04;
    // Make P7.0-P7.7 as output
    P7->DIR = 0xFF;
    //Charges a Capacitor
    P7->OUT = 0xFF;
    // Wait to be fully charged
    Clock_Delay1us(10);
    // Make P7.0-P7.7 as input
    P7->DIR = 0x00;
    // Wait for a while
    Clock_Delay1us(1000);
}
void IR_off(void){
// Turn off IR LEDs
      P5->OUT &= ~0x08;
      P9->OUT &= ~0x04;
      Clock_Delay1ms(10);
}
void IR_Init(void){
    // 0, 2, 4, 6 IR Emitter
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08; //GPIO
    P5->DIR |= 0x08; // OUTPUT
    P5->OUT &= ~0x08;  // turn off 4 even IR LEDS
    // 1,3,5,7 IR Emitter
    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04; //GPIO
    P9->DIR |= 0x04; // OUTPUT
    P9->OUT &= ~0x04; // turn off 4 odd IR LEDs
    // 0~7 IR Sensor
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF; //GPIO
    P7->DIR &= ~0xFF; //INPUT
}
void Is_Running_Led(void){
    P2->OUT |= 0x01;
}
void Rotate_CW( int rotation_duty){
    Left_Forward();
    Right_Backward();
    Move(rotation_duty, rotation_duty);
}
void Move_Forward_New(int forward_duty){
    Left_Forward();
    Right_Forward();
    Move(forward_duty, forward_duty); // determine distance moved later on **
}
void Move_Backward_New(int forward_duty){
    Left_Backward();
    Right_Backward();
    Move(forward_duty, forward_duty); // determine distance moved later on **
}
void Turn_Left_New(void){
    Left_Forward();
    Right_Forward();
    Move(5000, 5500); // determine distance moved later on ** 6500 6000, 5000, 5400
     //50
}
void Turn_Right_New(void){
    Left_Forward();
    Right_Forward();
    Move(5500,5000); // determine distance moved later on ** //init 2000;
}
int Moving_Straight_Sequence(int sensor, int forward_duty){
    int bm1, bm2, bm3, bm4, bm5, bm6;
    bm1 = sensor & 0x10;
    bm2 = sensor & 0x08;
    bm3 = sensor & 0x20;
    bm4 = sensor & 0x04;
    bm5 = sensor & 0x60;
    bm6 = sensor & 0x06;
    if ((bm1 && bm2 )) {
       printf("Move Forward \n");
       Is_Running_Led();
       Move_Forward_New(forward_duty);
       P2->OUT &= ~0x01; //Off LED
       return 1;
   } else if (bm1 || bm3 || bm5 ){
       printf("Turn Left \n");
       Is_Running_Led();
               Turn_Left_New();
               Clock_Delay1ms(100);
       P2->OUT &= ~0x01; //Off LED
       return 1;
   } else if (bm2 || bm4 || bm6){
       printf("Turn Right \n");
       Is_Running_Led();
        Turn_Right_New();
        Clock_Delay1ms(100);
       P2->OUT &= ~0x01; //Off LED
       return 1;
   }
    else {
    return 0;}
}
void Stop(void) {
    P2->OUT &= ~0x07; // Off LED
    // Set the enable pins to low
    P2->OUT &= ~0xC0; // Assuming P2.6 and P2.7 are the enable pins
    // Set PWM duty cycles to zero
    PWM_Duty3(0);
    PWM_Duty4(0);
    printf("Stopped\n");
    }
//------------------------------- Define a structure to hold the text part and the number part
typedef struct {
    char textPart[100];
    int numberPart;
} SplitResult;
//------------------------------- Function to split a string containing text and numbers
SplitResult SplitStringAndNumber(const char *input) {
    SplitResult result;
    int i = 0;
    int j = 0;
    int len = strlen(input);
    // Initialize the structure members
    result.numberPart = 0;
    memset(result.textPart, 0, sizeof(result.textPart));
    // Loop through the input string
    for (i = 0; i < len; i++) {
        // If the character is a digit, break the loop
        if (isdigit(input[i])) {
            break;
        }
        // Append the character to the text part
        result.textPart[j++] = input[i];
    }
    // Loop through the remaining part of the string to extract the number
    for (; i < len; i++) {
        if (isdigit(input[i])) {
            result.numberPart = (result.numberPart * 10) + (input[i] - '0');
        }
    }
    return result;
}
void Execute_Track_Reversal(const char **movement_array){
    // Call the function to split the string and number
    int i = 0;
    int array_size =10000;
    for (i=0; i < array_size; i++) {
            if (movement_array[i] != NULL) { // Check if the entry is not NULL
                SplitResult result = SplitStringAndNumber(movement_array[i]);
                // Print the results
                printf("Text: %s\n", result.textPart);
                printf("Number: %d\n", result.numberPart);
                if (strcmp(result.textPart, "Forward") == 0) {
                    printf("Moving Forward\n");
                    Move_Forward_New(5000);
                } else if (strcmp(result.textPart, "Left") == 0) {
                    printf("Moving Left\n");
                    Turn_Left_New();
                } else if (strcmp(result.textPart, "Right") == 0) {
                    printf("Moving Right\n");
                    Turn_Right_New();
                } else if (strcmp(result.textPart, "Back") == 0) {
                    printf("Moving Back\n");
                    Move_Backward_New(5000);
                } else if (strcmp(result.textPart, "Rotate") == 0) {
                    printf("Rotating\n");
                    Clock_Delay1ms(500);
                    Rotate_CW(10000);
                } else if (strcmp(result.textPart, "Stop") == 0){
                    printf("Stopping\n");
                    Stop();
                    Clock_Delay1ms(10000); //Stop for 10 seconds
                }
                Clock_Delay1ms(result.numberPart);//time
            }
        }
}
int main(void) {
    Clock_Init48MHz();
    Motor_Init();
    SysTick_Init();
    IR_Init();
    Led_Init();
    int start_line_flag = 0;
    int ending_line_flag = 0;
    int array_counter = 0; // 0 is allocated to starting line but counter is called once per loop
    int forward_duty = 5000;  // 0 to 65535 // init 2000
    int rotation_duty = 10000; // init 1500
    int frame_rate = 50; // in ms, e.g. 500 ms = 0.5 seconds, 250ms = 0.25 seconds
    int rotation_angle = 520; // in ms, precalculated based on duty X 1040
    int rotating_count = 0;
    int stopped;
    int v1_flag = 0;
    const char *movement_array[10000]; //Static array for simple tests
    while(1){
           //printf("----\n");
           Charge_capacitor();
           int sensor;
           sensor = P7->IN & 0xFF; // Read P7.7 - P7.0 Input // white: 0 , Black ; 1
           //------ Phase 0: Implement starting Line flag------//
           // Have to implement pause if not the following function will start rotating
           // ** problem im facing dragging identify where the pause is from
                   //---------Robot on starting line for 2.5 seconds--------
           if (start_line_flag == 0){       //Once set, don't enter
               if (sensor == 0xFF){
                   IR_off();
                   printf("? \n");
                   Clock_Delay1ms(2000);
                   printf("checking... ...");
                   Charge_capacitor();
                   sensor = P7->IN & 0xFF;
                            if (sensor == 0xFF){
                                printf("Starting line\n");
                                movement_array[0] = "Starting Line";
                                start_line_flag = 1;
                                // ---- Move Forward after confirming it is true ---//
                                printf("Moving Forward\n");
                                Move_Forward_New(5000);
                                array_counter += 1;
                                movement_array[array_counter] = "Forward500";
                                Clock_Delay1ms(500);
                                printf("Pausing...\n");
                                Stop();
                            }
                   }
           }
           //------ Phase 1: Go Forward/ Keep Alignment/ Rotate------//
                       //---------- Execute once "Starting Line" exists ------//
           if (start_line_flag){ //                                                     //strcmp(movement_array[0], "Starting Line") == 0
                       //---------- 1a. Go forward & Keep Alignment ----------//
           Charge_capacitor();
           int sensor;
           sensor = P7->IN & 0xFF;
           int Is_Running;
           Is_Running = Moving_Straight_Sequence( sensor, forward_duty); // Will terminate when it detects nothing
           if (Is_Running){ // checks if 0 or 1
               array_counter += 1;
               movement_array[array_counter] = "Forward140";
               Clock_Delay1ms(140); //200
               stopped = 0;
           } else {
               Stop();
               stopped = 1;
           }
           IR_off();
                       //---------- 1b. Rotate ----------//
           if (rotating_count < 8){
           if (stopped) {
               printf("rotating_count: %d\n", rotating_count);
                                   //--------Check Sensor--------//
               Clock_Delay1ms(500);
               Charge_capacitor();
               sensor = P7->IN & 0xFF;
               int bms1 ;
               bms1 = sensor & 0xFF;
               if (bms1 == 0)
               {                                     //Change if buggy
                                   //--------Move back------//
                                   //--------Rotate 135 degrees CW------//
               printf("Rotating \n");
               printf("Array Counter : %d\n", array_counter);
               Rotate_CW(rotation_duty);
               Clock_Delay1ms(335); //490
               Stop();
               rotating_count += 1;
               array_counter += 1;
               movement_array[array_counter] = "Rotate335";
               stopped = 0;
               printf("Turn Right!!");
               Turn_Right_New(); //
               Clock_Delay1ms(505);
               Stop();
               array_counter += 1;
               movement_array[array_counter] = "Right505";
                    }
               IR_off();
               }
           }else {
                   // -------------------Phase 2: When rotation hits 7, start tracing the polygon-----------------
                  printf("Phase 2, rotating_count: %d\n", rotating_count);
                  Clock_Delay1ms(500);
                    Charge_capacitor();
                    sensor = P7->IN & 0xFF;
                    int bmx1, bmx2;
                    bmx1 = sensor & 0xFF;
                    bmx2 = sensor & 0xFF;
                    if (rotating_count == 8){
                        // Moveback removed
                         printf("Rotating CW 90 \n");
                         Rotate_CW(rotation_duty);
                         Clock_Delay1ms(40);
                         array_counter += 1;
                         movement_array[array_counter] = "Rotate40";
                         stopped = 0;
                    } else if (rotating_count == 15){
                        printf("Rotating 270 \n");
                         Rotate_CW(rotation_duty);
                         Clock_Delay1ms(670);
                         array_counter += 1;
                         movement_array[array_counter] = "Rotate670";
                         stopped = 0;
                         //Execute end sequence
                         Stop();
                         Clock_Delay1ms(30000);
                         array_counter += 1;
                         movement_array[array_counter] = "Stop10000";
                         Execute_Track_Reversal(movement_array);
                    } else {
                          printf("Rotating 45 \n");
                         Rotate_CW(rotation_duty);
                         Clock_Delay1ms(55);
                         array_counter += 1;
                         movement_array[array_counter] = "Rotate55";
                         stopped = 0;
                         }
                    rotating_count += 1;
                    }
           // -------------------Phase 3: Reach Finish Line-----------------//
               IR_off();
           }
            // *
           // ---------------------Phase 4: Read the output and run it --------------//
           int condition = 0;
           if (condition){
               Execute_Track_Reversal(movement_array);
           }
//------ end of code-------
    }
}


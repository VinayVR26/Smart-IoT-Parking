// REMOVE coord
// make priority for interrupts and task same
// Servo change to up counting.

// Ultrasonic gets cut off by the PORTC_PORTD_interrupt

/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271_Assignment.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <string.h>
/* TODO: insert other include files here. */
/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

// Ultrasonic
#define TRIGGER_PIN 1 // PTC1
#define ECHO_PIN 12 // PTA12
#define BUZZER_PIN 3 // PTB3
#define HALL_PIN 4 // PTD4
#define GATE_PIN 5 // PTA5 - Timer 0 channel 2
#define ROOF_PIN 19 // PTB19 - Timer 2 Channel 1
#define IR_PIN 6 // PD6
#define BTN_PIN 2 //PTB2


#define BAUD_RATE 1200
#define UART_TX_PTE22 	22
#define UART_RX_PTE23 	23
#define UART2_INT_PRIO	128 // interrupt priority


TaskHandle_t ultrasonic_task_handle;
TaskHandle_t buzzer_task_handle;
TaskHandle_t gate_task_handle;



void initUltrasonicPins() {
	NVIC_DisableIRQ(PORTA_IRQn);
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; // (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTE_MASK)
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

	PORTC->PCR[TRIGGER_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[TRIGGER_PIN] |= PORT_PCR_MUX(1);

	PORTA->PCR[ECHO_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[ECHO_PIN] |= PORT_PCR_MUX(1);

	GPIOC->PDDR |=  (1 << TRIGGER_PIN); // TRIGGER PIN is output
	GPIOA->PDDR &=  ~(1 << ECHO_PIN); // ECHO PIN is input

	// Can comment out this
	/*PORTA->PCR[ECHO_PIN] &= ~PORT_PCR_PS_MASK;
	PORTA->PCR[ECHO_PIN] |= PORT_PCR_PS(0); // Activate pull-down resistor for ECHO_PIN means it is LOW until an event drives it HIGH
	PORTA->PCR[ECHO_PIN] &= ~PORT_PCR_PE_MASK;
	PORTA->PCR[ECHO_PIN] |= PORT_PCR_PE(1);*/

	PORTA->PCR[ECHO_PIN] &= ~PORT_PCR_IRQC_MASK;
	PORTA->PCR[ECHO_PIN] |= PORT_PCR_IRQC(0b1011); // trigger on rising and falling edge (LOW TO HIGH, HIGH TO LOW)
	// 1011 means either edge
	// 1010 means falling edge
}

void initBuzzerPin() {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[BUZZER_PIN] |= PORT_PCR_MUX(1);

	GPIOB->PDDR |= (1 << BUZZER_PIN); // configure BUZZER_PIN as output
}

void initButtonPin() {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB->PCR[BTN_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[BTN_PIN] |= PORT_PCR_MUX(1);

	GPIOE->PDDR &= ~(1 << BTN_PIN);

	PORTB->PCR[BTN_PIN] &= ~PORT_PCR_PS_MASK;
	PORTB->PCR[BTN_PIN] |= PORT_PCR_PS(1); // Activate pull-up resistor for HALL_PIN means it is HIGH until pressed then it becomes LOW
	PORTB->PCR[BTN_PIN] &= ~PORT_PCR_PE_MASK;
	PORTB->PCR[BTN_PIN] |= PORT_PCR_PE(1);
}

void initHallPin() {
	NVIC_DisableIRQ(PORTC_PORTD_IRQn);
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

	PORTD->PCR[HALL_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[HALL_PIN] |= PORT_PCR_MUX(1); //  set HALL_PIN as GPIO pin

	GPIOD->PDDR &= ~(1 << HALL_PIN); // configure HALL_PIN as input

	PORTD->PCR[HALL_PIN] &= ~PORT_PCR_PS_MASK;
	PORTD->PCR[HALL_PIN] |= PORT_PCR_PS(1); // Activate pull-up resistor for HALL_PIN means it is HIGH until a magnet drives it LOW
	PORTD->PCR[HALL_PIN] &= ~PORT_PCR_PE_MASK;
	PORTD->PCR[HALL_PIN] |= PORT_PCR_PE(1);

	PORTD->PCR[HALL_PIN] &= ~PORT_PCR_IRQC_MASK;
	PORTD->PCR[HALL_PIN] |= PORT_PCR_IRQC(0b1010); // Trigger on falling edge. When Magent comes, the pin goes from HIGH to LOW

	// Enable semaphore (held by interrupt)/ busy wait
}

void initIRPin() {
	PORTD->PCR[IR_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[IR_PIN] |= PORT_PCR_MUX(1);
	GPIOD->PDDR &= ~(1 << IR_PIN);


	PORTD->PCR[IR_PIN] &= ~PORT_PCR_PS_MASK;
	PORTD->PCR[IR_PIN] |= PORT_PCR_PS(1); // Activate pull-up resistor for IR_PIN means it is HIGH until a car drives it LOW
	PORTD->PCR[IR_PIN] &= ~PORT_PCR_PE_MASK;
	PORTD->PCR[IR_PIN] |= PORT_PCR_PE(1);

	PORTD->PCR[IR_PIN] &= ~PORT_PCR_IRQC_MASK;
	PORTD->PCR[IR_PIN] |= PORT_PCR_IRQC(0b1010); // Trigger on falling edge. When car comes, the pin goes from HIGH to LOW
}


void initGatePin() {
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	PORTA->PCR[GATE_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GATE_PIN] |= PORT_PCR_MUX(0b11); // Choose ALT3 = PWM
	//GPIOA->PDDR |= (1 << GATE_PIN); // set the pin as output
}

void initRoofPin() {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB->PCR[ROOF_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[ROOF_PIN] |= PORT_PCR_MUX(0b11); // Choose ALT3 = PWM
}


void initClock() {
	MCG->C1 &= ~MCG_C1_CLKS_MASK;
	MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

	MCG->C2 &= ~MCG_C2_IRCS_MASK; // 2Mhz clock NOT |= ~MCG_C2_IRCS_MASK; cos it makes all other bits 0
	MCG->C2 |= MCG_C2_IRCS(0b0);

	MCG->SC &= ~MCG_SC_FCRDIV_MASK;
	MCG->SC |= MCG_SC_FCRDIV(0b0); // divide by 1

	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
	MCG->MC |= MCG_MC_LIRC_DIV2(0b0); //  divide by 1 to get 2Mhz
}

void initTimer() {
	NVIC_DisableIRQ(TPM1_IRQn);

	initClock();

	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; // enable Timer 1

	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; // clear bits
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11); // MCGIRCLK chosen

	TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK); // Off timer and clear prescaler mask

	//TPM1->SC |= TPM_SC_PS(0b010); // to reduce the tick rate
	TPM1->CNT = 0; // set count to 0
}

void initTimer0() {
	NVIC_DisableIRQ(TPM0_IRQn);

	// clock initialised by function above

	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // enable Timer 0


	// THESE 2 lines are in initTimer()
	//SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; // clear bits
	//SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11); // MCGIRCLK chosen


	TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
	TPM0->SC &= ~TPM_SC_TOIE_MASK; // DISABLE INTERRUPTS

	//TPM0->SC |= TPM_SC_CPWMS_MASK; // centre-aligned
	TPM0->SC &= ~TPM_SC_CPWMS_MASK; // up counting
	TPM0->CNT = 0;

	TPM0->MOD = 39999;

	TPM0->CONTROLS[2].CnV = 2000; // starting position

	//  TPM0 Channel 2

	TPM0->CONTROLS[2].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK);
	TPM0->CONTROLS[2].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1));
}

void initTimer2() {
	NVIC_DisableIRQ(TPM2_IRQn);

	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK; // enable Timer 2


	// THESE 2 lines are in initTimer()
	//SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; // clear bits
	//SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11); // MCGIRCLK chosen


	TPM2->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
	TPM2->SC &= ~TPM_SC_TOIE_MASK; // DISABLE INTERRUPTS

	//TPM0->SC |= TPM_SC_CPWMS_MASK; // centre-aligned
	TPM2->SC &= ~TPM_SC_CPWMS_MASK; // up counting
	TPM2->CNT = 0;

	TPM2->MOD = 39999;

	TPM2->CONTROLS[1].CnV = 4000; // starting position

	//  TPM2 Channel 1

	TPM2->CONTROLS[1].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK);
	TPM2->CONTROLS[1].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1));

}




void initUART2(uint32_t baud_rate)
{
	NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

	//enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	//Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	//connect UART pins for PTE22, PTE23
	PORTE->PCR[UART_TX_PTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PTE22] |= PORT_PCR_MUX(4);

	PORTE->PCR[UART_RX_PTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PTE23] |= PORT_PCR_MUX(4);

	// Set the baud rate
	uint32_t bus_clk = CLOCK_GetBusClkFreq();

	// This version of sbr does integer rounding.
	uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

	// Set SBR. Bits 8 to 12 in BDH, 0-7 in BDL.
	// MUST SET BDH FIRST!
	UART2->BDH &= ~UART_BDH_SBR_MASK;
	UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
	UART2->BDL = (uint8_t) (sbr &0xFF);

	// Disable loop mode
	UART2->C1 &= ~UART_C1_LOOPS_MASK;
	UART2->C1 &= ~UART_C1_RSRC_MASK;

	// Disable parity
	UART2->C1 &= ~UART_C1_PE_MASK;

	// 8-bit mode
	UART2->C1 &= ~UART_C1_M_MASK;

	//Enable RX interrupt
	UART2->C2 |= UART_C2_RIE_MASK;

	// Enable the receiver
	UART2->C2 |= UART_C2_RE_MASK;

	NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
	NVIC_EnableIRQ(UART2_FLEXIO_IRQn);

}


void enableInterrupts() {
	NVIC_SetPriority(PORTA_IRQn, 128);
	NVIC_SetPriority(PORTC_PORTD_IRQn, 192);
	NVIC_SetPriority(UART2_FLEXIO_IRQn, 128); // lab 7 was 128

	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
	NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

void startTimer() {
	TPM1->SC |= TPM_SC_CMOD(0b1);
}

void stopTimer() {
	TPM1->SC &= ~TPM_SC_CMOD_MASK;
	TPM1->CNT = 0;
}

void startPWM() {
	TPM0->SC |= TPM_SC_CMOD(0b1);
}

void stopPWM() {
	TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

void startRoofPWM() {
	TPM2->SC |= TPM_SC_CMOD(0b1);
}

void stopRoofPWM() {
	TPM2->SC &= ~TPM_SC_CMOD_MASK;
}




int start_count;
int end_count;

SemaphoreHandle_t sema;
SemaphoreHandle_t tx;


void PORTA_IRQHandler() {
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	//PRINTF("ENTERED INTERRUPT\r\n");

	if(PORTA->ISFR & (1 << ECHO_PIN)) {
		PORTA->ISFR |= (1 << ECHO_PIN);
		if(GPIOA->PDIR & (1 << ECHO_PIN)) { // Rising edge
			start_count = TPM1->CNT;
		} else if (!(GPIOA->PDIR & (1 << ECHO_PIN))) {
			end_count = TPM1->CNT;
			BaseType_t hpw = pdFALSE;
			xSemaphoreGiveFromISR(sema, &hpw);
			portYIELD_FROM_ISR(hpw);
		}
	}
}

volatile int open = 1; // 1 means next event that will happen is the gate will be triggered to open.
// 0 means the next event that will happen is the gate will be triggered to close

static int count = 0;

void PORTC_PORTD_IRQHandler() {
	NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

	if (open && (PORTD->ISFR & (1 << HALL_PIN))) {

		TPM0->CONTROLS[2].CnV = 4000;
		open = 0;

	} else if (!open && (PORTD->ISFR & (1 << IR_PIN))) { // IR_PIN

		TPM0->CONTROLS[2].CnV = 2000;
		open = 1;
		count = count + 1;
	}
	PORTD->ISFR |= (1 << HALL_PIN);
	PORTD->ISFR |= (1 << IR_PIN);
}


volatile int distance = 275;
static void ultrasonic_task(void *p) {

	while(1){

		PRINTF("ultrasonic_task\r\n");
		GPIOC->PSOR |= (1 << TRIGGER_PIN); // set TRIGGER PIN TO HIGH
		TPM1->CNT = 0;
		while (TPM1->CNT < 20) {
			;
		}
		// 10microseconds / 50microseconds = CNT = 20
		GPIOC->PCOR |= (1 << TRIGGER_PIN); // set TRIGGER PIN to LOW

		if (xSemaphoreTake(sema, portMAX_DELAY) == pdTRUE) {
			float total_time = ((float) (end_count - start_count)) * 0.0000005;
			distance = (34300.0 * total_time) / 2.0;
			PRINTF("Distance = %d \r\n\n", (int) distance);
		}
		vTaskDelay(pdMS_TO_TICKS(30));
	}
}



#define MAX_MSG_LEN		256
#define QLEN	5
QueueHandle_t queue;
typedef struct tm {
	char message[MAX_MSG_LEN];
} TMessage;

char send_buffer[MAX_MSG_LEN];

void UART2_FLEXIO_IRQHandler(void)
{
	// Send and receive pointers
	static int recv_ptr=0, send_ptr=0;
	char rx_data;
	char recv_buffer[MAX_MSG_LEN];

    NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
	if(UART2->S1 & UART_S1_TDRE_MASK) // Send data
	{
		if(send_buffer[send_ptr] == '\0') {
			send_ptr = 0;

			// Disable the transmit interrupt
			UART2->C2 &= ~UART_C2_TIE_MASK;

			// Disable the transmitter
			UART2->C2 &= ~UART_C2_TE_MASK;

			BaseType_t hpw = pdFALSE;//
			xSemaphoreGiveFromISR(tx, &hpw);//
			portYIELD_FROM_ISR(hpw);//

		}
		else {
			UART2->D = send_buffer[send_ptr++];
		}
	}

	if(UART2->S1 & UART_S1_RDRF_MASK)
	{

		TMessage msg;
		rx_data = UART2->D;
		recv_buffer[recv_ptr++] = rx_data;
		if(rx_data == '\n') {
			// Copy over the string
			BaseType_t hpw;
			recv_buffer[recv_ptr]='\0';
			strncpy(msg.message, recv_buffer, MAX_MSG_LEN);
			xQueueSendFromISR(queue, (void *)&msg, &hpw);
			portYIELD_FROM_ISR(hpw);
			recv_ptr = 0;
		}
	}

}

void sendMessage(char *message) {

	if (xSemaphoreTake(tx, portMAX_DELAY) == pdTRUE) {//
		strncpy(send_buffer, message, MAX_MSG_LEN);

		// Enable the TIE interrupt
		UART2->C2 |= UART_C2_TIE_MASK;

		// Enable the transmitter
		UART2->C2 |= UART_C2_TE_MASK;
	}//
}

static void recvTask(void *p) { // ESP32 sends "Hello this is ESP32 which is printed out in serial monitor"
	while(1) {
		TMessage msg;
		if(xQueueReceive(queue, (TMessage *) &msg, portMAX_DELAY) == pdTRUE) {
			//PRINTF("Received message: %s\r\n", msg.message);
			if (strncmp(msg.message, "Act", 3) == 0) {
				PRINTF("ACTIVATE ROOF\r\n");
				TPM2->CONTROLS[1].CnV = 2000;
			} else if (strncmp(msg.message, "Dea", 3) == 0) {
				PRINTF("DEACTIVATE ROOF\r\n");
				TPM2->CONTROLS[1].CnV = 4000;
			}
		}
	}
}

static void sendTask(void *p) {
	//int count=0;
	char buffer[MAX_MSG_LEN];
	while(1) {
		sprintf(buffer, "%d\n", count);
		sendMessage(buffer);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

int buttonPressed = 0;
static void buzzer_task(void *p){
	while (1) {

		if (buttonPressed) {
			GPIOB->PCOR |= (1 << BUZZER_PIN);
			vTaskDelay(pdMS_TO_TICKS(20));
		} else {

			if (distance >= 0 && distance < 4){
				GPIOB->PSOR |= (1 << BUZZER_PIN);
				vTaskDelay(pdMS_TO_TICKS(5));
				GPIOB->PCOR = (1 << BUZZER_PIN);
				vTaskDelay(pdMS_TO_TICKS(5));
			}
			else {
				GPIOB->PCOR |= (1 << BUZZER_PIN);
				vTaskDelay(pdMS_TO_TICKS(20));
			}
		}
	}
}


static void buttonTask(void *p) {
	while (1) {

		if (!buttonPressed && !(GPIOB->PDIR & (1 << BTN_PIN))){ // button not pressed before and BTN_PIN became LOW (PRESSED)
			buttonPressed = 1;

		} else if (buttonPressed && distance >= 5) { // car leaving
			buttonPressed = 0;

		}
		vTaskDelay(pdMS_TO_TICKS(15));
	}
}




int main(void) {

	sema = xSemaphoreCreateBinary(); // starts with 0 until its given (in my case is by ISR)
	queue = xQueueCreate(QLEN, sizeof(TMessage));
	tx = xSemaphoreCreateBinary();//
	xSemaphoreGive(tx);//

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

    // ALL MY INIT() go here
    initUltrasonicPins();
    initBuzzerPin();
    initHallPin();
    initIRPin();
    initGatePin();
    initRoofPin();
    initTimer();
    initTimer0();
    initTimer2();
    initButtonPin();
    enableInterrupts();
    startTimer();
    startPWM();
    startRoofPWM();

    initUART2(BAUD_RATE);




#endif


    xTaskCreate(ultrasonic_task, "ultrasonic", configMINIMAL_STACK_SIZE+100, NULL, 1, &ultrasonic_task_handle);

    xTaskCreate(buzzer_task, "buzzer", configMINIMAL_STACK_SIZE+100, NULL, 1, &buzzer_task_handle);
    xTaskCreate(recvTask, "recvTask", configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
    xTaskCreate(sendTask, "sendTask", configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);
    xTaskCreate(buttonTask, "buttonTask", configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);
    vTaskStartScheduler();



    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

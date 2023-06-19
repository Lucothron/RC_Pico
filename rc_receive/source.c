// ------------ INCLUDES ------------

#include <stdio.h>
#include "pico/stdlib.h"
#include "string.h"
#include "hardware/spi.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "nrf24l01.h"

// ------------ DEFINITIONS ------------

#define SEND_FLAG 0 // change to 1 to run the program in send mode
#define SEND_FLAG_INT 0 // change to 1 to run the program in send int mode
#define RECV_FLAG 0 // change to 1 to run the program in recv mode
#define RECV_FLAG_INT 1 // change to 1 to run the program in recv int mode
#define LED 25

#define BUZZER_PIN 20
#define STEER_A 21
#define STEER_B 22
#define L_PIN 17
#define R_PIN 18
#define BRAKE_PIN 19
#define FRONT_PIN 16
#define H_EN 7

char msg_sent[10] = {" "}; // test data to send
char msg_recv[10]; // test data received

/*
s = speed
t = steer
l = left turn
r = right turn
e = blinker
f = front lights
m = mid gear
d = drive gear
h = buzzer
*/

static QueueHandle_t xSpeedQueue = NULL;
static QueueHandle_t xSteerQueue = NULL;
static QueueHandle_t xBlinkerQueue = NULL;
static QueueHandle_t xLightsQueue = NULL;
static QueueHandle_t xBuzzerQueue = NULL;

TaskHandle_t xLeftTurn = NULL;
TaskHandle_t xRightTurn = NULL;
TaskHandle_t xBlinkers = NULL;
TaskHandle_t xLights = NULL;
TaskHandle_t xBrake = NULL;
TaskHandle_t xBuzzer = NULL;

bool bBlinker = false;
bool bLight = false;
bool bManual = false;
bool bDrive = false;
bool bBuzzer = false;

// ------------ FREERTOS TASKS ------------

void vRFReceive(void *pvParameters) {
	uint uIValueToSend[9];

	nrf24l01_set_mode_rx();

	while(1)
	{
		if(nrf24l01_new_msg() )
		{
			gpio_put(LED, 1);
			nrf24l01_recv_msg((uint8_t *)&msg_recv, 10);
			printf("%s\n\r", msg_recv);
			char *token = strtok(msg_recv, " ");
			switch (*token) {
			case 's':
				token = strtok(NULL, " ");
				int temp_s;
				sscanf(token, "%d", &temp_s);
				//printf("speed: %s\n", token);
				xQueueSend(xSpeedQueue, &temp_s, 0U);
				break;
			case 't':
				token = strtok(NULL, " ");
				int temp_t;
				sscanf(token, "%d", &temp_t);
			 	//printf("steer: %d\n", temp);
				xQueueSend(xSteerQueue, &temp_t, 0U);
				break;
			case 'l':
				printf("left\n");
				vTaskResume(xLeftTurn);
				break;
			case 'r':
				printf("right\n");
				vTaskResume(xRightTurn);
				break;
			case 'e':
				printf("emergency\n");
				if (bBlinker == false) {
					bBlinker = true;
					vTaskResume(xBlinkers);
				} else {
					bBlinker = false;
					gpio_put(R_PIN, 0);
					gpio_put(L_PIN, 0);
					vTaskSuspend(xBlinkers);
				}
				break;
			case 'f':
				printf("lights\n");
				if (bLight == false) {
					bLight = true;
					vTaskResume(xLights);
				} else if(bLight == true) {
					bLight = false;
					gpio_put(R_PIN, 0);
					gpio_put(L_PIN, 0);
					vTaskSuspend(xLights);
				}
				break;
			case 'm':
				printf("medium\n");
				break;
			case 'd':
				printf("drive\n");
				break;
			case 'h':
				printf("horn\n");
				vTaskResume(xBuzzer);
				break;
			
			default:
				break;
			}

			sleep_ms(50);
			gpio_put(LED, 0);
		}
	}
}

void vSteeringLogic(void *pvParameters) {
	int uIReceivedValue;

	while(true) {
		xQueueReceive(xSteerQueue, &uIReceivedValue, portMAX_DELAY);

		// UART to control steering
		if ((uIReceivedValue > -25) || (uIReceivedValue < 25)) {
		gpio_put(STEER_A, 0);
		gpio_put(STEER_B, 0);
		} else if (uIReceivedValue > 25) {
		gpio_put(STEER_A, 0);
		gpio_put(STEER_B, 1);
		} else if (uIReceivedValue < -25) {
		gpio_put(STEER_A, 1);
		gpio_put(STEER_B, 0);
		}

		vTaskDelay(100);
	}
}

void vLeftTurn( void *pvParameters ) {
  	int uIReceivedValue;

	vTaskSuspend(xLeftTurn);

	while (true) {
		xQueueReceive(xSteerQueue, &uIReceivedValue, portMAX_DELAY);
    
		printf("left turn: %d\n", uIReceivedValue);
		if(uIReceivedValue > -50) {
			gpio_put(L_PIN, 1);
			vTaskDelay(500);
			gpio_put(L_PIN, 0);
			vTaskDelay(500);
		} else {
			vTaskSuspend(xLeftTurn);
		}
	}
}

void vRightTurn( void *pvParameters ) {
	int uIReceivedValue;

	vTaskSuspend(xRightTurn);

	while (true) {
		xQueueReceive(xSteerQueue, &uIReceivedValue, portMAX_DELAY);

		printf("right turn\n");
		if(uIReceivedValue < 50) {
			gpio_put(R_PIN, 1);
			vTaskDelay(500);
			gpio_put(R_PIN, 0);
			vTaskDelay(500);
		} else {
			vTaskSuspend(xRightTurn);
		}
	}
}

void vBlinkers( void *pvParameters ) {
	char uIReceivedValue;

	vTaskSuspend(xBlinkers);

	while (true) {
    	
		gpio_put(R_PIN, 1);
		gpio_put(L_PIN, 1);
		vTaskDelay(500);
		gpio_put(R_PIN, 0);
		gpio_put(L_PIN, 0);
		vTaskDelay(500);

  	}
}

void vFrontLights( void *pvParameters ) {
	uint uIReceivedValue;
	bool bLights = false;

	vTaskSuspend(xLights);

	while (true) {

		gpio_put(R_PIN, 1);
		gpio_put(L_PIN, 1);

	}
}

void vBuzzer( void *pvParameters ) {
	uint uIReceivedValue;

	vTaskSuspend(xBuzzer);

	while (true) {

		gpio_put(BUZZER_PIN, 1);
		vTaskDelay(100);

		gpio_put(BUZZER_PIN, 0);

		vTaskSuspend(xBuzzer);

    }
}

void vBrake( void *pvParameters ) {
	uint uIReceivedValue;

	while (true) {

		xQueueReceive(xSpeedQueue, &uIReceivedValue, portMAX_DELAY);
		
		if (uIReceivedValue > 0) {
			gpio_put(BRAKE_PIN, 0);
		} else {
			gpio_put(BRAKE_PIN, 1);
		}
	}
}

void vSerialCommunication(void *pvParameters) {
	int uIReceivedValue;

	while(true) {
		xQueueReceive(xSpeedQueue, &uIReceivedValue, portMAX_DELAY);

		if(uart_is_writable(uart0)){
			uart_putc(uart0, uIReceivedValue);
    	}

		vTaskDelay(100);
	}
}

void main() {
	// Initialize I/O
	stdio_init_all();
	nrf24l01_init();

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
	gpio_init(STEER_A);
    gpio_set_dir(STEER_A, GPIO_OUT);
	gpio_init(STEER_B);
    gpio_set_dir(STEER_B, GPIO_OUT);
	gpio_init(L_PIN);
    gpio_set_dir(L_PIN, GPIO_OUT);
	gpio_init(R_PIN);
    gpio_set_dir(R_PIN, GPIO_OUT);
	gpio_init(BRAKE_PIN);
    gpio_set_dir(BRAKE_PIN, GPIO_OUT);
	gpio_init(FRONT_PIN);
    gpio_set_dir(FRONT_PIN, GPIO_OUT);
	gpio_init(H_EN);
    gpio_set_dir(H_EN, GPIO_OUT);

	gpio_put(H_EN, 1);

	xSpeedQueue = xQueueCreate(1, sizeof(uint));
	xSteerQueue = xQueueCreate(1, sizeof(uint));
 	xBlinkerQueue = xQueueCreate(1, sizeof(uint));
	xLightsQueue = xQueueCreate(1, sizeof(uint));
	xBuzzerQueue = xQueueCreate(1, sizeof(uint));

	// // Create Tasks
	xTaskCreate(vRFReceive, "RF receive", 256, NULL, 1, NULL);
	xTaskCreate(vSteeringLogic, "Steering", 256, NULL, 1, NULL);
	xTaskCreate(vSerialCommunication, "Serial send", 256, NULL, 1, NULL);
	xTaskCreate(vLeftTurn, "Turn Left", 256, NULL, 1, &xLeftTurn);
	xTaskCreate(vRightTurn, "Turn Right", 256, NULL, 1, &xRightTurn);
	xTaskCreate(vBlinkers, "Blinkers", 256, NULL, 1, &xBlinkers);
	xTaskCreate(vFrontLights, "Lights", 256, NULL, 1, &xLights);
	xTaskCreate(vBuzzer, "Buzzer", 256, NULL, 1, &xBuzzer);
	xTaskCreate(vBrake, "Brake", 256, NULL, 1, &xBrake);

	// Start scheduler
	vTaskStartScheduler();
}

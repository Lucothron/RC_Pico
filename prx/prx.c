#include <stdio.h>
#include <string.h>
#include "pico/util/queue.h"
#include "NRF24.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/util/datetime.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

payload_prx_t payload_rx;

typedef struct { void *func; } queue_entry_t;

static queue_t call_queue;

static QueueHandle_t xSpeedQueue = NULL;
static QueueHandle_t xSteerQueue = NULL;
static QueueHandle_t xBlinkerQueue = NULL;
static QueueHandle_t xLightsQueue = NULL;
static QueueHandle_t xBuzzerQueue = NULL;

TaskHandle_t xLeftTurn = NULL;
TaskHandle_t xRightTurn = NULL;
TaskHandle_t xBlinkers = NULL;
TaskHandle_t xLights = NULL;
TaskHandle_t xBuzzer = NULL;

void gpio_irq_handler() {

  queue_entry_t entry = {&check_irq_bit};

  queue_add_blocking(&call_queue, &entry);
}

void vReceiveRF ( void *pvParameters ) {
  uint uIValueToSend[7];
  uint bBlinkers = 0;
  uint bLights = 0;
  uint bBuzzer = 0;
  
  while(true) {
    if (!queue_is_empty(&call_queue))
    {
      queue_entry_t entry;

      queue_remove_blocking(&call_queue, &entry);

      uint8_t (*func)() = (uint8_t(*)())(entry.func);

      uint8_t irq_bit = (*func)();

      if (irq_bit == RX_DR_ASSERTED) {
        do
          {
            // Receive RF payload
            rx_message(&payload_rx);
            printf("Rx Message - %d: %d: %d: %d: %d: %d: %d:\n", payload_rx.moisture, payload_rx.moisture_1,payload_rx.moisture_2,payload_rx.moisture_3,payload_rx.moisture_4,payload_rx.moisture_5,payload_rx.moisture_6);
          } while (!check_fifo_status(RX_EMPTY));
      }

/*
      Data packet contains values from controller used to activate parts of the vehicle
      Packet breakdown:
        [0] = x
        [1] = y
        [2] = l_button
        [3] = r_button
        [4] = blinker
        [5] = lights
        [6] = buzzer
      */

      uIValueToSend[0] = payload_rx.moisture;
      uIValueToSend[1] = payload_rx.moisture_1;
      uIValueToSend[2] = payload_rx.moisture_2;
      uIValueToSend[3] = payload_rx.moisture_3;
      uIValueToSend[4] = payload_rx.moisture_4;
      uIValueToSend[5] = payload_rx.moisture_5;
      uIValueToSend[6] = payload_rx.moisture_6;

      // Send to tasks
      xQueueSend(xSpeedQueue, &uIValueToSend[0], 0U);
      xQueueSend(xSteerQueue, &uIValueToSend[1], 0U);

      // Left turn
      if (payload_rx.moisture_2 == 1) {
        vTaskResume(xLeftTurn);
      }
      // Rigt turn
      if (payload_rx.moisture_3 == 1) {
        vTaskResume(xRightTurn);
      }
      // Blinkers
      if (payload_rx.moisture_4 == 1) {
        if (bBlinkers == 0) {
          bBlinkers = 1;
          vTaskResume(xBlinkers);
          xQueueSend(xBlinkerQueue, &bBlinkers, 0U);
        } else {
          bBlinkers = 0;
          vTaskResume(xBlinkers);
          xQueueSend(xBlinkerQueue, &bBlinkers, 0U);
        }
      }
      // Lights
      if (payload_rx.moisture_5 == 1) {
        if (bLights == 0) {
          bLights = 1;
          vTaskResume(xLights);
          xQueueSend(xLightsQueue, &bLights, 0U);
        } else {
          bLights = 0;
          vTaskResume(xLights);
          xQueueSend(xLightsQueue, &bLights, 0U);
        }
      }
      // Horn
      if (payload_rx.moisture_6 == 1) {
        if (bBuzzer == 0) {
          bBuzzer = 1;
          vTaskResume(xBuzzer);
          xQueueSend(xBuzzerQueue, &bBuzzer, 0U);
        } else {
          bBuzzer = 0;
          vTaskResume(xBuzzer);
          xQueueSend(xBuzzerQueue, &bBuzzer, 0U);
        }
      }

    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
  }    
}

void vSpeedControl( void *pvParameters ) {
  uint uIReceivedValue;

  while (true) {
    xQueueReceive(xSpeedQueue, &uIReceivedValue, portMAX_DELAY);

    // Send to pico via serial
  }
}

void vSteeringControl( void *pvParameters ) {
  uint uIReceivedValue;

  while (true) {
    xQueueReceive(xSteerQueue, &uIReceivedValue, portMAX_DELAY);

    // PWM to control steering
  }
}

void vLeftTurn( void *pvParameters ) {
  
  vTaskSuspend(xLeftTurn);

  while (true) {
    
    // LED blink sequence

    vTaskSuspend(xLeftTurn);

  }
}

void vRightTurn( void *pvParameters ) {
  
  vTaskSuspend(xRightTurn);

  while (true) {

    // LED blink sequence

    vTaskSuspend(xRightTurn);

  }
}

void vBlinkers( void *pvParameters ) {
  uint uIReceivedValue;

  vTaskSuspend(xBlinkers);

  while (true) {

    xQueueReceive(xSteerQueue, &uIReceivedValue, portMAX_DELAY);
    if(uIReceivedValue == 1) {

      // LED blink sequence

    } else {
      vTaskSuspend(xBlinkers);
    }
  }
}

void vFrontLights( void *pvParameters ) {
  uint uIReceivedValue;

  vTaskSuspend(xLights);

  while (true) {

    xQueueReceive(xLightsQueue, &uIReceivedValue, portMAX_DELAY);
    if(uIReceivedValue == 1) {

      // Turn on lights

    } else {
      vTaskSuspend(xLights);
    }
  }
}

void vBuzzer( void *pvParameters ) {
  uint uIReceivedValue;

  vTaskSuspend(xBuzzer);

  while (true) {

    xQueueReceive(xBuzzerQueue, &uIReceivedValue, portMAX_DELAY);
    if(uIReceivedValue == 1) {

      // Turn on buzzer

    } else {
      vTaskSuspend(xBuzzer);
    }
  }
}

int main(void) {
  stdio_init_all();
  init_spi(); // Initialise SPI and GPIO pins
  init_nrf24(); // Initial config when device first powered
  init_nrf24_prx_registers(); // Config PRX specific registers
  set_mode(RX_MODE); // Activate RX_MODE
  sleep_ms(10000); // Sleep for 10s to facilitate opening PuTTy to read printf output
  debug_registers(); // printf register values
  debug_rx_address_pipes(RX_ADDR_P0); // printf RX_ADDR_P0 register
  debug_rx_address_pipes(RX_ADDR_P1); // printf RX_ADDR_P1 register
  debug_rx_address_pipes(RX_ADDR_P2); // printf RX_ADDR_P2 register
  debug_rx_address_pipes(RX_ADDR_P3); // printf RX_ADDR_P3 register
  debug_rx_address_pipes(RX_ADDR_P4); // printf RX_ADDR_P4 register
  debug_rx_address_pipes(RX_ADDR_P5); // printf RX_ADDR_P5 register
  queue_init(&call_queue, sizeof(queue_entry_t), 6);
  // IRQ interrupt handler
  gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  
  xSpeedQueue = xQueueCreate(1, sizeof(uint));
	xSteerQueue = xQueueCreate(1, sizeof(uint));
  xBlinkerQueue = xQueueCreate(1, sizeof(uint));
	xLightsQueue = xQueueCreate(1, sizeof(uint));
	xBuzzerQueue = xQueueCreate(1, sizeof(uint));

  // Create tasks
  xTaskCreate(vReceiveRF, "RX", 256, NULL, 1, NULL);
  xTaskCreate(vSpeedControl, "Speed", 256, NULL, 1, NULL);
  xTaskCreate(vSteeringControl, "Steering", 256, NULL, 1, NULL);
  xTaskCreate(vLeftTurn, "Turn Left", 256, NULL, 1, &xLeftTurn);
  xTaskCreate(vRightTurn, "Turn Right", 256, NULL, 1, &xRightTurn);
  xTaskCreate(vBlinkers, "Blinkers", 256, NULL, 1, &xBlinkers);
  xTaskCreate(vFrontLights, "Lights", 256, NULL, 1, &xLights);
  xTaskCreate(vBuzzer, "Buzzer", 256, NULL, 1, &xBuzzer);
  
  vTaskStartScheduler();

  return 0;
}
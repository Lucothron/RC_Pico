#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include <string.h>
#include "pico/util/queue.h"
#include "NRF24.h"
#include "SEN0308.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/util/datetime.h"
#include "FreeRTOS.h"
#include "task.h"

#define THROTTLE_PIN 26
#define BRAKE_PIN 27
#define BUZZER_BUTTON_PIN 13
#define LIGHTS_BUTTON_PIN 17
#define LEFT_BUTTON_PIN 14
#define RIGHT_BUTTON_PIN 15
#define EMERGENCY_BUTTON_PIN 16
#define PRIMERA_BUTTON_PIN 8
#define SEGUNDA_BUTTON_PIN 9

int previous_mapped_x = 0;
int mapped_x;
int mapped_y;
int abrupt_change_threshold = 50;  // Valor umbral para detectar un cambio abrupto
int joystick_values[2];  // Array para almacenar los valores del joystick (x y)
bool button_states[4];   // Array para almacenar los estados de los botones (izquierdo, derecho, emergencia, luces)
payload_t payload_tx;
uint8_t msg = 1;

typedef struct { void *func; } queue_entry_t;
static queue_t call_queue;
// send mssage flag
static volatile bool send_msg = true;

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    // Función para mapear un valor de un rango a otro
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int remove_noise(int value, int threshold) {
    // Función para eliminar el ruido dentro de un rango específico
    if (value < -threshold || value > threshold) {
        return value;
    } else {
        return 0;
    }
}

void Joystick()
{
    // Leer los valores analógicos del joystick
    int x = adc_read();
    adc_select_input(1);
    int y = adc_read();
    adc_select_input(0);

    mapped_x = map(x, 0, 4095, -100, 100);
    mapped_y = map(y, 0, 4095, -100, 100);

    // Eliminar el ruido dentro del rango del -15% al 15%
    int noise_threshold = 15;
    mapped_x = remove_noise(mapped_x, noise_threshold);
    mapped_y = remove_noise(mapped_y, noise_threshold);
}

void Horn(void *pvParameters)
{
    for (;;)
    {
        // Leer el estado del botón
        bool button_state = !gpio_get(BUZZER_BUTTON_PIN);

        // Verificar si se ha presionado el botón
        if (button_state)
        {
            //printf("Botón de bocina presionado\n");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
    }
}

void Front_Lights(void *pvParameters)
{
    bool lights_button_prev_state = false;
    bool lights_button_current_state = false;

    for (;;)
    {
        // Leer el estado del botón
        bool new_lights_button_state = !gpio_get(LIGHTS_BUTTON_PIN);

        // Verificar si se ha producido un cambio en el estado del botón de luces
        if (new_lights_button_state != lights_button_prev_state)
        {
            lights_button_prev_state = new_lights_button_state;
            if (lights_button_prev_state)
            {
                lights_button_current_state = !lights_button_current_state;
                if (lights_button_current_state)
                {
                    //printf("Luces frontales encendidas\n");
                }
                else
                {
                    //printf("Luces frontales apagadas\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
    }
}

void Brake(void *pvParameters)
{
    for (;;)
    {
        Joystick();

        // Detectar un cambio abrupto en el eje X del joystick
        if (abs(mapped_x - previous_mapped_x) > abrupt_change_threshold) {
            //printf("Luces de freno apagadas\n");
        } else {
            // printf("Luces de freno encendidas\n");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
    }
}

void Turn_Lights(void *pvParameters)
{
    bool left_active = false;
    bool right_active = false;
    //bool lights_active = false;
    bool left_button_state = false;
    bool right_button_state = false;
    bool emergency_button_state = false;
    bool lights_button_current_state = false;
    //bool lights_button_state = false;
    bool lights_button_prev_state = false;

    for (;;)
    {
        Joystick();

        // Condición para activar las luces direccionales izquierdas cuando el volante se gire hacia el lado izquierdo
        if (mapped_y < 0 && !left_active) {
            //printf("Direccional izquierda encendida\n");
            // printf("Direccional derecha apagada\n");
        } else {
            // printf("Direccional izquierda apagada\n");
        }

        // Condición para activar las luces direccionales derechas cuando el volante se gire hacia el lado derecho
        if (mapped_y > 0 && !right_active) {
            //printf("Direccional derecha activa\n");
            // printf("Direccional izquierda apagada\n");
        } else {
            // printf("Direccional derecha apagada\n");
        }

        // Pequeña pausa para evitar lecturas demasiado frecuentes
        sleep_ms(50);

        // Actualizar el estado de los botones
        bool new_left_button_state = !gpio_get(LEFT_BUTTON_PIN);
        bool new_right_button_state = !gpio_get(RIGHT_BUTTON_PIN);
        bool new_emergency_button_state = !gpio_get(EMERGENCY_BUTTON_PIN);
        bool new_lights_button_state = !gpio_get(LIGHTS_BUTTON_PIN);
        bool new_button_state = !gpio_get(BUZZER_BUTTON_PIN);

        button_states[0] = new_left_button_state;
        button_states[1] = new_right_button_state;
        button_states[2] = new_emergency_button_state;
        button_states[3] = new_lights_button_state;
        button_states[4] = new_button_state;

        // Verificar si se ha producido un cambio en el estado de los botones
        if (new_left_button_state != left_button_state) {
            left_button_state = new_left_button_state;
            if (left_button_state) {
                left_active = !left_active;
                printf("Direccional derecha encendido\n");
            }
        }

        if (new_right_button_state != right_button_state) {
            right_button_state = new_right_button_state;
            if (right_button_state) {
                right_active = !right_active;
                printf("Direccional izquierda encendido\n");
            }
        }

        if (new_emergency_button_state != emergency_button_state) {
            emergency_button_state = new_emergency_button_state;
            if (emergency_button_state) {
                printf("Luces de emergencia encendidas\n");
                // printf("Direccional izquierda encendida\n");
                // printf("Direccional derecha encendida\n");
            }
        }

        // Verificar si se ha producido un cambio en el estado del botón de luces
        if (new_lights_button_state != lights_button_prev_state) {
            lights_button_prev_state = new_lights_button_state;
            if (lights_button_prev_state) {
                lights_button_current_state = !lights_button_current_state;
                if (lights_button_current_state) {
                    // printf("Luces Frontales encendidas\n");
                    // printf("Direccional izquierda desactivada\n");
                    // printf("Direccional derecha desactivada\n");
                } else {
                    // printf("Luces Frontales apagadas\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
    }
}

void gpio_irq_handler() {
    queue_entry_t entry = {&check_irq_bit};
    queue_add_blocking(&call_queue, &entry);
    }

void TX(void *pvParameters)
{
 for(;;){
    // Mostrar los valores mapeados en la consola
    joystick_values[0] = mapped_x;
    joystick_values[1] = mapped_y;
    // Actualizar el estado de los botones
    bool new_left_button_state = !gpio_get(LEFT_BUTTON_PIN);
    bool new_right_button_state = !gpio_get(RIGHT_BUTTON_PIN);
    bool new_emergency_button_state = !gpio_get(EMERGENCY_BUTTON_PIN);
    bool new_lights_button_state = !gpio_get(LIGHTS_BUTTON_PIN);
    bool buzzer_button_state = !gpio_get(BUZZER_BUTTON_PIN);
    // Actualizar los estados de los botones en el array
    button_states[0] = new_left_button_state;
    button_states[1] = new_right_button_state;
    button_states[2] = new_emergency_button_state;
    button_states[3] = new_lights_button_state;
    button_states[4] = buzzer_button_state;

    int combined_array[6];
    combined_array[0] = joystick_values[0];  // Valor del eje X del joystick
    combined_array[1] = joystick_values[1];  // Valor del eje Y del joystick
        for (int i = 0; i < 5; i++) {
            combined_array[i + 2] = button_states[i];
        }
    if (send_msg)
    {
      send_msg = false; // Reset send message flag
      // Read control values into payload
      payload_tx.moisture = combined_array[0];
      payload_tx.moisture_1 = combined_array[1];
      payload_tx.moisture_2 = combined_array[2];
      payload_tx.moisture_3 = combined_array[3];
      payload_tx.moisture_4 = combined_array[4];
      payload_tx.moisture_5 = combined_array[5];
      payload_tx.moisture_6 = combined_array[6];
      // Transmit the payload to the PRX
      tx_message(&payload_tx);
      printf("Tx message # %d: %d: %d: %d: %d: %d: %d: %d: %d:\n", payload_tx.moisture, payload_tx.moisture_1, payload_tx.moisture_2,payload_tx.moisture_3,payload_tx.moisture_4,payload_tx.moisture_5,payload_tx.moisture_6,payload_tx.moisture_7,payload_tx.moisture_8);
      // Tranmission count for debugging
      msg++;
    }

    if (!queue_is_empty(&call_queue))
    {
      queue_entry_t entry;

      queue_remove_blocking(&call_queue, &entry);

      uint8_t (*func)() = (uint8_t(*)())(entry.func);

      uint8_t irq_bit = (*func)();

      switch (irq_bit)
      {       
        case RX_DR_ASSERTED:
          // Not possible in this PTX example as never enters Rx mode
        break;
        
        case TX_DS_ASSERTED:
          send_msg = true;
          sleep_ms(500);
        break;

        case MAX_RT_ASSERTED:
        printf("ERROR");
        break;
      }
    }
   vTaskDelay(pdMS_TO_TICKS(100));
  }
}

int main() {
    stdio_init_all();
    adc_init();
    adc_set_temp_sensor_enabled(true);
    gpio_init(THROTTLE_PIN);
    gpio_set_dir(THROTTLE_PIN, GPIO_IN);
    gpio_pull_up(THROTTLE_PIN);
    gpio_init(BRAKE_PIN);
    gpio_set_dir(BRAKE_PIN, GPIO_IN);
    gpio_pull_up(BRAKE_PIN);
    gpio_init(BUZZER_BUTTON_PIN);
    gpio_set_dir(BUZZER_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUZZER_BUTTON_PIN);
    gpio_init(LIGHTS_BUTTON_PIN);
    gpio_set_dir(LIGHTS_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(LIGHTS_BUTTON_PIN);
    gpio_init(LEFT_BUTTON_PIN);
    gpio_set_dir(LEFT_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(LEFT_BUTTON_PIN);
    gpio_init(RIGHT_BUTTON_PIN);
    gpio_set_dir(RIGHT_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_BUTTON_PIN);
    gpio_init(EMERGENCY_BUTTON_PIN);
    gpio_set_dir(EMERGENCY_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(EMERGENCY_BUTTON_PIN);
    gpio_init(PRIMERA_BUTTON_PIN);
    gpio_pull_up(PRIMERA_BUTTON_PIN);
    gpio_init(SEGUNDA_BUTTON_PIN);
    gpio_pull_up(SEGUNDA_BUTTON_PIN);
  init_spi(); // Initialise SPI and GPIO pins
  init_adc(); // Initialise ADC and GPIO pins
  init_nrf24(); // Initial config when device first powered
  // Config PTX specific registers and Tx payloads to PRX data pipe 0
  init_nrf24_ptx_registers(PRX_ADDR_P5); 
  set_mode(TX_MODE); // Activate TX_MODE
  sleep_ms(10000); // Sleep for 10s to facilitate opening PuTTy to read printf output
  debug_registers(); // printf register values
  debug_rx_address_pipes(RX_ADDR_P0); // printf RX_ADDR_P0 register
  debug_rx_address_pipes(TX_ADDR); // printf TX_ADDR register
  // Initialise the call_queue utilized by gpio_irq_handler
  queue_init(&call_queue, sizeof(queue_entry_t), 6);
  // Enable IRQ for PIN_IRQ GPIO and set interrupt handler (irq_handler)
  gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    xTaskCreate(Horn, "Horn", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(Front_Lights, "Front Lights", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(Brake, "Brake", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(Turn_Lights, "Turn Lights", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(TX, "TX", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    vTaskStartScheduler();

    sleep_ms(1000);
    return 0;
}

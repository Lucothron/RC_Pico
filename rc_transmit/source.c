// ------------ INCLUDES ------------

#include <stdio.h>
#include "pico/stdlib.h"
#include "string.h"
#include "hardware/spi.h"
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "nrf24l01.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// ------------ DEFINITIONS ------------

#define SEND_FLAG 0 // change to 1 to run the program in send mode
#define SEND_FLAG_INT 0 // change to 1 to run the program in send int mode
#define RECV_FLAG 0 // change to 1 to run the program in recv mode
#define RECV_FLAG_INT 1 // change to 1 to run the program in recv int mode
#define LED 25
#define THROTTLE_PIN 26
#define BRAKE_PIN 27
#define BUZZER_BUTTON_PIN 13
#define LIGHTS_BUTTON_PIN 17
#define LEFT_BUTTON_PIN 14
#define RIGHT_BUTTON_PIN 15
#define EMERGENCY_BUTTON_PIN 16
#define FORWARD1 18
#define FORWARD2 19

int previous_mapped_y = 0;
int mapped_x;
int mapped_y;
int abrupt_change_threshold = 50;  // Valor umbral para detectar un cambio abrupto

char msg_sent[10]; // test data to send
char msg_recv[10]; // test data received

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


// Mapped functions
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

// ------------ FREERTOS TASKS ------------

void vRFReceive(void *pvParameters) {
	uint uIValueToSend[7];
	uint light_value;

	bool b_blinkers = false;
	bool b_lights = false;
}

void vRFTransmit(void *pvParameters) {
	uint uIValueToSend[7];
	nrf24l01_set_mode_tx();
	while(1)
	{ 
		gpio_put(LED, 1);
		nrf24l01_send_msg((uint8_t*)msg_sent, 10);
		vTaskDelay(100);  // Esperar un tiempo antes de verificar el botón nuevamente
		gpio_put(LED, 0);
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

	sprintf(msg_sent, "t %d", mapped_x);
	nrf24l01_send_msg((uint8_t*)msg_sent, 10);

	sprintf(msg_sent, "s %d", mapped_y);
	nrf24l01_send_msg((uint8_t*)msg_sent, 10);

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
            printf("Botón de bocina presionado\n");
			sprintf(msg_sent, "h");
			nrf24l01_send_msg((uint8_t*)msg_sent, 10);
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
                    printf("Luces frontales encendidas\n");
					sprintf(msg_sent, "f");
					nrf24l01_send_msg((uint8_t*)msg_sent, 10);
                }
                else
                {
                    printf("Luces frontales apagadas\n");
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
        if (abs(mapped_y - previous_mapped_y) < abrupt_change_threshold) {
            //printf("Luces de freno encendidas\n");
        } else {
            // printf("Luces de freno encendidas\n");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
    }
}

void Forward_1(void *pvParameters)
{
    bool forward1_prev_state = false;
    bool forward1_current_state = false;

    for (;;)
    {
        // Leer el estado del botón
        bool new_forward1_button_state = !gpio_get(FORWARD1);

        // Verificar si se ha producido un cambio en el estado del botón de luces
        if (new_forward1_button_state != forward1_prev_state)
        {
            forward1_current_state = new_forward1_button_state;
            if (forward1_current_state)
            {
                forward1_current_state = !forward1_current_state;
                if (forward1_current_state)
                {
                    // printf("Forward1 apagado\n");
                    // printf("Forward2 encendido\n");
                }
                else
                {
                    printf("Forward1 encendido\n");
					sprintf(msg_sent, "m");
					nrf24l01_send_msg((uint8_t*)msg_sent, 10);
                    // printf("Forward2 apagado\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
    }
}

void Forward_2(void *pvParameters)
{
    bool forward2_prev_state = false;
    bool forward2_current_state = false;

    for (;;)
    {
        // Leer el estado del botón
        bool new_forward2_button_state = !gpio_get(FORWARD2);

        // Verificar si se ha producido un cambio en el estado del botón de luces
        if (new_forward2_button_state != forward2_prev_state)
        {
            forward2_current_state = new_forward2_button_state;
            if (forward2_current_state)
            {
                forward2_current_state = !forward2_current_state;
                if (forward2_current_state)
                {
                    // printf("Forward2 apagado\n");
                    // printf("Forward1 encendido\n");
                }
                else
                {
                    printf("Forward2 encendido\n");
					sprintf(msg_sent, "d");
					nrf24l01_send_msg((uint8_t*)msg_sent, 10);
                    // printf("Forward1 apagado\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un tiempo antes de verificar el botón nuevamente
    }
}

void Turn_Lights(void *pvParameters)
{
    bool left_active = false;
    bool right_active = false;
    bool lights_active = false;
    bool left_button_state = false;
    bool right_button_state = false;
    bool emergency_button_state = false;
    bool lights_button_current_state = false;
    bool lights_button_state = false;
    bool lights_button_prev_state = false;

    for (;;)
    {
        Joystick();

        // Condición para activar las luces direccionales izquierdas cuando el volante se gire hacia el lado izquierdo
        if (mapped_x < 0 && !left_active) {
            printf("Direccional izquierda encendida\n");
			sprintf(msg_sent, "l");
			nrf24l01_send_msg((uint8_t*)msg_sent, 10);
            // printf("Direccional derecha apagada\n");
        } else {
            // printf("Direccional izquierda apagada\n");
        }

        // Condición para activar las luces direccionales derechas cuando el volante se gire hacia el lado derecho
        if (mapped_x > 0 && !right_active) {
            printf("Direccional derecha activa\n");
			sprintf(msg_sent, "r");
			nrf24l01_send_msg((uint8_t*)msg_sent, 10);
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

        // Verificar si se ha producido un cambio en el estado de los botones
        if (new_left_button_state != left_button_state) {
            left_button_state = new_left_button_state;
            if (left_button_state) {
                left_active = !left_active;
                printf("Direccional derecha encendido\n");
				sprintf(msg_sent, "r");
				nrf24l01_send_msg((uint8_t*)msg_sent, 10);
            }
        }

        if (new_right_button_state != right_button_state) {
            right_button_state = new_right_button_state;
            if (right_button_state) {
                right_active = !right_active;
                printf("Direccional izquierda encendido\n");
				sprintf(msg_sent, "l");
				nrf24l01_send_msg((uint8_t*)msg_sent, 10);
            }
        }

        if (new_emergency_button_state != emergency_button_state) {
            emergency_button_state = new_emergency_button_state;
            if (emergency_button_state) {
                printf("Luces de emergencia encendidas\n");
				sprintf(msg_sent, "e");
				nrf24l01_send_msg((uint8_t*)msg_sent, 10);
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
    gpio_init(FORWARD1);
    gpio_set_dir(FORWARD1, GPIO_IN);
    gpio_pull_up(FORWARD1);
    gpio_init(FORWARD2);
    gpio_set_dir(FORWARD2, GPIO_IN);
    gpio_pull_up(FORWARD2);
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	stdio_init_all();
	nrf24l01_init();

     xTaskCreate(Horn, "Horn", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
     xTaskCreate(Front_Lights, "Front Lights", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
     xTaskCreate(Brake, "Brake", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
     xTaskCreate(Turn_Lights, "Turn Lights", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
     xTaskCreate(Forward_1, "Fordward_1", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
     xTaskCreate(Forward_2, "Fordward_2", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(vRFTransmit, "vRFTransmit", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    vTaskStartScheduler();
    //sleep_ms(1000);
    return 0;
}

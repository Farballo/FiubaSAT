#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "circular_buffer.h"
#include "NMEA.h"

#include "blink.h"
#include "timers.h"
#include <stdio.h>
#include "semphr.h"
#include "test.h"
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// Handle para la tarea del parpadeo
static TaskHandle_t blink_handle;
static int Copy_from_to(const char *source, const char *pattern_start, const char *pattern_finish, char *dest);

#define SIZE_BUFFER 512

/* Tarea asignada a recolección de datos de GPS */
static void taskUART1_GPS(uint32_t usart_id) {
    vTaskDelay(pdMS_TO_TICKS(4000));
    char buffer[SIZE_BUFFER];
    uint16_t datos;

    char GGA[100];
    char RMC[100];

    GPSSTRUCT gpsData;

    int flagGGA = 0, flagRMC = 0;

    for (;;) {
        datos = UART_available_data(usart_id);
        if (datos > 400) {
            UART_clear_rx_queue(usart_id, pdMS_TO_TICKS(100));
        } else if (datos > 250) {
            memset(buffer, 0, SIZE_BUFFER);
            get_rxq_buffer(usart_id, buffer, SIZE_BUFFER);

            Copy_from_to(buffer, "$GPGGA,", "*", GGA);
            if (decodeGGA(GGA, &gpsData.ggastruct) == 0) flagGGA = 2;  // 2 indicates the data is valid
            else flagGGA = 1;  // 1 indicates the data is invalid

            Copy_from_to(buffer, "$GPRMC,", "*", RMC);
            if (decodeRMC(RMC, &gpsData.rmcstruct) == 0) flagRMC = 2;  // 2 indicates the data is valid
            else flagRMC = 1;  // 1 indicates the data is invalid

            if (flagGGA == 2 && flagRMC == 2)
                UART_puts(USART3, "GGA and RMC data is valid\r\n", pdMS_TO_TICKS(100));
            
            UART_puts(USART3, buffer, pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            
            UART_puts(USART3, "GGA: ", pdMS_TO_TICKS(100));
            UART_puts(USART3, GGA, pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            
            UART_puts(USART3, "RMC: ", pdMS_TO_TICKS(100));
            UART_puts(USART3, RMC, pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));

            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            UART_clear_rx_queue(usart_id, pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

static int Copy_from_to(const char *source, const char *pattern_start, const char *pattern_finish, char *dest) {
    // Encontrar la posición del patrón de inicio en la cadena fuente
    const char *start = strstr(source, pattern_start);
    if (start == NULL) {
        return -1; // Patrón de inicio no encontrado
    }

    // Avanzar el puntero al final del patrón de inicio
    start += strlen(pattern_start);

    // Encontrar la posición del patrón de final después del patrón de inicio
    const char *finish = strstr(start, pattern_finish);
    if (finish == NULL) {
        return -2; // Patrón de final no encontrado
    }

    // Calcular la longitud de la subcadena a copiar
    size_t len = finish - start;

    // Copiar la subcadena al buffer de destino
    strncpy(dest, start, len);
    dest[len] = '\0'; // Null-terminar el buffer de destino

    return 0; // Éxito
}

/* Acá estaría la tarea asignada al periférico conectado a la interfaz USART3 */
static void taskUART3_receive(uint32_t usart_id) {
    uint16_t data;
    for (;;) {
        // Esperar a que el semáforo indique que hay datos disponibles
        if (UART_semaphore_take(usart_id, portMAX_DELAY) == pdTRUE) {
            // Procesar todos los datos en la cola
            while (UART_receive(usart_id, &data)) {
                // Aquí puedes manejar el dato recibido (por ejemplo, almacenarlo o procesarlo)
                UART_putchar(USART3, data, pdMS_TO_TICKS(100));
            }
            // Liberar el semáforo después de procesar los datos
            UART_semaphore_release(usart_id);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* 
static void taskUART3_TuteTransmit(uint32_t usart_id) {
    // Se inicializa la USART

    serial_begin(USART2, BAUD115K2);
    serial_begin(USART3, BAUD115K2);

    // PC13 (LED) as output:
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set(GPIOC, GPIO13);
    gpio_clear(GPIOC, GPIO13); // start buffering.

    const char saludo[] = "Hola mundo!\r\n";
    int saludo_len = sizeof(saludo) - 1;
    for(;;) {
        if(serial_puts(usart_id, saludo) != saludo_len)
            gpio_toggle(GPIOC, GPIO13); 
        usart_enable_tx_interrupt(usart_id);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/

/* Handler en caso de que la aplicación cause un overflow del stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

/* Main loop donde arranca el programa */
int main(void) {
    // Setup main clock, using external 8MHz crystal 
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Inicialización del LED para el blink
    blink_setup();
    
    // Inicialización de UARTs con sus baudrates
    if(UART_setup(USART1, 9600) != pdPASS) return -1;
    if(UART_setup(USART2, 115200) != pdPASS) return -1;
    if(UART_setup(USART3, 115200) != pdPASS) return -1;

    

    // Crear tarea para parpadear el LED
    xTaskCreate(taskBlink, "LED", 100, NULL, 2, &blink_handle);  // Crear tarea para parpadear el LED

    // Creación de tareas genéricas para transmisión UART
    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART1 TX", 128, (void *)USART1, 2, NULL);
    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART2 TX", 128, (void *)USART2, 2, NULL);
    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART3 TX", 128, (void *)USART3, 2, NULL);

    // Creación de tareas genéricas para recepción UART
    xTaskCreate((TaskFunction_t)taskUART3_receive, "UART3 RX", 128, (void *)USART3, 2, NULL);
    xTaskCreate((TaskFunction_t)taskUART1_GPS, "UART1 GPS", 1024, (void *)USART1, 2, NULL);
    //xTaskCreate((TaskFunction_t)taskUART3_TuteTransmit, "UART3 Tute", 128, (void *)USART3, 2, NULL);

    // Crear tareas para Test
    xTaskCreate(taskTestUART_Semaphore, "Test_Semaphore", 100, NULL, 2, NULL);  // Crear tarea para Test
    xTaskCreate(taskTest, "Test", 100, NULL, 2, NULL);  // Crear tarea para Test
    //xTaskCreate(taskPrintBuffer, "Print_buffer", 100, NULL, 2, NULL);  // Crear tarea para Test

    // Start RTOS Task scheduler
	vTaskStartScheduler();

    // The task scheduler is blocking, so we should never come here...
	for (;;);
    
	return 0;
}
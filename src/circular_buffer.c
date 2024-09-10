#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include "circular_buffer.h"
#include "uart.h"
#include "string.h"

// Definición de la estructura circular_buffer_t
struct circular_buffer_t {
    uint16_t in;      // Índice de entrada, incrementa cuando se añade un dato
    uint16_t out;     // Índice de salida, incrementa cuando se lee un dato
    uint16_t *data;   // Arreglo para almacenar los datos
    uint16_t mask;    // Máscara para hacer la operación circular (debe ser tamaño-1)
};

// ---------------------------------------------------------------

/**
 * @brief Crea e inicializa un buffer circular.
 *
 * @param size Tamaño del buffer (debe ser una potencia de 2).
 * @return Puntero a la estructura circular_buffer_t, o NULL si falla.
 */
circular_buffer_t *xCircularBufferCreate(uint16_t size) {
    // Crear la estructura circular_buffer_t
    circular_buffer_t *cbuff = pvPortMalloc(sizeof(circular_buffer_t));
    if (cbuff == NULL) {
        return NULL;  // Si falla la asignación, retornar NULL
    }

    // Asignar memoria para almacenar los datos
    cbuff->data = pvPortMalloc(size * sizeof(uint16_t));
    if (cbuff->data == NULL) {
        vPortFree(cbuff);  // Liberar la estructura si falla la asignación de datos
        return NULL;
    }

    // Inicializar índices y máscara
    cbuff->in = 0;
    cbuff->out = 0;
    cbuff->mask = size - 1;  // La máscara debe ser tamaño - 1 (asumiendo que el tamaño es potencia de 2)
    return cbuff;
}

/**
 * @brief Libera la memoria asociada a un buffer circular.
 *
 * @param cbuff Puntero al buffer circular que se va a eliminar.
 */
void vCircularBufferDelete(circular_buffer_t *cbuff) {
    if (cbuff != NULL) {
        vPortFree(cbuff->data);  // Liberar el arreglo de datos
        vPortFree(cbuff);        // Liberar la estructura
    }
}

/**
 * @brief Retorna el número de elementos disponibles en el buffer circular.
 *
 * @param cbuff Puntero al buffer circular.
 * @return Número de elementos disponibles en el buffer.
 */
uint16_t serial_available(circular_buffer_t *cbuff) {
    if (cbuff == NULL) {
        return 0;
    }
    return (uint16_t)(cbuff->in - cbuff->out);
}

/**
 * @brief Retorna el número de espacios disponibles para enviar datos.
 *
 * @param cbuff Puntero al buffer circular.
 * @return Número de espacios disponibles en el buffer.
 */
uint16_t serial_sendable(circular_buffer_t *cbuff) {
    if (cbuff == NULL) {
        return 0;
    }
    return (uint16_t)(cbuff->mask - (cbuff->in - cbuff->out));
}

/**
 * @brief Escribe un dato en el buffer circular.
 *
 * @param cbuff Puntero al buffer circular.
 * @param data Dato a escribir en el buffer.
 * @return pdTRUE si se escribe exitosamente, pdFALSE si el buffer está lleno.
 */
BaseType_t serial_write(circular_buffer_t *cbuff, const uint16_t data) {
    BaseType_t result = pdFALSE;
    if (cbuff != NULL) {
        result = push(cbuff, data);  // Escribir en el buffer
        // Aquí se podría habilitar la interrupción de transmisión si fuera necesario
        /*
        if ((uint16_t)(cbuff->in - cbuff->out) == 1) {
            usart_enable_tx_interrupt(usart_id);
        }
        */
    }
    return result;
}

/**
 * @brief Escribe una cadena de caracteres en el buffer circular.
 *
 * @param cbuff Puntero al buffer circular.
 * @param str Cadena de caracteres a escribir.
 * @return Número de caracteres enviados.
 */
uint16_t serial_puts(circular_buffer_t *cbuff, const char *str) {
    uint32_t nsend = 0;

    while (*str && serial_write(cbuff, *str++)) {
        nsend++;
    }

    return nsend;
}

/**
 * @brief Inserta un dato en el buffer circular.
 *
 * @param buff Puntero al buffer circular.
 * @param data Dato a insertar.
 * @return pdTRUE si se inserta exitosamente, pdFALSE si el buffer está lleno.
 */
BaseType_t push(circular_buffer_t *buff, const uint16_t data) {
    BaseType_t result = pdFALSE;
    // Verificar si hay espacio en el buffer (FIFO no lleno)
    if ((uint16_t)(buff->in - buff->out) < buff->mask) {
        result = pdTRUE;
        buff->data[buff->in & buff->mask] = data;  // Insertar dato en la posición calculada
        buff->in++;  // Incrementar el índice de entrada
    }
    return result;
}

/**
 * @brief Extrae un dato del buffer circular.
 *
 * @param buff Puntero al buffer circular.
 * @param data Puntero donde se almacenará el dato extraído.
 * @return pdPASS si se extrae exitosamente, pdFAIL si el buffer está vacío.
 */
BaseType_t pop(circular_buffer_t *buff, uint16_t *data) {
    // Verificar si hay datos en el buffer
    if (buff->in != buff->out) {
        *data = buff->data[buff->out & buff->mask];  // Extraer el dato en la posición calculada
        buff->out++;  // Incrementar el índice de salida
        return pdPASS;
    }
    return pdFAIL;  // Si no hay datos, retornar fallo
}

/**
 * @brief Devuelve en una cadena los datos no leídos del buffer.
 * 
 * Recoge los datos no leídos del buffer circular y los almacena en una cadena. 
 * Esta función no extrae los datos del buffer, solo los lee sin modificar los índices.
 * 
 * @param buff Puntero al buffer circular.
 * @param output Puntero a la cadena donde se almacenarán los datos no leídos.
 * @param max_len Tamaño máximo de la cadena para evitar desbordamientos.
 * @return uint16_t Cantidad de datos copiados a la cadena.
 */
uint16_t circular_buffer_to_string(circular_buffer_t *buff, char *output, uint16_t max_len) {
    if (buff == NULL || output == NULL) {
        return 0;
    }

    uint16_t count = 0;
    uint16_t current_out = buff->out; // Copia temporal del índice de lectura

    while (current_out != buff->in && count < max_len - 1) {
        output[count++] = (char)(buff->data[current_out & buff->mask]); // Lee el dato
        current_out++; // Avanza en el buffer
    }

    output[count] = '\0'; // Asegura que la cadena termine en '\0'
    return count;
}

/**
 * @brief Resetea el buffer circular.
 * 
 * Esta función vacía el buffer circular, reiniciando los índices `in` y `out` para
 * que el buffer se interprete como vacío.
 * 
 * @param buff Puntero al buffer circular a resetear.
 */
void circular_buffer_reset(circular_buffer_t *buff) {
    if (buff != NULL) {
        buff->in = 0;
        buff->out = 0;
    }
}


int Copy_from_to(const char *source, const char *pattern_start, const char *pattern_finish, char *dest) {
    // Encontrar la posición del patrón de inicio en la cadena fuente
    const char *start = strstr(source, pattern_start);
    if (start == NULL) {
        return -1; // Patrón de inicio no encontrado
    }

    // Avanzar el puntero al final del patrón de inicio
    //start += strlen(pattern_start);

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
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#define NUM_MUESTRAS 100

uint16_t buffer_adc[NUM_MUESTRAS];

int main() {
    stdio_init_all();
    sleep_ms(2000); // Esperar a que se conecte la consola

    // Inicializar ADC
    adc_init();
    adc_gpio_init(26);        // GPIO26 = ADC0
    adc_select_input(0);      // Seleccionar canal 0 (ADC0)

    // Configurar el DMA
    int canal_dma = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(canal_dma);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_16);   // ADC da 12 bits, usamos 16
    channel_config_set_read_increment(&config, false);             // El ADC FIFO no cambia de dirección
    channel_config_set_write_increment(&config, true);             // El buffer sí
    channel_config_set_dreq(&config, DREQ_ADC);                    // Activador del ADC

    dma_channel_configure(
        canal_dma,
        &config,
        buffer_adc,              // Destino: nuestro buffer
        &adc_hw->fifo,           // Origen: FIFO del ADC
        NUM_MUESTRAS,            // Número de transferencias
        false                    // No iniciar aún
    );

    // Activar ADC y su FIFO
    adc_fifo_setup(
        true,     // Activar FIFO
        true,     // Activar solicitud DMA
        1,        // Solicitud por muestra
        false,
        false
    );

    adc_run(true);  // Iniciar el ADC

    printf("Empieza con el While\n");

    while (true) {
        dma_channel_start(canal_dma);  // Iniciar DMA
        // Esperar a que termine
        dma_channel_wait_for_finish_blocking(canal_dma);

        // Procesar los datos
        printf("Muestras (voltios):\n");
        for (int i = 0; i < NUM_MUESTRAS; i++) {
            float voltaje = buffer_adc[i] * 3.3f / 4095.0f;
            printf("%.2f V\t", voltaje);
            if ((i + 1) % 10 == 0) printf("\n");
        }

        sleep_ms(1000);  // Esperar antes de la próxima lectura
    }

    return 0;
}

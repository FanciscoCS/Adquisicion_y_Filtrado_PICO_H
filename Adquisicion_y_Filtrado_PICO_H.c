#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/time.h"

#define NUM_MUESTRAS 100000

//const long frecuencia_deseada = 100000; // 10 Khz
const long frecuencia_adc = 48000000; // 48 Mhz
float divisor = frecuencia_adc / NUM_MUESTRAS;

uint16_t buffer_adc[NUM_MUESTRAS];

int main() {
    stdio_init_all();
    sleep_ms(3000); // Tiempo para que se conecte la consola USB

    // Inicializa ADC y el pin GPIO26 (canal 0)
    adc_init();
    adc_set_clkdiv(divisor); //Define velocidad del adc
    adc_gpio_init(26);
    adc_select_input(0);

    // Configura la FIFO del ADC
    adc_fifo_setup(
        true,     // Activar FIFO
        true,     // Activar solicitud de DMA
        1,        // Solicitar DMA con 1 muestra
        false,
        false
    );

    // Activa el ADC
    adc_run(true);

    // Solicita un canal DMA libre
    int canal_dma = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(canal_dma);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, DREQ_ADC); // El DMA se sincroniza con el ADC

    // Configura el canal DMA
    dma_channel_configure(
        canal_dma,
        &config,
        buffer_adc,            // Destino
        &adc_hw->fifo,         // Origen
        NUM_MUESTRAS,          // Cantidad de transferencias
        false                  // No empezar todavía
    );

    while (true) {

        absolute_time_t inicio = get_absolute_time();

        // Reiniciar el canal DMA para una nueva transferencia
        dma_channel_set_read_addr(canal_dma, &adc_hw->fifo, false);
        dma_channel_set_write_addr(canal_dma, buffer_adc, false);
        dma_channel_set_trans_count(canal_dma, NUM_MUESTRAS, true);

        // Espera a que termine la transferencia
        dma_channel_wait_for_finish_blocking(canal_dma);

        absolute_time_t fin = get_absolute_time();
        int64_t tiempo = absolute_time_diff_us(inicio, fin);//Microsegundos

        float muestras_por_segundo = (NUM_MUESTRAS * 1e6f) / tiempo;
        printf("Estimacion de Lecturas por segundo: %.2f\n", muestras_por_segundo);
        printf("\n\n");

        sleep_ms(2000); // Espera antes de Imprimir y volver a leer

        // Procesa y muestra los datos
        printf("Muestras (en voltios):\n");
        for (int i = 0; i < NUM_MUESTRAS; i++) {
            float voltaje = buffer_adc[i] * 3.3f / 4095.0f;
            printf("%.2f V\t", voltaje);
        }
        printf("\n\n");

    }

    return 0;
}
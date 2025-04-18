#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/time.h"
#include "kiss_fft.h"

#define NUM_MUESTRAS 4096

#define frecuencia_deseada 100000 // 10 Khz
const long frecuencia_adc = 48000000; // 48 Mhz
float divisor = frecuencia_adc / frecuencia_deseada;

uint16_t buffer_adc[NUM_MUESTRAS];
float buffer_dma[NUM_MUESTRAS];
kiss_fft_cpx in[NUM_MUESTRAS], out[NUM_MUESTRAS];
float magnitude[NUM_MUESTRAS / 2];

void compute_fft() {
    kiss_fft_cfg cfg = kiss_fft_alloc(NUM_MUESTRAS, 0, NULL, NULL);
    
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        in[i].r = buffer_dma[i];
        in[i].i = 0.0f;
    }

    kiss_fft(cfg, in, out);

    for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
        magnitude[i] = sqrtf(out[i].r * out[i].r + out[i].i * out[i].i);
    }

    free(cfg);
}

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

        // Reiniciar el canal DMA para una nueva transferencia
        dma_channel_set_read_addr(canal_dma, &adc_hw->fifo, false);
        dma_channel_set_write_addr(canal_dma, buffer_adc, false);
        dma_channel_set_trans_count(canal_dma, NUM_MUESTRAS, true);

        // Espera a que termine la transferencia
        dma_channel_wait_for_finish_blocking(canal_dma);

        for (int i = 0; i < NUM_MUESTRAS; i++) {
            buffer_dma[i] = (float)buffer_adc[i]; // Convert to float
        }

        compute_fft();

        //for (int i = 0; i < NUM_MUESTRAS; i++) {
        //    float voltaje = buffer_adc[i] * 3.3f / 4095.0f; 
        //    printf("%.2f V\t\n", voltaje);
        //}

        for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
            float freq = i * frecuencia_deseada / NUM_MUESTRAS;
            printf("Freq: %6.1f Hz | Mag: %6.2f\n", freq, magnitude[i]);
        }
        printf("\n\n");

        sleep_ms(2000); // Espera antes de Imprimir y volver a leer

    }

    return 0;
}
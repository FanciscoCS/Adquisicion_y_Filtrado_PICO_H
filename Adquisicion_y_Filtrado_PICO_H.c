#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/time.h"
#include "kiss_fft.h"
#include "math.h"

#define NUM_MUESTRAS 4096
#define PI 3.14159265358979323846

int Select = 1;
int V = 0;
double f1 = 0.0005f; //Frecuencia de Corte Normalizada
double f2 = 0.02f; // Freciencia de corte Normalizada
double h[NUM_MUESTRAS];
double H[NUM_MUESTRAS/2];
double y[NUM_MUESTRAS/2];

float output[NUM_MUESTRAS];
typedef struct {
    float x1, x2; // Entradas anteriores
    float y1, y2; // Salidas anteriores
} FilterState;

#define frecuencia_deseada 10000.0 // 10 Khz
const long frecuencia_adc = 48000000.0; // 48 Mhz
float divisor = frecuencia_adc / frecuencia_deseada;

uint16_t buffer_adc[NUM_MUESTRAS];
float buffer_dma[NUM_MUESTRAS];
kiss_fft_cpx in[NUM_MUESTRAS], out[NUM_MUESTRAS];
float magnitude[NUM_MUESTRAS / 2];
float freq[NUM_MUESTRAS];
float tiempo[NUM_MUESTRAS];

void compute_fft() {
    kiss_fft_cfg cfg = kiss_fft_alloc(NUM_MUESTRAS, 0, NULL, NULL);
    
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        in[i].r = buffer_dma[i];
        in[i].i = 0.0f;
    }

    kiss_fft(cfg, in, out);

    for (int i = 0; i < NUM_MUESTRAS/2; i++) {
        magnitude[i] = sqrtf(out[i].r * out[i].r + out[i].i * out[i].i);
    }

    free(cfg);
}

void generar_filtro_pasabanda() {
    //int M = (NUM_MUESTRAS- 1) / 2;

    for (int n = 0; n < NUM_MUESTRAS; n++) {
        float wn; // ventana de Hamming
        float M = 2 * PI * n / (NUM_MUESTRAS - 1);

        if (V == 0){
            wn = 0.54f - 0.46f * cosf(M); //Ventana Hamming
        } if (V == 1){
            wn = 0.5f-(0.5f*cosf(M)); // ventana de Hanning
        } if (V == 2){
            wn = 0.42f - 5.0f * cosf(M) + 0.08f * cosf(M); // Ventana de Blackman
        } if (V == 3){
            wn = 0.54f - 0.46f * cosf(M); // Ventana de Bartlett (Triangular)
        }
        //printf("%6.1f" , wn);
        
        float sinc1, sinc2;
        float t = n - ((NUM_MUESTRAS - 1)/2);

        // Evitar división por cero en el centro del sinc
        if (t == 0.0f) {
            sinc1 = 2 * f2;
            sinc2 = 2 * f1;
        } else {
            sinc1 = sinf(2 * PI * f2 * t) / (PI * t);
            sinc2 = sinf(2 * PI * f1 * t) / (PI * t);
        }

        h[n] = (sinc1 - sinc2) * wn;
        freq[n] = (float)n * frecuencia_deseada / NUM_MUESTRAS;
    }
}

void Tansformar_Filtro() {
    kiss_fft_cfg cfg = kiss_fft_alloc(NUM_MUESTRAS, 0, NULL, NULL);
    for (int i = 0; i < NUM_MUESTRAS; i++ ){
        in[i].r = h[i];
        in[i].i = 0.0;
    }
    
    kiss_fft(cfg, in, out);
    free(cfg);

    for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
        H[i] = sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
    }
}

void Multiplicacion() {
    for (int n = 0; n < NUM_MUESTRAS/2; n++) {
        y[n] = magnitude[n] * H[n];
    }
}

void Coeficientes_Pasabanda(float fs,
                          float *b0, float *b1, float *b2,
                          float *a1, float *a2) {
    float f0 = sqrtf(f1 * f2);              // Frecuencia central
    float Q = f0 / (f2 - f1);               // Calidad (factor Q)
    float omega = 2.0f * M_PI * f0 / fs;
    float sin_omega = sinf(omega);
    float cos_omega = cosf(omega);
    float alpha = sin_omega / (2.0f * Q);

    float a0 = 1.0f + alpha;
    *b0 = alpha / a0;
    *b1 = 0.0f;
    *b2 = -alpha / a0;
    *a1 = -2.0f * cos_omega / a0;
    *a2 = (1.0f - alpha) / a0;
}

float Filtro_Pasabanda(float x, FilterState *s,
                      float b0, float b1, float b2, float a1, float a2) {
    float y = b0 * x + b1 * s->x1 + b2 * s->x2
                   - a1 * s->y1 - a2 * s->y2;

    s->x2 = s->x1;
    s->x1 = x;
    s->y2 = s->y1;
    s->y1 = y;

    return y;
}

float Transformada (){
        kiss_fft_cfg cfg = kiss_fft_alloc(NUM_MUESTRAS, 0, NULL, NULL);
    for (int i = 0; i < NUM_MUESTRAS; i++ ){
        in[i].r = output[i];
        in[i].i = 0.0;
    }
    
    kiss_fft(cfg, in, out);
    free(cfg);

    for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
        y[i] = sqrt(out[i].r * out[i].r + out[i].i * out[i].i);
    }
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
            buffer_dma[i] = buffer_dma[i] * 3.3f / 4095.0f;
        }

        if(Select = 0){
            compute_fft();

            generar_filtro_pasabanda();

            Tansformar_Filtro();

            Multiplicacion();
        }if (Select = 1){
            float b0, b1, b2, a1, a2;
            Coeficientes_Pasabanda(frecuencia_deseada, &b0, &b1, &b2, &a1, &a2);

            FilterState state = {0};
            for (int i = 0; i < NUM_MUESTRAS; i++) {
                output[i] = Filtro_Pasabanda(buffer_dma[i], &state, b0, b1, b2, a1, a2);
            }

            //Transformada();
        }

        // Espectro de Frecuencia
        /*
        printf("BEGIN\n");
        for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
            printf("%6.1f,%6.2f\n", freq[i], y[i]);
        }
        printf("END");
        //printf("\n\n");
        */

        //Espectro de Tiempo
        printf("BEGIN\n");
        for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
            printf("%6.1f\n", output[i]);
        }
        printf("END");
        printf("\n");

        sleep_ms(1000); // Espera antes de volver a leer

    }

    return 0;
}
# ADQUISICION Y FILTRADO DE SEÑALES EMG CON RASPBERRY PICO
Como el titulo indica se trata de un codigo en C para la adquisicion de datos biometricos haciendo uso de un AD8232 y una Raspberry py pico, los objetivos 
del proyecto son:
- adquirir la señal
- Hacer uso de la FFT para conseguir las frecuencias
- Generar un filtro FIR
- Conseguir las frecuencias del filtro
- Implementar el filtro en la señal original
- Regresar la señal a su funcion en el tiempo

Se redactó el siguiente código con la ayuda de la documentación otorgada por Raspberry, comenzando por las librerías requeridas:
- #include <stdio.h>
- #include "pico/stdlib.h"
- #include "hardware/gpio.h"
- #include "hardware/adc.h"

Dentro del main se inicializa la librería stdlib y la librería adc, así como el pin gpio26, imprimiendo un mensaje para saber que todo está correcto antes de la lectura.
- stdio_init_all();
- printf("ADC Example, measuring GPIO26\n");
- adc_init();

Se selecciona la entrada 0 del adc, conectada al pin 26
- adc_select_input(0);

Para la lectura se crea un bucle while siempre activo, donde dentro se agrega una variable definida como factor de
conversión la cual sirve para convertir los valores crudos leídos por el adc a valores en escala de voltaje, se realiza
la lectura con “adc_read()” y se guarda dentro de la variable resultado, con “printf()” se imprimen los valores crudos
por un lado y por el otro los multiplicados por el factor de conversión para la tenencia de los valores en voltaje.
Concluye el while colocando un delay que permite la lectura de los valores.
- while (1) {
- const float conversion_factor = 3.3f / (1 << 12);
- uint16_t result = adc_read();
- printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
- sleep_ms(500);
- }


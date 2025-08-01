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

### INTEGRACION DEL DMA

Una vez la lectura de señales analógicas ha sido completada se implementó el dma con el fin de mejorar la latencia, el 
número de muestras por segundo y tener una mejor definición en la señal.
Se comenzó definiendo las librerías dma e irq.
- #include "hardware/dma.h"
- #include "hardware/irq.h"
  
Iniciando la variable “NUM_MUESTRAS” con un valor definido, generando un buffer de 16 bits con el tamaño del número de muestras.
- #define NUM_MUESTRAS 100
- uint16_t buffer_adc[NUM_MUESTRAS];
  
Dentro del Main() se configura el dma, empezando por crear variable para el canal y habilitarlo 
- int canal_dma = dma_claim_unused_channel(true);
  
Recuperamos una configuración básica (dma_channel_config)para el canal. 
- dma_channel_config config = dma_channel_get_default_config(canal_dma);
  
Modificamos la configuración para la aplicación en la que será empleada. configuramos el tamaño al valor inmediatamente mayor al
tamaño que se requiere, tamaño requerido 12 bits, tamaño inmediato superior 16 bits, indicar que la dirección de lectura no se
modifica, indicar que la dirección de escritura si cambia e indicamos que se active cada que haya nuevos valores disponibles en el adc.
- channel_config_set_transfer_data_size(&config, DMA_SIZE_16); // ADC da 12 bits, usamos 16
- channel_config_set_read_increment(&config, false); // El ADC FIFO no cambia de dirección
- channel_config_set_write_increment(&config, true); // El buffer sí
- channel_config_set_dreq(&config, DREQ_ADC); // Activador del adc.

Definimos el destino, el origen y el tamaño de la transferencia
- dma_channel_configure(
- canal_dma,
- &config,
- buffer_adc, // Destino: nuestro buffer
- &adc_hw->fifo, // Origen: FIFO del ADC
- NUM_MUESTRAS, // Número de transferencias
- alse // No iniciar aún
- );
- Activar el ADC y su FIFO
- adc_fifo_setup(
- true, // Activar FIFO
- true, // Activar solicitud DMA
- 1, // Solicitud por muestra
- false,
- false
- );
  
Se inicializa el ADC
- adc_run(true);
  
dentro del while() inicia el DMA y espera hasta completar el número de muestras
- dma_channel_start(canal_dma); // Iniciar DMA
- // Esperar a que termine
- dma_channel_wait_for_finish_blocking(canal_dma);
  
Generamos un for() encargado de pasar todos los datos del buffer uno por uno a través de la fórmula del factor de conversión 
y la impresión de los datos para su visualización.
- for (int i = 0; i < NUM_MUESTRAS; i++) {
- float voltaje = buffer_adc[i] * 3.3f / 4095.0f;
- printf("%.2f V\t", voltaje);
- if ((i + 1) % 10 == 0) printf("\n");
- }

### CONTROL DE FRECUENCIA DE ADQUISICION
Uno de los puntos importantes del proyecto es pasar la señal al dominio de la frecuencia y para esto es necesario conocer 
la frecuencia de muestreo exacta, así como la cantidad de muestras a tratar. Para eso se usó la velocidad del reloj del 
dma e investigó el número de ciclos necesarios para hacer una lectura, la velocidad de reloj para el dma es 48 MHz, mientras 
la cantidad regular de ciclos requerida por la raspberry pi pico para hacer una lectura ronda los 96 ciclos, con estos datos 
es posible conocer la velocidad máxima a la que se pueden solicitar datos por estos medios con una simple 
división, 48,000,000 / 96 ≈ 500,000 Lecturas por segundo, sabiendo esto se decidió implementar una configuración en 
el adc que permite definir el intervalo de ciclos entre muestra y muestra.
- adc_set_clkdiv(divisor); //Define velocidad del adc

Para definir el número de ciclos que debe esperar entre muestras definimos dos variables al principio del código que contengan 
la velocidad del reloj y la velocidad con la que deseamos que se recojan las muestras, dentro de otra variable llamada divisor, 
la velocidad del reloj es dividida entre la velocidad de las muestras.
Divisor = 48,000,000 / 10,000 ≈ 480 ciclos
- const long frecuencia_adc = 48000000; // 48 Mhz
- float divisor = frecuencia_adc / NUM_MUESTRAS; 
Para comprobar el correcto funcionamiento de este método se usó la librería “time” para medir el tiempo global antes de tomar
las muestras y después de tomar las muestras.
- absolute_time_t inicio = get_absolute_time();
- // Reiniciar el canal DMA para una nueva transferencia
- dma_channel_set_read_addr(canal_dma, &adc_hw->fifo, false);
- dma_channel_set_write_addr(canal_dma, buffer_adc, false);
- dma_channel_set_trans_count(canal_dma, NUM_MUESTRAS, true);
- dma_channel_wait_for_finish_blocking(canal_dma);	// Espera a que termine la transferencia
- absolute_time_t fin = get_absolute_time();

De este modo es posible calcular el tiempo que tardo en tomar las muestras y este a su vez usarlo para dividir el número de 
muestras multiplicadas por 1e6, debido a que la función time nos devuelve microsegundos, y terminar con la velocidad de muestras por segundo
- int64_t tiempo = absolute_time_diff_us(inicio, fin);//Microsegundos
- float muestras_por_segundo = (NUM_MUESTRAS * 1e6f) / tiempo;

### IMPLEMENTACION DE FFT
Es necesario mantener la eficiencia y velocidad lo más alto posible, con esto en mente se optó por una librería encargada de realizar la 
transformada rápida de Fourier en lugar de realizar una desde cero. Kissfft es una librería para C de fácil implementación y lo 
suficientemente rápida para el objetivo que se persigue. 

Se comenzó por abrir la terminal para copiar el repositorio Git con el comando:
- git clone https:/github.com/mborgerding/kissfft.git
Generar una capeta “kissfft” dentro del workspace del proyecto y copiar dentro la siguiente lista de archivos desde la carpeta recién clonada
- kiss_fft.c
- kiss_fft.h
- _kiss_fft_guts.h
- kiss_fft_log.h
De vuelta en VSCODE dentro del archivo CmakeList.txt será necesario agregar la librería. Para ello se modifican las líneas
“add executable(Adquisicion_Y_Filtrado_Pico2 Adquisicion_Y_Filtrado_Pico2)” añadiendo “kissfft/kiss_fft.c” entre los paréntesis y
“target_include_directories(FFT PRIVATE ${…}” Agregando “/kissfft” al final de la línea.

Para comprobar que se haya realizado con éxito se importa la librería dentro del código en C con “#include kiss_fft.h”, 
si no muestra un error, al compilar todo va bien y dentro de c_cpp_properties.json aparece la línea “${workspacefolder}/kissfft”, 
es indicativo de que está lista para trabajar.
Lo siguiente fue generar un arreglo de números de punto flotante con el tamaño de la muestra donde se guardarán automáticamente 
por el dma lar señales adquiridas por el adc.
- float buffer_dma[NUM_MUESTRAS];
  
Se declaran dos arreglos de tipo kiss_fft_cpx, “in[]” es el arreglo de entrada, cada elemento contiene una parte real e imaginaria y 
“out[]” es el arreglo que contiene la salida de la FFT, siendo esta la estructura de número complejo usada por la librería KissFFT, 
cada una con tamaño “NUM_MUESTRAS”.
- kiss_fft_cpx in[NUM_MUESTRAS], out[NUM_MUESTRAS];
  
El último arreglo tiene la función de almacenar la magnitud del espectro de la señal, solo requieren la mitad de las muestras debido 
a que, para señales reales, la FFT es simétrica, y solo la mitad del espectro contiene información única. La magnitud es calculada 
típicamente como la raíz de la suma del cuadro de la componente real y la imaginaria, de cada componente compleja del arreglo out.
- float magnitude[NUM_MUESTRAS / 2];
  
A continuación, se realizó una función llamada “compute_fft” sin parámetros ni valor de retorno, con el fin de calcular la transformada 
de la señal almacenada en el buffer_dma y obtener su magnitud espectral. “kiss_fft_cfg cfg = kiss_fft_alloc(NUM_MUESTRAS, 0, NULL, NULL);” 
Reserva y configura un objeto cfg necesario para ejecutar la FFT con la librería KissFFT. “0” indica que se trata de una transformada 
normal y no inversa, los dos NULL indican que Kiss FT usará malloc internamente para asignar memoria automáticamente. 
El objeto cfg se usará para ejecutar la transformada.
- void compute_fft() {
- kiss_fft_cfg cfg = kiss_fft_alloc(NUM_MUESTRAS, 0, NULL, NULL);
- for (int i = 0; i < NUM_MUESTRAS; i++) {
- in[i].r = buffer_dma[i];
- in[i].i = 0.0f;
- }
- kiss_fft(cfg, in, out);	//Aquí se ejecuta la Transformada de laplace
- for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
- magnitude[i] = sqrtf(out[i].r * out[i].r + out[i].i * out[i].i);
- }
- free(cfg);	//libera la memoria que fue asignada por kiss_fft_alloc.
- }
  
Esto último es importante para evitar fugas de memoria 
Después se convierte la lectura a una variable flotante dentro del main() y se llama a la función anterior.
- for (int i = 0; i < NUM_MUESTRAS; i++) {
- buffer_dma[i] = (float)buffer_adc[i]; // Convert to float
- }

Por último se imprimen las nuevas variables por la terminal.
- for (int i = 0; i < NUM_MUESTRAS / 2; i++) {
- float freq = i * frecuencia_deseada / NUM_MUESTRAS;
- printf("Freq: %6.1f Hz | Mag: %6.2f\n", freq, magnitude[i]);
- }
- printf("\n\n");
- sleep_ms(2000); // Espera antes de Imprimir y volver a leer

###FILTRO PASA BANDA FIR
Lo primero fue gerar las ventanas a probar (Hamming, Hanning, Blackman, Bartlett)
-void generar_filtro_pasabanda() {
- //int M = (NUM_MUESTRAS- 1) / 2;
- for (int n = 0; n < NUM_MUESTRAS; n++) {
- float wn; // ventana de Hamming
- float M = 2 * PI * n / (NUM_MUESTRAS - 1);
- if (V == 0){
- wn = 0.54f - 0.46f * cosf(M); //Ventana Hamming
- } if (V == 1){
- wn = 0.5f-(0.5f*cosf(M)); // ventana de Hanning
- } if (V == 2){
- wn = 0.42f - 5.0f * cosf(M) + 0.08f * cosf(M); // Ventana de Blackman
- } if (V == 3){
- wn = 0.54f - 0.46f * cosf(M); // Ventana de Bartlett (Triangular)
- }
- //printf("%6.1f" , wn);

Se deflararan las variables requeridas
- float sinc1, sinc2;
- float t = n - ((NUM_MUESTRAS - 1)/2);


Evitar división por cero en el centro del sinc
- if (t == 0.0f) {
- sinc1 = 2 * f2;
- sinc2 = 2 * f1;
- } else {
- sinc1 = sinf(2 * PI * f2 * t) / (PI * t);
- sinc2 = sinf(2 * PI * f1 * t) / (PI * t);
- }
- h[n] = (sinc1 - sinc2) * wn;
- //freq[n] = (float)n * frecuencia_deseada / NUM_MUESTRAS;
- }
- }

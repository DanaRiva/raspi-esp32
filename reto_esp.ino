#include "FFT.h" // include the library
#include <driver/i2s.h>
#include <WiFi.h>
#include "ThingSpeak.h"
#include "simpleDSP.h"

const char* ssid = "Tec-IoT";   // your network SSID (name) 
const char* password = "spotless.magnetic.bridge";   // your network password

WiFiClient client;

unsigned long myChannelNumber = 1;
const char * myWriteAPIKey = "LE4VKE1LVSEK7ARF";

// you shouldn't need to change these settings
#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 8000
//FFT values
#define FFT_N 512 
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_22
#define I2S_MIC_SERIAL_DATA GPIO_NUM_21

// don't mess around with this
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

float coefB[7] =
    {
        0.6306,
       -1.2612,
       -0.6306,
        2.5225,
       -0.6306,
       -1.2612,    
        0.6306
    };

float coefA[7] =
    {
            1,
      -2.1285,
       0.2949,
       1.8242,
      -0.8057,
      -0.3947,
       0.2099
    };

int32_t raw_samples[SAMPLE_BUFFER_SIZE];

IIR iir1;

float fft_input[FFT_N];
float fft_output[FFT_N];

unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

float res, sum;
double med = 0.001953125; // 1/512 -> 1/FFT_N
int e;
fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

void setup() 
{
    Serial.begin(115200); // use the serial port

    iirInit(&iir1, 7, coefB, 7, coefA);

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);

    WiFi.mode(WIFI_STA);   
    lastTime = millis();
  
    ThingSpeak.begin(client);  // Initialize ThingSpeak
}

void loop() 
{
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);

  for (int i = 0 ; i < FFT_N ; i++)
    real_fft_plan->input[i] = iirFilt(&iir1, raw_samples[i]);    

  fft_execute(real_fft_plan);
  
  sum = 0;
  for (int k = 1 ; k < real_fft_plan->size / 2 ; k++){
    sum += pow(abs(real_fft_plan->output[2*k]),2)/med;
  }

  res = 10 * log10(sum) + 55;
  if(WiFi.status() != WL_CONNECTED)
            while(WiFi.status() != WL_CONNECTED)
            {
                WiFi.begin(ssid, password); 
                delay(5000);     
            } 
            
  if(res >= 70 and (millis() - lastTime) > timerDelay){
    e = ThingSpeak.writeField(myChannelNumber, 1, res, myWriteAPIKey);
    Serial.println(e == 200 ? "Successful!!" : "Error ");
    lastTime = millis();
  }
  Serial.println(res);
  
}

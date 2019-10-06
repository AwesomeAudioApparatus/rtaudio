#if defined(__FREERTOS_ESP32__)
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include <math.h>

#include <cstdio>

/*---------------------------------------------------------------
                            CONFIG
---------------------------------------------------------------*/
#define I2S_READ_LEN      (16 * 1024)
#define I2S_FORMAT        (I2S_CHANNEL_FMT_RIGHT_LEFT)
#define I2S_CHANNEL_NUM   ((EXAMPLE_I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
#define I2S_ADC_UNIT      ADC_UNIT_1
#define I2S_ADC_CHANNEL   ADC1_CHANNEL_0


static const unsigned int SUPPORTED_SAMPLERATES[] = { 8000, 16000, 22050, 32000,
                                                      44100, 48000, 96000, 0};

                                                    
RtApiEsp32::RtApiEsp32(int i2s_num, int sample_rate, int sample_bits, ) 
{
  if( !contains( SUPPORTED_SAMPLERATES, sample_rate ) )  
    error("Unsupported Sample Rate.");
  else
  {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN,
        .sample_rate =  sample_rate,
        .bits_per_sample = sample_bits,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = I2S_FORMAT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,
        .use_apll = 1,
    };
    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
  }
}

RtApiEsp32::~RtApiEsp32()
{
  if ( stream_.state != STREAM_CLOSED )
    closeStream();
}

unsigned int RtApiEsp32::getDeviceCount( void )
{
  return 1;
}

RtAudio::DeviceInfo RtApiEsp32::getDeviceInfo( unsigned int /*device*/ )
{
  RtAudio::DeviceInfo info;
  info.probed = true;
  info.name = "ESP32/I2S";
  info.outputChannels = 2;
  info.inputChannels = 2;
  info.duplexChannels = 2;
  info.isDefaultOutput = true;
  info.isDefaultInput = true;

  for ( const unsigned int *sr = SUPPORTED_SAMPLERATES; *sr; ++sr )
    info.sampleRates.push_back( *sr );

  info.preferredSampleRate = 96000;
  info.nativeFormats = RTAUDIO_SINT16 | RTAUDIO_SINT32 | RTAUDIO_FLOAT32;

  return info;
}

void RtApiEsp32::closeStream( void )
{
    // TODO: Close the resources. May not ever happen in embedded scenario
  stream_.callbackInfo.isRunning = false;

  if ( stream_.userBuffer[0] ) {
    free( stream_.userBuffer[0] );
    stream_.userBuffer[0] = 0;
  }
  if ( stream_.userBuffer[1] ) {
    free( stream_.userBuffer[1] );
    stream_.userBuffer[1] = 0;
  }

  stream_.state = STREAM_CLOSED;
  stream_.mode = UNINITIALIZED;
}

void RtApiEsp32::callbackEvent( void )
{
}

void RtApiEsp32::startStream( void )
{
}

void RtApiEsp32::stopStream( void )
{
}

void RtApiEsp32::abortStream( void )
{
}

bool RtApiEsp32::probeDeviceOpen( unsigned int device, StreamMode mode,
                                  unsigned int channels, unsigned int firstChannel,
                                  unsigned int sampleRate, RtAudioFormat format,
                                  unsigned int *bufferSize, RtAudio::StreamOptions *options )
{
}

//******************** End of __LINUX_PULSE__ *********************//

#endif

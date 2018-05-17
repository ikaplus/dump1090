// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_bladerf.c: bladeRF support
//
// Copyright (c) 2017 FlightAware LLC
//
// This file is free software: you may copy, redistribute and/or modify it  
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your  
// option) any later version.  
//
// This file is distributed in the hope that it will be useful, but  
// WITHOUT ANY WARRANTY; without even the implied warranty of  
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License  
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "dump1090.h"
#include "sdr_soapy.h"

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <inttypes.h>

static struct {
    const char *device_str;
    const char *fpga_path;
    unsigned decimation;
    unsigned lpf_bandwidth;

    SoapySDRStream* rxStream;
    SoapySDRDevice* device;

    iq_convert_fn converter;
    struct converter_state *converter_state;

    unsigned block_size;
} Soapy;

void soapyShowHelp()
{
  printf("      soapy-specific options (use with --device-type soapy)\n");
  printf("\n");
}

bool soapyHandleOption(int argc, char **argv, int *jptr)
{
  MODES_NOTUSED(argc);
  MODES_NOTUSED(argv);
  MODES_NOTUSED(jptr);
  return false;
}

void soapyClose()
{
}

void soapyInitConfig()
{
    Soapy.device_str = NULL;
    Soapy.fpga_path = NULL;
    Soapy.decimation = 1;
    Soapy.lpf_bandwidth = 1750000;
    Soapy.device = NULL;
}

bool soapyOpen()
{
    if (Soapy.device) {
        return true;
    }

    int ret;

    //create device instance
    //args can be user defined or from the enumeration result
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "lime");
    Soapy.device = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (Soapy.device == NULL)
    {
        fprintf(stderr, "SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }
    if ((ret = SoapySDRDevice_setAntenna(Soapy.device, SOAPY_SDR_RX, 0, "LNAW")) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setAntenna fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
    if ((ret = SoapySDRDevice_setSampleRate(Soapy.device, SOAPY_SDR_RX, 0, 2400000)) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setSampleRate fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
    if ((ret = SoapySDRDevice_setFrequency(Soapy.device, SOAPY_SDR_RX, 0, 1090000000, NULL)) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setFrequency fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
    if ((ret = SoapySDRDevice_setBandwidth(Soapy.device, SOAPY_SDR_RX, 0, 8000000)) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setBandwidth fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
    /*
    if ((ret = SoapySDRDevice_setGainElement (Soapy.device, SOAPY_SDR_RX, 0, "TIA", 12.0)) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setGainElement fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
    if ((ret = SoapySDRDevice_setGainElement (Soapy.device, SOAPY_SDR_RX, 0, "LNA", Modes.gain / 10.0)) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setGainElement fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
    if ((ret = SoapySDRDevice_setGainElement (Soapy.device, SOAPY_SDR_RX, 0, "PGA", 19.0)) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setGainElement fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
*/
    if ((ret = SoapySDRDevice_setGain (Soapy.device, SOAPY_SDR_RX, 0, (Modes.gain * 2.0) / 10.0)) != 0)
      {
        fprintf(stderr, "SoapySDRDevice_setGain fail: %d %s\n", ret, SoapySDRDevice_lastError());
        return EXIT_FAILURE;
      }
    Soapy.converter = init_converter(INPUT_SC16,
                                       Modes.sample_rate,
                                       0,
                                       &Soapy.converter_state);
    return true;
}

static struct timespec thread_cpu;
static unsigned timeouts = 0;

static void *handle_soapy_samples(  void *dev,
                                    void *stream,
                                    void *meta,
                                    void *samples,
                                    size_t num_samples,
                                    void *user_data)
{
    static bool dropping = false;
    uint32_t slen = 0;
    unsigned block_duration;
    static uint64_t sampleCounter = 0;

    MODES_NOTUSED(dev);
    MODES_NOTUSED(stream);
    MODES_NOTUSED(meta);
    MODES_NOTUSED(user_data);

    // record initial time for later sys timestamp calculation
    struct timespec entryTimestamp;
    clock_gettime(CLOCK_REALTIME, &entryTimestamp);

    pthread_mutex_lock(&Modes.data_mutex);
    if (Modes.exit) {
        pthread_mutex_unlock(&Modes.data_mutex);
        return NULL;
    }

    unsigned next_free_buffer = (Modes.first_free_buffer + 1) % MODES_MAG_BUFFERS;
    struct mag_buf *outbuf = &Modes.mag_buffers[Modes.first_free_buffer];
    struct mag_buf *lastbuf = &Modes.mag_buffers[(Modes.first_free_buffer + MODES_MAG_BUFFERS - 1) % MODES_MAG_BUFFERS];
    unsigned free_bufs = (Modes.first_filled_buffer - next_free_buffer + MODES_MAG_BUFFERS) % MODES_MAG_BUFFERS;

    if (free_bufs == 0 || (dropping && free_bufs < MODES_MAG_BUFFERS/2)) {
        // FIFO is full. Drop this block.
        dropping = true;
        pthread_mutex_unlock(&Modes.data_mutex);
        return samples;
    }

    dropping = false;
    pthread_mutex_unlock(&Modes.data_mutex);

    // Compute the sample timestamp and system timestamp for the start of the block
    outbuf->sampleTimestamp = sampleCounter * 12e6 / Modes.sample_rate;
    sampleCounter += slen;
    block_duration = 1e9 * slen / Modes.sample_rate;

    // Get the approx system time for the start of this block
    clock_gettime(CLOCK_REALTIME, &outbuf->sysTimestamp);
    outbuf->sysTimestamp.tv_nsec -= block_duration;
    normalize_timespec(&outbuf->sysTimestamp);

    // Copy trailing data from last block (or reset if not valid)
    if (outbuf->dropped == 0) {
        memcpy(outbuf->data, lastbuf->data + lastbuf->length, Modes.trailing_samples * sizeof(uint16_t));
    } else {
        memset(outbuf->data, 0, Modes.trailing_samples * sizeof(uint16_t));
    }

    // Convert the new data
    outbuf->length = num_samples / 1;

        Soapy.converter(samples, &outbuf->data[Modes.trailing_samples], num_samples, Soapy.converter_state, &outbuf->mean_level, &outbuf->mean_power);
    // Push the new data to the demodulation thread
    pthread_mutex_lock(&Modes.data_mutex);

    // accumulate CPU while holding the mutex, and restart measurement
    end_cpu_timing(&thread_cpu, &Modes.reader_cpu_accumulator);
    start_cpu_timing(&thread_cpu);

    Modes.mag_buffers[next_free_buffer].dropped = 0;
    Modes.mag_buffers[next_free_buffer].length = 0;  // just in case
    Modes.first_free_buffer = next_free_buffer;

    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
    return samples;
}


void soapyRun()
{
    if (!Soapy.device) {
        return;
    }


    int16_t buffers[MODES_MAG_BUF_SAMPLES * 2];
    void *buffs[] = {buffers};

    SoapySDRDevice_setupStream(Soapy.device, &Soapy.rxStream, SOAPY_SDR_RX, SOAPY_SDR_CS16, NULL, 0, NULL);
    SoapySDRDevice_activateStream(Soapy.device, Soapy.rxStream, 0, 0, 0);

    unsigned long ms_per_transfer = 1000 * MODES_MAG_BUF_SAMPLES / Modes.sample_rate;

    start_cpu_timing(&thread_cpu);

    timeouts = 0; // reset to zero when we get a callback with some data
    while (1)
      {
	int flags;
	long long timeNs;
	int r;
	r = SoapySDRDevice_readStream(Soapy.device,
				  Soapy.rxStream,
				  buffs,
				  MODES_MAG_BUF_SAMPLES,
				  &flags,
				  &timeNs,
				   ms_per_transfer * 1000);
	if (r >= 0)
	  {
	    handle_soapy_samples(NULL, NULL, NULL, buffers, MODES_MAG_BUF_SAMPLES, NULL);
	  }
	else
	  {
	    printf ("SoapySDRDevice_readStream error %d %d\n", r, flags);
	  }
      }

}


// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/**********************************************************************************************

     Ensoniq ES5505/6 driver
     by Aaron Giles

Ensoniq OTIS - ES5505                                            Ensoniq OTTO - ES5506

  OTIS is a VLSI device designed in a 2 micron double metal        OTTO is a VLSI device designed in a 1.5 micron double metal
   CMOS process. The device is the next generation of audio         CMOS process. The device is the next generation of audio
   technology from ENSONIQ. This new chip achieves a new            technology from ENSONIQ. All calculations in the device are
   level of audio fidelity performance. These improvements          made with at least 18-bit accuracy.
   are achieved through the use of frequency interpolation
   and on board real time digital filters. All calculations       The major features of OTTO are:
   in the device are made with at least 16 bit accuracy.           - 68 pin PLCC package
                                                                   - On chip real time digital filters
 The major features of OTIS are:                                   - Frequency interpolation
  - 48 Pin dual in line package                                    - 32 independent voices
  - On chip real time digital filters                              - Loop start and stop posistions for each voice
  - Frequency interpolation                                        - Bidirectional and reverse looping
  - 32 independent voices (up from 25 in DOCII)                    - 68000 compatibility for asynchronous bus communication
  - Loop start and stop positions for each voice                   - separate host and sound memory interface
  - Bidirectional and reverse looping                              - 6 channel stereo serial communication port
  - 68000 compatibility for asynchronous bus communication         - Programmable clocks for defining serial protocol
  - On board pulse width modulation D to A                         - Internal volume multiplication and stereo panning
  - 4 channel stereo serial communication port                     - A to D input for pots and wheels
  - Internal volume multiplication and stereo panning              - Hardware support for envelopes
  - A to D input for pots and wheels                               - Support for dual OTTO systems
  - Up to 10MHz operation                                          - Optional compressed data format for sample data
                                                                   - Up to 16MHz operation
              ______    ______
            _|o     \__/      |_
 A17/D13 - |_|1             48|_| - VSS                                                           A A A A A A
            _|                |_                                                                  2 1 1 1 1 1 A
 A18/D14 - |_|2             47|_| - A16/D12                                                       0 9 8 7 6 5 1
            _|                |_                                                                  / / / / / / 4
 A19/D15 - |_|3             46|_| - A15/D11                                   H H H H H H H V V H D D D D D D /
            _|                |_                                              D D D D D D D S D D 1 1 1 1 1 1 D
      BS - |_|4             45|_| - A14/D10                                   0 1 2 3 4 5 6 S D 7 5 4 3 2 1 0 9
            _|                |_                                             ------------------------------------+
  PWZERO - |_|5             44|_| - A13/D9                                  / 9 8 7 6 5 4 3 2 1 6 6 6 6 6 6 6 6  |
            _|                |_                                           /                    8 7 6 5 4 3 2 1  |
    SER0 - |_|6             43|_| - A12/D8                                |                                      |
            _|       E        |_                                      SER0|10                                  60|A13/D8
    SER1 - |_|7      N      42|_| - A11/D7                            SER1|11                                  59|A12/D7
            _|       S        |_                                      SER2|12                                  58|A11/D6
    SER2 - |_|8      O      41|_| - A10/D6                            SER3|13              ENSONIQ             57|A10/D5
            _|       N        |_                                      SER4|14                                  56|A9/D4
    SER3 - |_|9      I      40|_| - A9/D5                             SER5|15                                  55|A8/D3
            _|       Q        |_                                      WCLK|16                                  54|A7/D2
 SERWCLK - |_|10            39|_| - A8/D4                            LRCLK|17               ES5506             53|A6/D1
            _|                |_                                      BCLK|18                                  52|A5/D0
   SERLR - |_|11            38|_| - A7/D3                             RESB|19                                  51|A4
            _|                |_                                       HA5|20                                  50|A3
 SERBCLK - |_|12     E      37|_| - A6/D2                              HA4|21                OTTO              49|A2
            _|       S        |_                                       HA3|22                                  48|A1
     RLO - |_|13     5      36|_| - A5/D1                              HA2|23                                  47|A0
            _|       5        |_                                       HA1|24                                  46|BS1
     RHI - |_|14     0      35|_| - A4/D0                              HA0|25                                  45|BS0
            _|       5        |_                                    POT_IN|26                                  44|DTACKB
     LLO - |_|15            34|_| - CLKIN                                 |   2 2 2 3 3 3 3 3 3 3 3 3 3 4 4 4 4  |
            _|                |_                                          |   7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3  |
     LHI - |_|16            33|_| - CAS                                   +--------------------------------------+
            _|                |_                                              B E E B E B B D S B B B E K B W W
     POT - |_|17     O      32|_| - AMUX                                      S B L N L S S D S S X S   L Q / /
            _|       T        |_                                              E E R E H M C V V A U A   C R R R
   DTACK - |_|18     I      31|_| - RAS                                       R R D H           R M C     I M
            _|       S        |_                                              _ D                 A
     R/W - |_|19            30|_| - E                                         T
            _|                |_                                              O
      MS - |_|20            29|_| - IRQ                                       P
            _|                |_
      CS - |_|21            28|_| - A3
            _|                |_
     RES - |_|22            27|_| - A2
            _|                |_
     VSS - |_|23            26|_| - A1
            _|                |_
     VDD - |_|24            25|_| - A0
             |________________|

***********************************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "es5506.h"

// Constants
#define MAX_VOICES         32
#define MAX_REGIONS        4
#define ULAW_MAXBITS       8
#define MAX_SAMPLE_CHUNK   10000

#define VOLUME_ACC_BIT     20

// Control register bits
#define CONTROL_STOP0      0x0001
#define CONTROL_STOP1      0x0002
#define CONTROL_STOPMASK   (CONTROL_STOP0 | CONTROL_STOP1)
#define CONTROL_LEI        0x0004
#define CONTROL_BS0        0x4000
#define CONTROL_BS1        0x8000
#define CONTROL_LPE        0x0008
#define CONTROL_BLE        0x0010
#define CONTROL_LOOPMASK   (CONTROL_LPE | CONTROL_BLE)
#define CONTROL_CA_MASK    0x1C00
#define CONTROL_LP_MASK    0x0300
#define CONTROL_LP3        0x0100
#define CONTROL_LP4        0x0200
#define CONTROL_IRQE       0x0020
#define CONTROL_DIR        0x0040
#define CONTROL_IRQ        0x0080
#define CONTROL_CMPD       0x2000

// Helper macro
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#define FINE_FILTER_BIT 16
#define FILTER_BIT      12
#define FILTER_SHIFT    ((FINE_FILTER_BIT) - (FILTER_BIT))
typedef struct {
    UINT32 control;
    UINT32 freqcount;
    UINT32 start;
    UINT32 end;
    UINT64 accum;
    UINT32 lvol;
    UINT32 rvol;
    UINT32 lvramp;
    UINT32 rvramp;
    UINT32 ecount;
    UINT32 k2;
    UINT32 k2ramp;
    UINT32 k1;
    UINT32 k1ramp;
    INT32 o4n1;
    INT32 o3n1;
    INT32 o3n2;
    INT32 o2n1;
    INT32 o2n2;
    INT32 o1n1;
    UINT32 exbank;
    UINT8 filtcount;
    UINT8 Muted;
} ES5506_Voice;

typedef struct {
    DEV_DATA _devData;

	void (*irq_func)(void *, UINT8);	// IRQ callback
	void *irq_param;

    ES5506_Voice voice[MAX_VOICES];
    UINT16* region_base[MAX_REGIONS];
    UINT32 region_size[MAX_REGIONS];
    UINT32 master_clock;
    UINT32 write_latch;
    UINT32 read_latch;
    UINT8 current_page;
    UINT8 active_voices;
    UINT8 mode;
    UINT8 wst;
    UINT8 wend;
    UINT8 lrend;
    UINT8 irqv;
    UINT8 sndtype;
	// chip configurations
	UINT8 acc_shift;
	UINT8 bank_shift;
	UINT8 bank_mask;
	UINT8 ca_shift;
	UINT8 ca_mask;
	UINT8 lp_shift;
	UINT8 exponent_bit;
	UINT8 mantissa_bit;
	INT8 volume_shift;
	UINT8 total_volume_bit;
	INT64 volume_acc_shift;
    UINT64 accum_mask;

    INT16* ulaw_lookup;
    UINT32* volume_lookup;
    INT32* scratch;
    UINT8 output_channels;
    UINT32 output_rate;
    
    DEVCB_SRATE_CHG SmpRateFunc;
    void* SmpRateData;
} ES5506_Chip;

// Prototypes
static void update_irq_state(ES5506_Chip *chip);
static void update_internal_irq_state(ES5506_Chip *chip);
static UINT16 read_sample(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 addr);
static INT32 interpolate(ES5506_Chip *chip, INT32 sample1, INT32 sample2, UINT64 accum);
static void generate_irq(ES5506_Chip *chip, ES5506_Voice *voice, int v);
static void generate_ulaw(ES5506_Chip *chip, ES5506_Voice *voice, INT32 *dest);
static void generate_pcm(ES5506_Chip *chip, ES5506_Voice *voice, INT32 *dest);
static UINT64 get_volume(ES5506_Chip* chip, UINT32 volume);
static INT64 get_sample(ES5506_Chip* chip, INT32 sample, UINT32 volume);
static void generate_samples(ES5506_Chip* chip, INT32** outputs, int samples);
static void update_envelopes(ES5506_Voice* voice);
static void apply_filters(ES5506_Chip* chip, ES5506_Voice* voice, INT32* sample);
static void compute_tables(ES5506_Chip* chip);
static void es5506_check_for_end_forward(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum);
static void es5506_check_for_end_reverse(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum);
static void es5505_check_for_end_forward(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum);
static void es5505_check_for_end_reverse(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum);

// Device interface
static UINT8 device_start_es5506(const ES5506_CFG *cfg, DEV_INFO* retDevInf);
static void device_stop_es5506(void* info);
static void device_reset_es5506(void* info);
static void es5506_pcm_update(void* param, UINT32 samples, DEV_SMPL** outputs);
static void es5506_write(void* info, UINT8 offset, UINT8 data);
static UINT8 es5506_read(void* info, UINT8 offset);
static void es5505_write(void* info, UINT8 offset, UINT16 data);
static UINT16 es5505_read(void* info, UINT8 offset);
static void es5506_write_rom(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void es5506_set_mute_mask(void* info, UINT32 MuteMask);
static void es5506_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr);

static DEVDEF_RWFUNC devFunc[] = {
    { RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, es5506_write },
    { RWF_REGISTER | RWF_READ,  DEVRW_A8D8, 0, es5506_read },
    { RWF_REGISTER | RWF_WRITE, DEVRW_A8D16, 0, es5505_write },
    { RWF_REGISTER | RWF_READ,  DEVRW_A8D16, 0, es5505_read },
    { RWF_MEMORY | RWF_WRITE,   DEVRW_BLOCK, 0, es5506_write_rom },
    { RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, es5506_set_mute_mask },
    { 0x00, 0x00, 0, NULL }
};

static DEV_DEF devDef = {
    "ES5506", "MAME", FCC_MAME,
    (DEVFUNC_START)device_start_es5506,
    device_stop_es5506,
    device_reset_es5506,
    es5506_pcm_update,
    NULL,
    es5506_set_mute_mask,
    NULL,
    es5506_set_srchg_cb,
    NULL,
    NULL,
    devFunc
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	if (devCfg != NULL && devCfg->flags)
		return "ES5506";
	return "ES5505";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return MAX_VOICES;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_ES5506 =
{
	DEVID_ES5506,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	{	// cores
		&devDef,
		NULL
	}
};


//-------------------------------------------------
// Device initialization
//-------------------------------------------------

static UINT8 device_start_es5506(const ES5506_CFG *cfg, DEV_INFO* retDevInf) {
    ES5506_Chip* chip = calloc(1, sizeof(ES5506_Chip));
    if (!chip) return 0xFF;

	chip->irq_func = NULL;
	chip->irq_param = NULL;

    chip->master_clock = cfg->_genCfg.clock;
	chip->irqv = 0x80;
    chip->output_channels = (cfg->output & 0x0F) ? (cfg->output & 0x0F) : 1;
    chip->sndtype = cfg->_genCfg.flags & 0x01;

	// set configurations
	chip->acc_shift = chip->sndtype ? 11 : 9;
	chip->bank_shift = chip->sndtype ? 14 : 2;
	chip->bank_mask = chip->sndtype ? 3 : 1;
	chip->ca_shift = chip->sndtype ? 10 : 8;
	chip->ca_mask = chip->sndtype ? 7 : 3;
	chip->lp_shift = chip->sndtype ? 8 : 10;
	chip->exponent_bit = 4;
	chip->mantissa_bit = chip->sndtype ? 8 : 4;
	chip->total_volume_bit = chip->sndtype ? 16 : 8;
    chip->accum_mask = chip->sndtype ? 0xFFFFFFFF : 0x1FFFFFFF;

    for (int i = 0; i < MAX_REGIONS; i++) {
        chip->region_base[i] = NULL;
        chip->region_size[i] = 0;
    }

    compute_tables(chip);
    chip->scratch = malloc(2 * MAX_SAMPLE_CHUNK * sizeof(INT32));

    chip->active_voices = 31;
    chip->output_rate = chip->master_clock / (16 * (chip->active_voices + 1));
    
    es5506_set_mute_mask(chip, 0x00000000);
    chip->_devData.chipInf = chip;
    INIT_DEVINF(retDevInf, &chip->_devData, chip->output_rate, &devDef);

    // ----------- DUMMY ROM PATCH FOR VGM LOGGING -----------
    // If no sample ROM was loaded, fill each region with a repeating saw pattern.
    for (int r = 0; r < MAX_REGIONS; ++r) {
        if (!chip->region_base[r]) {
            chip->region_size[r] = 0x10000; // 64k words (128KB)
            chip->region_base[r] = (UINT16*)calloc(chip->region_size[r], 1);
            for (int i = 0; i < chip->region_size[r] / 2; ++i)
                chip->region_base[r][i] = (i & 0xFF) << 8; // Sawtooth waveform
        }
    }
    // -------------------------------------------------------

    return 0x00;
}

static void device_stop_es5506(void* info) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    if (!chip) return;

    free(chip->ulaw_lookup);
    free(chip->volume_lookup);
    free(chip->scratch);
    for (int i = 0; i < MAX_REGIONS; i++) free(chip->region_base[i]);
    free(chip);
}

static void device_reset_es5506(void* info) {
    ES5506_Chip* chip = (ES5506_Chip*)info;

    memset(chip->voice, 0, sizeof(chip->voice));
    for (int i = 0; i < MAX_VOICES; i++) {
		chip->voice[i].exbank = 0;
        chip->voice[i].control = CONTROL_STOPMASK;
        chip->voice[i].lvol = chip->sndtype ? 0xFFFF : 0xFF;
        chip->voice[i].rvol = chip->sndtype ? 0xFFFF : 0xFF;
    }

    chip->active_voices = 31;
    chip->output_rate = chip->master_clock / (16 * (chip->active_voices + 1));
    if (chip->SmpRateFunc)
        chip->SmpRateFunc(chip->SmpRateData, chip->output_rate);
}

//-------------------------------------------------
// Audio generation core
//-------------------------------------------------

static UINT64 get_volume(ES5506_Chip* chip, UINT32 volume)
{
	return chip->volume_lookup[volume >> chip->volume_shift];
}

static INT64 get_sample(ES5506_Chip* chip, INT32 sample, UINT32 volume)
{
	return (sample * get_volume(chip, volume)) >> chip->volume_acc_shift;
}

static void es5506_pcm_update(void* param, UINT32 samples, DEV_SMPL** outputs) {
    ES5506_Chip* chip = (ES5506_Chip*)param;
    INT32* buffers[12];

    for (int i = 0; i < (chip->output_channels << 1); i++) {
        memset(outputs[i], 0, samples * sizeof(DEV_SMPL));
        buffers[i] = (INT32*)outputs[i];
    }

	generate_samples(chip, buffers, samples);
}

static UINT16 read_sample(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 addr)
{
	UINT8 bank = (voice->control >> chip->bank_shift) & chip->bank_mask;
	return chip->region_base[bank][((voice->exbank << 20) + addr) & (chip->region_size[bank] - 1)];
}

/**********************************************************************************************

     interpolate -- interpolate between two samples

***********************************************************************************************/

inline INT32 interpolate(ES5506_Chip *chip, INT32 sample1, INT32 sample2, UINT64 accum)
{
	const UINT32 shifted = 1 << chip->acc_shift;
	const UINT32 mask = shifted - 1;
	accum &= mask & chip->accum_mask;
	return (sample1 * (INT32)(shifted - accum) +
			sample2 * (INT32)(accum)) >> chip->acc_shift;
}

/**********************************************************************************************

     generate_ulaw -- general u-law decoding routine

***********************************************************************************************/

static void generate_ulaw(ES5506_Chip *chip, ES5506_Voice *voice, INT32 *dest)
{
	if (!chip->sndtype)
	{
		generate_pcm(chip, voice, dest);
		return;
	}
	UINT32 freqcount = voice->freqcount;
	UINT64 accum = voice->accum & chip->accum_mask;

	// outer loop, in case we switch directions
	if (!(voice->control & CONTROL_STOPMASK))
	{
		// two cases: first case is forward direction
		if (!(voice->control & CONTROL_DIR))
		{
			// fetch two samples
			INT32 val1 = read_sample(chip, voice, accum >> chip->acc_shift);
			INT32 val2 = read_sample(chip, voice, (accum >> chip->acc_shift) + 1);

			// decompress u-law
			val1 = chip->ulaw_lookup[val1 >> (16 - ULAW_MAXBITS)];
			val2 = chip->ulaw_lookup[val2 >> (16 - ULAW_MAXBITS)];

			// interpolate
			val1 = interpolate(chip, val1, val2, accum);
			accum = (accum + freqcount) & chip->accum_mask;

			// apply filters
			apply_filters(chip, voice, &val1);

			// update filters/volumes
			if (voice->ecount != 0)
				update_envelopes(voice);

			// apply volumes and add
			dest[0] += get_sample(chip, val1, voice->lvol) >> 7;
			dest[1] += get_sample(chip, val1, voice->rvol) >> 7;

			// check for loop end
			if (chip->sndtype)
				es5506_check_for_end_forward(chip, voice, &accum);
			else
				es5505_check_for_end_forward(chip, voice, &accum);
		}

		// two cases: second case is backward direction
		else
		{
			// fetch two samples
			INT32 val1 = read_sample(chip, voice, accum >> chip->acc_shift);
			INT32 val2 = read_sample(chip, voice, (accum >> chip->acc_shift) + 1);

			// decompress u-law
			val1 = chip->ulaw_lookup[val1 >> (16 - ULAW_MAXBITS)];
			val2 = chip->ulaw_lookup[val2 >> (16 - ULAW_MAXBITS)];

			// interpolate
			val1 = interpolate(chip, val1, val2, accum);
			accum = (accum - freqcount) & chip->accum_mask;

			// apply filters
			apply_filters(chip, voice, &val1);

			// update filters/volumes
			if (voice->ecount != 0)
				update_envelopes(voice);

			// apply volumes and add
			dest[0] += get_sample(chip, val1, voice->lvol) >> 7;
			dest[1] += get_sample(chip, val1, voice->rvol) >> 7;

			// check for loop end
			if (chip->sndtype)
				es5506_check_for_end_reverse(chip, voice, &accum);
			else
				es5505_check_for_end_reverse(chip, voice, &accum);
		}
	}
	else
	{
		// if we stopped, process any additional envelope
		if (voice->ecount != 0)
			update_envelopes(voice);
	}

	voice->accum = accum;
}



/**********************************************************************************************

     generate_pcm -- general PCM decoding routine

***********************************************************************************************/

static void generate_pcm(ES5506_Chip *chip, ES5506_Voice *voice, INT32 *dest)
{
	UINT32 freqcount = voice->freqcount;
	UINT64 accum = voice->accum & chip->accum_mask;

	// outer loop, in case we switch directions
	if (!(voice->control & CONTROL_STOPMASK))
	{
		// two cases: first case is forward direction
		if (!(voice->control & CONTROL_DIR))
		{
			// fetch two samples
			INT32 val1 = (INT16)read_sample(chip, voice, accum >> chip->acc_shift);
			INT32 val2 = (INT16)read_sample(chip, voice, (accum >> chip->acc_shift) + 1);

			// interpolate
			val1 = interpolate(chip, val1, val2, accum);
			accum = (accum + freqcount) & chip->accum_mask;

			// apply filters
			apply_filters(chip, voice, &val1);

			// update filters/volumes
			if (voice->ecount != 0)
				update_envelopes(voice);

			// apply volumes and add
			dest[0] += get_sample(chip, val1, voice->lvol) >> 7;
			dest[1] += get_sample(chip, val1, voice->rvol) >> 7;

			// check for loop end
			if (chip->sndtype)
				es5506_check_for_end_forward(chip, voice, &accum);
			else
				es5505_check_for_end_forward(chip, voice, &accum);
		}

		// two cases: second case is backward direction
		else
		{
			// fetch two samples
			INT32 val1 = (INT16)read_sample(chip, voice, accum >> chip->acc_shift);
			INT32 val2 = (INT16)read_sample(chip, voice, (accum >> chip->acc_shift) + 1);

			// interpolate
			val1 = interpolate(chip, val1, val2, accum);
			accum = (accum - freqcount) & chip->accum_mask;

			// apply filters
			apply_filters(chip, voice, &val1);

			// update filters/volumes
			if (voice->ecount != 0)
				update_envelopes(voice);

			// apply volumes and add
			dest[0] += get_sample(chip, val1, voice->lvol) >> 7;
			dest[1] += get_sample(chip, val1, voice->rvol) >> 7;

			// check for loop end
			if (chip->sndtype)
				es5506_check_for_end_reverse(chip, voice, &accum);
			else
				es5505_check_for_end_reverse(chip, voice, &accum);
		}
	}
	else
	{
		// if we stopped, process any additional envelope
		if (voice->ecount != 0)
			update_envelopes(voice);
	}

	voice->accum = accum;
}

/**********************************************************************************************

     update_irq_state -- update the IRQ state

***********************************************************************************************/


static void update_irq_state(ES5506_Chip *chip)
{
	// ES5505/6 irq line has been set high - inform the host
	if (chip->irq_func != NULL)
		chip->irq_func(chip->irq_param, 1); // IRQB set high
}

static void update_internal_irq_state(ES5506_Chip *chip)
{
	/*  Host (cpu) has just read the voice interrupt vector (voice IRQ ack).

	    Reset the voice vector to show the IRQB line is low (top bit set).
	    If we have any stacked interrupts (other voices waiting to be
	    processed - with their IRQ bit set) then they will be moved into
	    the vector next time the voice is processed.  In emulation
	    terms they get updated next time generate_samples() is called.
	*/

	chip->irqv = 0x80;

	if (chip->irq_func != NULL)
		chip->irq_func(chip->irq_param, 0); // IRQB set low
}


/**********************************************************************************************

     generate_irq -- general interrupt handling routine

***********************************************************************************************/

static void generate_irq(ES5506_Chip *chip, ES5506_Voice *voice, int v)
{
	// does this voice have it's IRQ bit raised?
	if (voice->control & CONTROL_IRQ)
	{
		// only update voice vector if existing IRQ is acked by host
		if (chip->irqv & 0x80)
		{
			// latch voice number into vector, and set high bit low
			chip->irqv = v & 0x1F;

			// take down IRQ bit on voice
			voice->control &= ~CONTROL_IRQ;

			// inform host of irq
			update_irq_state(chip);
		}
	}
}


static void generate_samples(ES5506_Chip* chip, INT32** buffers, int samples)
{
	// loop while we still have samples to generate
	for (int s = 0; s < samples; s++)
	{
		// loop over voices
		INT32 cursample[12] = { 0 };
		for (int v = 0; v <= chip->active_voices; v++)
		{
			ES5506_Voice *voice = &chip->voice[v];

			// special case: if end == start, stop the voice
			if (voice->start == voice->end)
				voice->control |= CONTROL_STOP0;

			const int voice_channel = (voice->control >> chip->ca_shift) & chip->ca_mask;
			const int channel = voice_channel % chip->output_channels;
			const int l = channel << 1;

			// generate from the appropriate source
			if ((voice->control & CONTROL_CMPD) && chip->sndtype)
				generate_ulaw(chip, voice, &cursample[l]);
			else
				generate_pcm(chip, voice, &cursample[l]);

			// does this voice have it's IRQ bit raised?
			generate_irq(chip, voice, v);
		}

		for (int c = 0; c < (chip->output_channels << 1); c++)
			buffers[c][s] += cursample[c];
	}
}

static void es5506_check_for_end_forward(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum)
{
	// are we past the end?
	if (*accum > voice->end && !(voice->control & CONTROL_LEI))
	{
		// generate interrupt if required
		if (voice->control & CONTROL_IRQE)
			voice->control |= CONTROL_IRQ;

		// handle the different types of looping
		switch (voice->control & CONTROL_LOOPMASK)
		{
			// non-looping
			case 0:
				voice->control |= CONTROL_STOP0;
				break;

			// uni-directional looping
			case CONTROL_LPE:
				*accum = (voice->start + (*accum - voice->end)) & chip->accum_mask;
				break;

			// trans-wave looping
			case CONTROL_BLE:
				*accum = (voice->start + (*accum - voice->end)) & chip->accum_mask;
				voice->control = (voice->control & ~CONTROL_LOOPMASK) | CONTROL_LEI;
				break;

			// bi-directional looping
			case CONTROL_LPE | CONTROL_BLE:
				*accum = (voice->end - (*accum - voice->end)) & chip->accum_mask;
				voice->control ^= CONTROL_DIR;
				break;
		}
	}
}

static void es5506_check_for_end_reverse(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum)
{
	// are we past the end?
	if (*accum < voice->start && !(voice->control & CONTROL_LEI))
	{
		// generate interrupt if required
		if (voice->control & CONTROL_IRQE)
			voice->control |= CONTROL_IRQ;

		// handle the different types of looping
		switch (voice->control & CONTROL_LOOPMASK)
		{
			// non-looping
			case 0:
				voice->control |= CONTROL_STOP0;
				break;

			// uni-directional looping
			case CONTROL_LPE:
				*accum = (voice->end - (voice->start - *accum)) & chip->accum_mask;
				break;

			// trans-wave looping
			case CONTROL_BLE:
				*accum = (voice->end - (voice->start - *accum)) & chip->accum_mask;
				voice->control = (voice->control & ~CONTROL_LOOPMASK) | CONTROL_LEI;
				break;

			// bi-directional looping
			case CONTROL_LPE | CONTROL_BLE:
				*accum = (voice->start + (voice->start - *accum)) & chip->accum_mask;
				voice->control ^= CONTROL_DIR;
				break;
		}
	}
}

// ES5505 : BLE is ignored when LPE = 0
static void es5505_check_for_end_forward(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum)
{
	// are we past the end?
	if (*accum > voice->end)
	{
		// generate interrupt if required
		if (voice->control & CONTROL_IRQE)
			voice->control |= CONTROL_IRQ;

		// handle the different types of looping
		switch (voice->control & CONTROL_LOOPMASK)
		{
			// non-looping
			case 0:
			case CONTROL_BLE:
				voice->control |= CONTROL_STOP0;
				break;

			// uni-directional looping
			case CONTROL_LPE:
				*accum = (voice->start + (*accum - voice->end)) & chip->accum_mask;
				break;

			// bi-directional looping
			case CONTROL_LPE | CONTROL_BLE:
				*accum = (voice->end - (*accum - voice->end)) & chip->accum_mask;
				voice->control ^= CONTROL_DIR;
				break;
		}
	}
}

static void es5505_check_for_end_reverse(ES5506_Chip *chip, ES5506_Voice *voice, UINT64 *accum)
{
	// are we past the end?
	if (*accum < voice->start)
	{
		// generate interrupt if required
		if (voice->control & CONTROL_IRQE)
			voice->control |= CONTROL_IRQ;

		// handle the different types of looping
		switch (voice->control & CONTROL_LOOPMASK)
		{
			// non-looping
			case 0:
			case CONTROL_BLE:
				voice->control |= CONTROL_STOP0;
				break;

			// uni-directional looping
			case CONTROL_LPE:
				*accum = (voice->end - (voice->start - *accum)) & chip->accum_mask;
				break;

			// bi-directional looping
			case CONTROL_LPE | CONTROL_BLE:
				*accum = (voice->start + (voice->start - *accum)) & chip->accum_mask;
				voice->control ^= CONTROL_DIR;
				break;
		}
	}
}


static void apply_filters(ES5506_Chip* chip, ES5506_Voice* voice, INT32* sample) {
    INT32 temp = *sample;
	INT32 filter_div_lp = (1 << FILTER_BIT);
	INT32 filter_div_hp = filter_div_lp << 1;
    
    // Pole 1
    temp = ((voice->k1 >> FILTER_SHIFT) * (temp - voice->o1n1) / filter_div_lp) + voice->o1n1;
    voice->o1n1 = temp;
    
    // Pole 2
    temp = ((voice->k1 >> FILTER_SHIFT) * (temp - voice->o2n1) / filter_div_lp) + voice->o2n1;
    voice->o2n2 = voice->o2n1;
    voice->o2n1 = temp;
    
    switch ((voice->control >> chip->lp_shift) & 3) {
        case 0:
            temp = temp - voice->o2n2 + 
                 ((voice->k2 >> FILTER_SHIFT) * voice->o3n1) / filter_div_hp + voice->o3n1 / 2;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = temp - voice->o3n2 + 
                 ((voice->k2 >> FILTER_SHIFT) * voice->o4n1) / filter_div_hp + voice->o4n1 / 2;
            voice->o4n1 = temp;
            break;
            
        case 1: // LP3
            temp = ((voice->k1 >> FILTER_SHIFT) * (temp - voice->o3n1) / filter_div_lp) + voice->o3n1;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = temp - voice->o3n2 + 
                 ((voice->k2 >> FILTER_SHIFT) * voice->o4n1) / filter_div_hp + voice->o4n1 / 2;
            voice->o4n1 = temp;
            break;
            
        case 2: // LP4
            temp = ((voice->k2 >> FILTER_SHIFT) * (temp - voice->o3n1) / filter_div_lp) + voice->o3n1;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = ((voice->k2 >> FILTER_SHIFT) * (temp - voice->o4n1) / filter_div_lp) + voice->o4n1;
            voice->o4n1 = temp;
            break;
            
        case 3: // LP3 | LP4
            temp = ((voice->k1 >> FILTER_SHIFT) * (temp - voice->o3n1) / filter_div_lp) + voice->o3n1;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = ((voice->k2 >> FILTER_SHIFT) * (temp - voice->o4n1) / filter_div_lp) + voice->o4n1;
            voice->o4n1 = temp;
            break;
    }
    
    *sample = temp;
}

static void update_envelopes(ES5506_Voice* voice) {
    voice->ecount--;
    
    if (voice->lvramp) {
        voice->lvol += (INT8)voice->lvramp;
        voice->lvol = CLAMP(voice->lvol, 0, 0xFFFF);
    }
    if (voice->rvramp) {
        voice->rvol += (INT8)voice->rvramp;
        voice->rvol = CLAMP(voice->rvol, 0, 0xFFFF);
    }
    
    if (voice->k1ramp && ((INT32)voice->k1ramp >= 0 || !(voice->filtcount & 7))) {
        voice->k1 += (INT8)voice->k1ramp;
        voice->k1 = CLAMP(voice->k1, 0, 0xFFFF);
    }
    if (voice->k2ramp && ((INT32)voice->k2ramp >= 0 || !(voice->filtcount & 7))) {
        voice->k2 += (INT8)voice->k2ramp;
        voice->k2 = CLAMP(voice->k2, 0, 0xFFFF);
    }
    
    voice->filtcount++;
}

//-------------------------------------------------
// Register I/O
//-------------------------------------------------

static void es5506_write(void* info, UINT8 offset, UINT8 data) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
	if (offset & 0x80)
	{
		ES5506_Voice* voice = &chip->voice[offset & 0x1F];
		voice->exbank = data;
		return;
	}
	else
	{
		ES5506_Voice* voice = &chip->voice[chip->current_page & 0x1F];
		if (chip->sndtype)
		{
			UINT32 shift = 8 * (offset & 3);
			UINT32 mask = ~(0xFF << (24 - shift));
			UINT32 value = data << (24 - shift);

			chip->write_latch = (chip->write_latch & mask) | value;

			if (shift != 24) return;

			switch (chip->current_page & 0x60) {
				case 0x00:
					switch (offset >> 2) {
						case 0x0: voice->control = chip->write_latch & 0xFFFF; break;
						case 0x1: voice->freqcount = chip->write_latch & 0x1FFFF; break;
						case 0x2: voice->lvol = chip->write_latch & 0xFFFF; break;
						case 0x3: voice->lvramp = (chip->write_latch >> 8) & 0xFF; break;
						case 0x4: voice->rvol = chip->write_latch & 0xFFFF; break;
						case 0x5: voice->rvramp = (chip->write_latch >> 8) & 0xFF; break;
						case 0x6:
							voice->ecount = chip->write_latch & 0x1FF;
							voice->filtcount = 0;
							break;
						case 0x7: voice->k2 = chip->write_latch & 0xFFFF; break;
						case 0x8: voice->k2ramp = ((chip->write_latch & 0xFF00) >> 8) | ((chip->write_latch & 0x0001) >> 31); break;
						case 0x9: voice->k1 = chip->write_latch & 0xFFFF; break;
						case 0xA: voice->k1ramp = ((chip->write_latch & 0xFF00) >> 8) | ((chip->write_latch & 0x0001) >> 31); break;
						case 0xB:
							chip->active_voices = chip->write_latch & 0x1F;
							chip->output_rate = chip->master_clock / (16 * (chip->active_voices + 1));
							if (chip->SmpRateFunc)
								chip->SmpRateFunc(chip->SmpRateData, chip->output_rate);
							break;
						case 0xC: // MODE
							break;
					}
					break;
					
				case 0x20:
					switch (offset >> 2) {
						case 0x0: voice->control = chip->write_latch & 0xFFFF; break;
						case 0x1: voice->start = chip->write_latch & 0xFFFFF800; break;
						case 0x2: voice->end = chip->write_latch & 0xFFFFFF80; break;
						case 0x3: voice->accum = chip->write_latch & 0xFFFFFFFF; break;
						case 0x4: voice->o4n1 = (INT32)(chip->write_latch << 14) >> 14; break;
						case 0x5: voice->o3n1 = (INT32)(chip->write_latch << 14) >> 14; break;
						case 0x6: voice->o3n2 = (INT32)(chip->write_latch << 14) >> 14; break;
						case 0x7: voice->o2n1 = (INT32)(chip->write_latch << 14) >> 14; break;
						case 0x8: voice->o2n2 = (INT32)(chip->write_latch << 14) >> 14; break;
						case 0x9: voice->o1n1 = (INT32)(chip->write_latch << 14) >> 14; break;
						case 0xA: // W_ST
						case 0xB: // W_END
						case 0xC: // LR_END
							break;
					}
					break;
					
				case 0x40:
					switch (offset >> 2) {
						case 0x0: // Channel 0 Left
						case 0x1: // Channel 0 Right
						case 0x2: // Channel 1 Left
						case 0x3: // Channel 1 Right
						case 0x4: // Channel 2 Left
						case 0x5: // Channel 2 Right
						case 0x6: // Channel 3 Left
						case 0x7: // Channel 3 Right
						case 0x8: // Channel 4 Left
						case 0x9: // Channel 4 Right
						case 0xA: // Channel 5 Left
						case 0xB: // Channel 5 Right
							break;
						case 0xC: // EMPTY
							break;
					}
					break;
			}
			switch (offset >> 2)
			{
				case 0xD: // PAR - read only
					break;
				case 0xE: // IRQV - read only
					break;
				case 0xF: // PAGE
					chip->current_page = chip->write_latch & 0x7F;
					break;
			}

			chip->write_latch = 0;
		}
		else
		{
			UINT32 shift = 8 * (offset & 1);
			UINT32 mask = 0xFF << shift;
			UINT32 value = data << shift;

			switch (chip->current_page & 0x60) {
				case 0x00:
					switch (offset >> 1) {
						case 0x0: voice->control = (voice->control & ~mask) | (value & mask) | 0xF000; break;
						case 0x1:
							voice->freqcount <<= 1;
							voice->freqcount = ((voice->freqcount & ~mask) | (value & mask)) & 0xFFFE;
							voice->freqcount >>= 1;
							break;
						case 0x2: voice->start = ((voice->start & ~(mask << 16)) | ((value & mask) << 16)) & 0x1FFFFFE0; break;
						case 0x3: voice->start = ((voice->start & ~mask) | (value & mask)) & 0x1FFFFFE0; break;
						case 0x4: voice->end = ((voice->end & ~(mask << 16)) | ((value & mask) << 16)) & 0x1FFFFFE0; break;
						case 0x5: voice->end = ((voice->end & ~mask) | (value & mask)) & 0x1FFFFFE0; break;
						case 0x6: voice->k2 = ((voice->k2 & ~mask) | (value & mask)) & 0xFFF0; break;
						case 0x7: voice->k1 = ((voice->k1 & ~mask) | (value & mask)) & 0xFFF0; break;
						case 0x8:
							voice->lvol <<= 8;
							voice->lvol = ((voice->lvol & ~mask) | (value & mask)) & 0xFF00;
							voice->lvol >>= 8;
							break;
						case 0x9:
							voice->rvol <<= 8;
							voice->rvol = ((voice->rvol & ~mask) | (value & mask)) & 0xFF00;
							voice->rvol >>= 8;
							break;
						case 0xA: voice->accum = ((voice->accum & ~(mask << 16)) | ((value & mask) << 16)) & 0x1FFFFFFF; break;
						case 0xB: voice->accum = ((voice->accum & ~mask) | (value & mask)) & 0x1FFFFFFF; break;
					}
					break;
					
				case 0x20:
					switch (offset >> 1) {
						case 0x0: voice->control = (voice->control & ~mask) | (value & mask) | 0xF000; break;
						case 0x1: voice->o4n1 = (voice->o4n1 & ~mask) | (value & mask); break;
						case 0x2: voice->o3n1 = (voice->o3n1 & ~mask) | (value & mask); break;
						case 0x3: voice->o3n2 = (voice->o3n2 & ~mask) | (value & mask); break;
						case 0x4: voice->o2n1 = (voice->o2n1 & ~mask) | (value & mask); break;
						case 0x5: voice->o2n2 = (voice->o2n2 & ~mask) | (value & mask); break;
						case 0x6: voice->o1n1 = (voice->o1n1 & ~mask) | (value & mask); break;
					}
					break;
					
				case 0x40:
					switch (offset >> 1) {
						case 0x0:  // CH0L
						case 0x1:  // CH0R
						case 0x2:  // CH1L
						case 0x3:  // CH1R
						case 0x4:  // CH2L
						case 0x5:  // CH2R
						case 0x6:  // CH3L
						case 0x7:  // CH3R
							break;
						case 0x8: // SERMODE
							break;
						case 0x9: // PAR
							break;
					}
					break;
			}
			switch (offset)
			{
				case 0xD:
					chip->active_voices = ((chip->active_voices & ~mask) | (value & mask)) & 0x1F;
					chip->output_rate = chip->master_clock / (16 * (chip->active_voices + 1));
					if (chip->SmpRateFunc)
						chip->SmpRateFunc(chip->SmpRateData, chip->output_rate);
					break;
				case 0xE: // IRQV - read only
					break;
				case 0xF: // PAGE
					chip->current_page = ((chip->current_page & ~mask) | (value & mask)) & 0x7F;
					break;
			}

			chip->write_latch = 0;
		}
	}
}

static UINT8 es5506_read(void* info, UINT8 offset) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    ES5506_Voice* voice = &chip->voice[chip->current_page & 0x1F];

    if (chip->sndtype)
	{
		UINT32 value = chip->read_latch;
		if ((offset & 3) == 0)
		{
			switch (chip->current_page & 0x60) {
				case 0x00:
					switch (offset >> 2) {
						case 0x0: chip->read_latch = voice->control; break;
						case 0x1: chip->read_latch = voice->freqcount; break;
						case 0x2: chip->read_latch = voice->lvol; break;
						case 0x3: chip->read_latch = voice->lvramp << 8; break;
						case 0x4: chip->read_latch = voice->rvol; break;
						case 0x5: chip->read_latch = voice->rvramp << 8; break;
						case 0x6: chip->read_latch = voice->ecount; break;
						case 0x7: chip->read_latch = voice->k2; break;
						case 0x8: chip->read_latch = (voice->k2 << 8) | (voice->k2ramp >> 31); break;
						case 0x9: chip->read_latch = voice->k1; break;
						case 0xA: chip->read_latch = (voice->k1 << 8) | (voice->k1ramp >> 31); break;
						case 0xB: chip->read_latch = chip->active_voices; break;
						case 0xC: // MODE
							break;
					}
					break;

				case 0x20:
					switch (offset >> 2) {
						case 0x0: chip->read_latch = voice->control; break;
						case 0x1: chip->read_latch = voice->start; break;
						case 0x2: chip->read_latch = voice->end; break;
						case 0x3: chip->read_latch = voice->accum; break;
						case 0x4: chip->read_latch = voice->o4n1 & 0x3FFFF; break;
						case 0x5: chip->read_latch = voice->o3n1 & 0x3FFFF; break;
						case 0x6: chip->read_latch = voice->o3n2 & 0x3FFFF; break;
						case 0x7: chip->read_latch = voice->o2n1 & 0x3FFFF; break;
						case 0x8: chip->read_latch = voice->o2n2 & 0x3FFFF; break;
						case 0x9: chip->read_latch = voice->o1n1 & 0x3FFFF; break;
						case 0xA: // W_ST
						case 0xB: // W_END
						case 0xC: // MODE
							break;
					}
					break;

				case 0x40:
					break;
			}
			switch (offset >> 2)
			{
				case 0xD: // PAR - read only
					break;
				case 0xE: // IRQV - read only
					value = chip->irqv;
					update_internal_irq_state(chip);
					break;
				case 0xF: // PAGE
					chip->read_latch = chip->current_page;
					break;
			}
		}

		return (value >> (24 - (offset & 3)*8)) & 0xFF;
	}
	else
	{
		UINT16 value = 0;
		switch (chip->current_page & 0x60) {
			case 0x00:
				switch (offset >> 1) {
					case 0x0: value = voice->control | 0xF000; break;
					case 0x1: value = voice->freqcount << 1; break;
					case 0x2: value = voice->start >> 16; break;
					case 0x3: value = voice->start & 0xFFFF; break;
					case 0x4: value = voice->end >> 16; break;
					case 0x5: value = voice->end & 0xFFFF; break;
					case 0x6: value = voice->k2; break;
					case 0x7: value = voice->k1; break;
					case 0x8: value = (voice->lvol << 8) & 0xFF00; break;
					case 0x9: value = (voice->rvol << 8) & 0xFF00; break;
					case 0xA: value = voice->accum >> 16; break;
					case 0xB: value = voice->accum & 0xFFFF; break;
				}
				break;
				
			case 0x20:
				switch (offset >> 1) {
					case 0x0: value = voice->control | 0xF000; break;
					case 0x1: value = voice->o4n1 & 0xFFFF; break;
					case 0x2: value = voice->o3n1 & 0xFFFF; break;
					case 0x3: value = voice->o3n2 & 0xFFFF; break;
					case 0x4: value = voice->o2n1 & 0xFFFF; break;
					case 0x5: value = voice->o2n2 & 0xFFFF; break;
					case 0x6: value = voice->o1n1 & 0xFFFF; break;
				}
				break;
				
			case 0x40:
				switch (offset >> 1) {
					case 0x0: // CH0L
					case 0x1: // CH0R
					case 0x2: // CH1L
					case 0x3: // CH1R
					case 0x4: // CH2L
					case 0x5: // CH2R
					case 0x6: // CH3L
					case 0x7: // CH3R
						break;
					case 0x8: // SERMODE
						break;
					case 0x9: // PAR
						break;
				}
				break;
		}
		switch (offset >> 1)
		{
			case 0xD: // ACT
				value = chip->active_voices;
				break;
			case 0xE: // IRQV
				value = chip->irqv;
				update_internal_irq_state(chip);
				break;
			case 0xF: // PAGE
				value = chip->current_page;
				break;
		}

		return (value >> ((offset & 1)*8)) & 0xFF;
	}
}

static void es5505_write(void* info, UINT8 offset, UINT16 data) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
	if (chip->sndtype)
		return;

    ES5506_Voice* voice = &chip->voice[chip->current_page & 0x1F];

    switch (chip->current_page & 0x60) {
        case 0x00:
            switch (offset) {
                case 0x0: voice->control = 0xF000 | (data & 0x0FFF); break;
                case 0x1: voice->freqcount = (data & 0xFFFE) >> 1; break;
                case 0x2: voice->start = (voice->start & ~0x1FFF0000) | (((UINT32)data << 16) & 0x1FFF0000); break;
                case 0x3: voice->start = (voice->start & ~0x0000FFE0) | (data & 0x0000FFE0); break;
                case 0x4: voice->end = (voice->end & ~0x1FFF0000) | (((UINT32)data << 16) & 0x1FFF0000); break;
                case 0x5: voice->end = (voice->end & ~0x0000FFE0) | (data & 0x0000FFE0); break;
                case 0x6: voice->k2 = data & 0xFFF0; break;
                case 0x7: voice->k1 = data & 0xFFF0; break;
                case 0x8: voice->lvol = (data >> 8) & 0x00FF; break;
                case 0x9: voice->rvol = (data >> 8) & 0x00FF; break;
                case 0xA: voice->accum = (voice->accum & ~0x1FFF0000) | (((UINT32)data << 16) & 0x1FFF0000); break;
                case 0xB: voice->accum = (voice->accum & ~0x0000FFFF) | (data & 0x0000FFFF); break;
            }
            break;
            
        case 0x20:
            switch (offset) {
                case 0x0: voice->control = 0xF000 | (data & 0x0FFF); break;
                case 0x1: voice->o4n1 = data & 0xFFFF; break;
                case 0x2: voice->o3n1 = data & 0xFFFF; break;
                case 0x3: voice->o3n2 = data & 0xFFFF; break;
                case 0x4: voice->o2n1 = data & 0xFFFF; break;
                case 0x5: voice->o2n2 = data & 0xFFFF; break;
                case 0x6: voice->o1n1 = data & 0xFFFF; break;
            }
            break;
            
        case 0x40:
            switch (offset) {
                case 0x0:  // CH0L
                case 0x1:  // CH0R
                case 0x2:  // CH1L
                case 0x3:  // CH1R
                case 0x4:  // CH2L
                case 0x5:  // CH2R
                case 0x6:  // CH3L
                case 0x7:  // CH3R
					break;
				case 0x8: // SERMODE
					break;
				case 0x9: // PAR
					break;
            }
            break;
    }
	switch (offset)
	{
		case 0xD:
			chip->active_voices = data & 0x1F;
			chip->output_rate = chip->master_clock / (16 * (chip->active_voices + 1));
			if (chip->SmpRateFunc)
				chip->SmpRateFunc(chip->SmpRateData, chip->output_rate);
			break;
		case 0xE: // IRQV - read only
			break;
		case 0xF: // PAGE
			chip->current_page = data & 0x7F;
			break;
	}
}

static UINT16 es5505_read(void* info, UINT8 offset) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
	if (chip->sndtype)
		return 0;

    ES5506_Voice* voice = &chip->voice[chip->current_page & 0x1F];
    UINT16 value = 0;

    switch (chip->current_page & 0x60) {
        case 0x00:
            switch (offset) {
                case 0x0: value = voice->control | 0xF000; break;
                case 0x1: value = voice->freqcount << 1; break;
                case 0x2: value = voice->start >> 16; break;
                case 0x3: value = voice->start & 0xFFFF; break;
                case 0x4: value = voice->end >> 16; break;
                case 0x5: value = voice->end & 0xFFFF; break;
				case 0x6: value = voice->k2; break;
				case 0x7: value = voice->k1; break;
				case 0x8: value = (voice->lvol << 8) & 0xFF00; break;
				case 0x9: value = (voice->rvol << 8) & 0xFF00; break;
                case 0xA: value = voice->accum >> 16; break;
                case 0xB: value = voice->accum & 0xFFFF; break;
            }
            break;
            
        case 0x20:
            switch (offset) {
                case 0x0: value = voice->control | 0xF000; break;
                case 0x1: value = voice->o4n1 & 0xFFFF; break;
                case 0x2: value = voice->o3n1 & 0xFFFF; break;
                case 0x3: value = voice->o3n2 & 0xFFFF; break;
                case 0x4: value = voice->o2n1 & 0xFFFF; break;
                case 0x5: value = voice->o2n2 & 0xFFFF; break;
                case 0x6: value = voice->o1n1 & 0xFFFF; break;
            }
            break;
            
        case 0x40:
            switch (offset) {
                case 0x0: // CH0L
                case 0x1: // CH0R
                case 0x2: // CH1L
                case 0x3: // CH1R
                case 0x4: // CH2L
                case 0x5: // CH2R
                case 0x6: // CH3L
                case 0x7: // CH3R
					break;
				case 0x8: // SERMODE
					break;
				case 0x9: // PAR
					break;
            }
            break;
    }
	switch (offset)
	{
		case 0xD: // ACT
			value = chip->active_voices;
			break;
		case 0xE: // IRQV
			value = chip->irqv;
			update_internal_irq_state(chip);
			break;
		case 0xF: // PAGE
			value = chip->current_page;
			break;
	}

    return value;
}

static void es5506_write_rom(void* info, UINT32 offset, UINT32 length, const UINT8* data) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    UINT8 region = (offset >> 28) & 0x03;
    UINT8 is8bit = (offset >> 31) & 0x01;
    offset &= 0x0FFFFFFF;

    if (is8bit) {
        offset *= 2;
        length *= 2;
        if (chip->region_size[region] < offset + length) {
            chip->region_base[region] = realloc(chip->region_base[region], offset + length);
            chip->region_size[region] = offset + length;
        }
        for (UINT32 i = 0; i < length/2; i++)
            chip->region_base[region][offset/2 + i] = data[i] << 8;
    } else {
        if (chip->region_size[region] < offset + length) {
            chip->region_base[region] = realloc(chip->region_base[region], offset + length);
            chip->region_size[region] = offset + length;
        }
        memcpy(chip->region_base[region] + offset/2, data, length);
    }
}

static void es5506_set_mute_mask(void* info, UINT32 MuteMask) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    for (int i = 0; i < MAX_VOICES; i++)
        chip->voice[i].Muted = (MuteMask >> i) & 1;
}

static void es5506_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    chip->SmpRateFunc = CallbackFunc;
    chip->SmpRateData = DataPtr;
}

static void compute_tables(ES5506_Chip* chip) {
    chip->ulaw_lookup = malloc((1 << ULAW_MAXBITS) * sizeof(INT16));
	// generate ulaw lookup table
	for (int i = 0; i < (1 << ULAW_MAXBITS); i++)
	{
		UINT16 rawval = (i << (16 - ULAW_MAXBITS)) | (1 << (15 - ULAW_MAXBITS));
		UINT8 exponent = rawval >> 13;
		UINT32 mantissa = (rawval << 3) & 0xffff;

		if (exponent == 0)
			chip->ulaw_lookup[i] = (INT16)(mantissa) >> 7;
		else
		{
			mantissa = (mantissa >> 1) | (~mantissa & 0x8000);
			chip->ulaw_lookup[i] = (INT16)(mantissa) >> (7 - exponent);
		}
	}

	UINT32 volume_bit = (chip->exponent_bit + chip->mantissa_bit);
	chip->volume_shift = chip->total_volume_bit - volume_bit;
	UINT32 volume_len = 1 << volume_bit;
    chip->volume_lookup = malloc(volume_len * sizeof(UINT32));
	// generate volume lookup table
	UINT32 exponent_shift = 1 << chip->exponent_bit;
	UINT32 exponent_mask = exponent_shift - 1;

	UINT32 mantissa_len = (1 << chip->mantissa_bit);
	UINT32 mantissa_mask = (mantissa_len - 1);
	UINT32 mantissa_shift = exponent_shift - chip->mantissa_bit - 1;

	for (int i = 0; i < volume_len; i++)
	{
		UINT32 exponent = (i >> chip->mantissa_bit) & exponent_mask;
		UINT32 mantissa = (i & mantissa_mask) | mantissa_len;

		chip->volume_lookup[i] = (mantissa << mantissa_shift) >> (exponent_shift - exponent);
	}
	chip->volume_acc_shift = (16 + exponent_mask) - VOLUME_ACC_BIT;

}
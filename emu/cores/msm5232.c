#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../stdtype.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "../EmuCores.h"
#include "../logging.h"
#include "../SoundDevs.h"
#include "../EmuHelper.h"
#include "../RatioCntr.h"
#include "msm5232.h"

// Voice state with all MAME components
typedef struct {
    uint8_t mode;           // 0=tone, 1=noise
    uint16_t pitch;
    uint16_t TG_count_period;
    int32_t TG_count;
    uint8_t TG_cnt;
    uint8_t TG_out16;
    uint8_t TG_out8;
    uint8_t TG_out4;
    uint8_t TG_out2;
    
    int32_t egvol;          // Current envelope volume (0-2047)
    int32_t eg;             // Envelope generator value
    int32_t eg_sect;        // Envelope section (-1=off)
    int32_t counter;        // Envelope counter
    uint8_t eg_arm;         // Envelope arm
    uint8_t GF;             // Gate flag
    uint8_t mute;
    
    double ar_rate;         // Attack rate
    double dr_rate;         // Decay rate
    double rr_rate;         // Release rate
} MSM5232_VOICE;

// Device state with complete MAME emulation
typedef struct {
    DEV_DATA _devData;
    
    MSM5232_VOICE voi[MSM5232_NUM_CHANNELS];
    uint32_t noise_rng;
    uint32_t noise_clocks;
    int32_t noise_step;
    int32_t noise_cnt;
    
    // Control registers
    uint8_t control1;
    uint8_t control2;
    uint32_t EN_out16[2];
    uint32_t EN_out8[2];
    uint32_t EN_out4[2];
    uint32_t EN_out2[2];
    
    // Timing
    uint32_t clock;
    uint32_t sample_rate;
    uint32_t UpdateStep;
    RATIO_CNTR cycle_cntr;
    
    // Capacitor data
    double capacitors[8];
    double ar_tbl[8];
    double dr_tbl[16];
} MSM5232_STATE;

// MSM5232 ROM Table (88 entries)
static const uint16_t MSM5232_ROM[88] = {
    0x1F9F, 0x1DDF, 0x1C1F, 0x1A9F, 0x193F, 0x17DF, 0x169F, 0x15BF, 0x145F, 0x131F,
    0x11FF, 0x10FF, 0x0F9F, 0x0E9F, 0x0D9F, 0x0C9F, 0x0B9F, 0x0A9F, 0x09FF, 0x091F,
    0x08DF, 0x089F, 0x085F, 0x081F, 0x07DF, 0x079F, 0x075F, 0x071F, 0x06DF, 0x069F,
    0x065F, 0x061F, 0x05DF, 0x059F, 0x055F, 0x051F, 0x04DF, 0x049F, 0x045F, 0x041F,
    0x03DF, 0x039F, 0x035F, 0x031F, 0x02DF, 0x029F, 0x025F, 0x021F, 0x01DF, 0x019F,
    0x015F, 0x011F, 0x00DF, 0x009F, 0x005F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F,
    0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F,
    0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F,
    0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F,
    0x001F, 0x001F
};

// libvgm interface functions
static UINT8 device_start(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop(void* info);
static void device_reset(void* info);
static void device_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void write_reg(void* info, UINT8 reg, UINT8 value);
static void set_mute_mask(void* info, UINT32 muteMask);

static DEVDEF_RWFUNC devFunc[] = {
    {RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, write_reg},
    {RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, set_mute_mask},
    {0x00, 0x00, 0, NULL}
};

static DEV_DEF devDef = {
    "MSM5232", "MAME", FCC_MAME,
    device_start, device_stop, device_reset, device_update,
    NULL, set_mute_mask, NULL, NULL, NULL, NULL, devFunc
};

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 8;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_MSM5232 =
{
	DEVID_MSM5232,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	{	// cores
		&devDef,
		NULL
	}
};

// Device initialization
static UINT8 device_start(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf) {
    MSM5232_STATE* chip;
    const MSM5232_CFG* msm_cfg = (const MSM5232_CFG*)cfg;

    chip = calloc(1, sizeof(MSM5232_STATE));
    if (!chip) return 0xFF;

    // Initialize parameters
    chip->clock = msm_cfg->clock;
    memcpy(chip->capacitors, msm_cfg->capacitors, sizeof(double)*8);
    chip->sample_rate = chip->clock / CLOCK_RATE_DIVIDER;
    SRATE_CUSTOM_HIGHEST(cfg->srMode, chip->sample_rate, cfg->smplRate);

    // Initialize rate tables
    const double R51 = 870.0, R52 = 17400.0, R53 = 101000.0;
    chip->UpdateStep = (uint32_t)((1 << STEP_SH) * (double)chip->sample_rate / chip->clock);

    // Attack rate table
    for (int i = 0; i < 8; i++) {
        double clockscale = (double)chip->clock / 2119040.0;
        int rcp = 1 << ((i & 4) ? (i & ~2) : i);
        chip->ar_tbl[i] = (rcp / clockscale) * R51;
    }

    // Decay rate table
    for (int i = 0; i < 16; i++) {
        double clockscale = (double)chip->clock / 2119040.0;
        int rcp = 1 << ((i & 4) ? (i & ~2) : i);
        chip->dr_tbl[i] = (i < 8) ? (rcp / clockscale) * R52 : (rcp / clockscale) * R53;
    }

    // Initialize voices
    for (int i = 0; i < MSM5232_NUM_CHANNELS; i++) {
        MSM5232_VOICE* v = &chip->voi[i];
        v->ar_rate = chip->ar_tbl[0] * chip->capacitors[i];
        v->dr_rate = chip->dr_tbl[0] * chip->capacitors[i];
        v->rr_rate = chip->dr_tbl[0] * chip->capacitors[i];
        v->eg_sect = -1;
    }

    // Initialize noise
    chip->noise_rng = 1;
    chip->noise_step = (int32_t)((1 << STEP_SH) / 128.0 * (double)chip->clock / chip->sample_rate);

    RC_SET_RATIO(&chip->cycle_cntr, chip->clock, chip->sample_rate);
    chip->_devData.chipInf = chip;
    INIT_DEVINF(retDevInf, &chip->_devData, chip->sample_rate, &devDef);
    return 0x00;
}

// Reset device
static void device_reset(void* info) {
    MSM5232_STATE* chip = (MSM5232_STATE*)info;
    memset(chip->voi, 0, sizeof(chip->voi));
    chip->noise_rng = 1;
    chip->noise_clocks = 0;
    chip->control1 = chip->control2 = 0;

    for (int i = 0; i < MSM5232_NUM_CHANNELS; i++) {
        MSM5232_VOICE* v = &chip->voi[i];
        v->eg_sect = -1;
        v->eg = 0;
        v->egvol = 0;
        v->counter = 0;
        v->GF = 0;
        v->ar_rate = chip->ar_tbl[0] * chip->capacitors[i];
        v->dr_rate = chip->dr_tbl[0] * chip->capacitors[i];
        v->rr_rate = chip->dr_tbl[0] * chip->capacitors[i];
    }
}

// Update envelopes (MAME's EG_voices_advance)
static void update_envelopes(MSM5232_STATE* chip) {
    const int VMAX = 32768;
    const int VMIN = 0;

    for (int i = 0; i < MSM5232_NUM_CHANNELS; i++) {
        MSM5232_VOICE* v = &chip->voi[i];
        
        switch (v->eg_sect) {
        case 0: // Attack
            if (v->eg < VMAX) {
                v->counter -= (int)((VMAX - v->eg) / v->ar_rate);
                if (v->counter <= 0) {
                    int n = -v->counter / chip->sample_rate + 1;
                    v->eg += n;
                    if (v->eg > VMAX) v->eg = VMAX;
                    v->counter += n * chip->sample_rate;
                }
                if (!v->eg_arm && v->eg >= VMAX * 0.8f)
                    v->eg_sect = 1;
            }
            break;

        case 1: // Decay
            if (v->eg > VMIN) {
                v->counter -= (int)((v->eg - VMIN) / v->dr_rate);
                if (v->counter <= 0) {
                    int n = -v->counter / chip->sample_rate + 1;
                    v->eg -= n;
                    if (v->eg < VMIN) v->eg = VMIN;
                    v->counter += n * chip->sample_rate;
                }
            } else {
                v->eg_sect = -1;
            }
            break;

        case 2: // Release
            if (v->eg > VMIN) {
                v->counter -= (int)((v->eg - VMIN) / v->rr_rate);
                if (v->counter <= 0) {
                    int n = -v->counter / chip->sample_rate + 1;
                    v->eg -= n;
                    if (v->eg < VMIN) v->eg = VMIN;
                    v->counter += n * chip->sample_rate;
                }
            } else {
                v->eg_sect = -1;
            }
            break;
        }
        v->egvol = v->eg >> 4; // Convert to 0-2047
    }
}

// Update tone generators (MAME's TG_group_advance)
static void update_tone(MSM5232_STATE* chip, int group) {
    int start = group * 4;
    for (int i = start; i < start+4; i++) {
        MSM5232_VOICE* v = &chip->voi[i];
        if (v->mode != 0 || v->mute) continue;

        int32_t remain = 1 << STEP_SH;
        while (remain > 0) {
            int32_t chunk = (remain < v->TG_count) ? remain : v->TG_count;
            remain -= chunk;
            v->TG_count -= chunk;
            
            if (v->TG_count <= 0) {
                v->TG_count += v->TG_count_period;
                v->TG_cnt++;
            }
        }
    }
}

// Update noise generator (MAME's noise logic)
static void update_noise(MSM5232_STATE* chip, uint32_t clocks) {
    chip->noise_cnt += chip->noise_step * clocks;
    while (chip->noise_cnt >= (1 << 16)) {
        chip->noise_cnt -= (1 << 16);
        uint32_t feedback = (chip->noise_rng >> 15) ^ (chip->noise_rng >> 12);
        chip->noise_rng = (chip->noise_rng << 1) | (feedback & 1);
        if ((chip->noise_rng & 0x10000) != ((chip->noise_rng << 1) & 0x10000)) {
            chip->noise_clocks++;
        }
    }
}

// Main update function
static void device_update(void* info, UINT32 samples, DEV_SMPL** outputs) {
    MSM5232_STATE* chip = (MSM5232_STATE*)info;
    DEV_SMPL* outL = outputs[0];
    DEV_SMPL* outR = outputs[1];

    for (UINT32 i = 0; i < samples; i++) {
        RC_STEP(&chip->cycle_cntr);
        uint32_t clocks = RC_GET_VAL(&chip->cycle_cntr);

        // Process components
        update_envelopes(chip);
        update_noise(chip, clocks);
        
        int32_t mixL = 0, mixR = 0;
        
        // Process groups
        for (int g = 0; g < 2; g++) {
            update_tone(chip, g);
            
            // Mix group outputs
            for (int v = g*4; v < (g+1)*4; v++) {
                MSM5232_VOICE* voice = &chip->voi[v];
                if (voice->mute) continue;

                int32_t sample = 0;
                if (voice->mode == 1) { // Noise
                    sample = (chip->noise_rng & 0x10000) ? voice->egvol : -voice->egvol;
                } else { // Tone
                    uint8_t cnt = voice->TG_cnt;
                    sample = ((cnt & voice->TG_out16) ? 1 : -1) * voice->egvol;
                }
                
                // Apply output enables
                if (g == 0) {
                    if (chip->EN_out16[0]) sample &= (voice->TG_out16 ? -1 : 0);
                    if (chip->EN_out8[0]) sample &= (voice->TG_out8 ? -1 : 0);
                } else {
                    if (chip->EN_out16[1]) sample &= (voice->TG_out16 ? -1 : 0);
                    if (chip->EN_out8[1]) sample &= (voice->TG_out8 ? -1 : 0);
                }
                
                mixL += sample;
                mixR += sample;
            }
        }

        outL[i] = mixL / 8;
        outR[i] = mixR / 8;
        RC_MASK(&chip->cycle_cntr);
    }
}

// Register write handler
static void write_reg(void* info, UINT8 reg, UINT8 value) {
    MSM5232_STATE* chip = (MSM5232_STATE*)info;
    
    if (reg < 0x08) { // Pitch registers
        int ch = reg & 7;
        MSM5232_VOICE* v = &chip->voi[ch];
        
        v->GF = (value >> 7) & 1;
        if (value & 0x80) {
            if (value >= 0xD8) { // Noise mode
                v->mode = 1;
                v->eg_sect = 0;
            } else { // Tone mode
                v->mode = 0;
                v->pitch = value & 0x7F;
                const uint16_t rom = MSM5232_ROM[v->pitch];
                v->TG_count_period = (rom & 0x1FF) * chip->UpdateStep / 2;
                uint8_t n = (rom >> 9) & 7;
                v->TG_out16 = 1 << n;
                v->TG_out8 = 1 << ((n > 0) ? n-1 : 0);
                v->TG_out4 = 1 << ((n > 1) ? n-2 : 0);
                v->TG_out2 = 1 << ((n > 2) ? n-3 : 0);
                v->eg_sect = 0;
            }
        } else {
            v->eg_sect = v->eg_arm ? 1 : 2;
        }
    }
    else switch (reg) {
        case 0x08: // Group 1 attack
            for (int i=0; i<4; i++)
                chip->voi[i].ar_rate = chip->ar_tbl[value&7] * chip->capacitors[i];
            break;
            
        case 0x09: // Group 2 attack
            for (int i=4; i<8; i++)
                chip->voi[i].ar_rate = chip->ar_tbl[value&7] * chip->capacitors[i];
            break;
            
        case 0x0C: // Group 1 control
            chip->control1 = value;
            chip->EN_out16[0] = (value & 1) ? ~0 : 0;
            chip->EN_out8[0] = (value & 2) ? ~0 : 0;
            chip->EN_out4[0] = (value & 4) ? ~0 : 0;
            chip->EN_out2[0] = (value & 8) ? ~0 : 0;
            break;
            
        case 0x0D: // Group 2 control
            chip->control2 = value;
            chip->EN_out16[1] = (value & 1) ? ~0 : 0;
            chip->EN_out8[1] = (value & 2) ? ~0 : 0;
            chip->EN_out4[1] = (value & 4) ? ~0 : 0;
            chip->EN_out2[1] = (value & 8) ? ~0 : 0;
            break;
    }
}

// Mute handling
static void set_mute_mask(void* info, UINT32 muteMask) {
    MSM5232_STATE* chip = (MSM5232_STATE*)info;
    for (int i = 0; i < MSM5232_NUM_CHANNELS; i++)
        chip->voi[i].mute = (muteMask >> i) & 1;
}

// Device cleanup
static void device_stop(void* info) {
    free(info);
}
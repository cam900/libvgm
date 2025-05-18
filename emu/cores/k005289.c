#include <stdlib.h>
#include <string.h>
#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "k005289.h"

#define PROM_SIZE 0x200 // 512 bytes (256 per channel)

typedef struct _k005289_state {
    DEV_DATA _devData;
    
    struct {
        UINT16 pitch;
        UINT16 freq;
        UINT8 volume;
        UINT8 waveform;
        INT16 counter;
        UINT8 addr;
    } voice[2];
    
    UINT8 prom[PROM_SIZE]; // Internal PROM storage
    UINT32 clock;
    UINT32 rate;
    UINT8 mute_mask;
} k005289_state;

static void k005289_update(void* param, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_k005289(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_k005289(void* chip);
static void device_reset_k005289(void* chip);
static void k005289_set_mute_mask(void* chip, UINT32 mute_mask);
static void k005289_write(void* chip, UINT8 address, UINT16 data);

// Add PROM write handler
static void k005289_write_prom(void* chip, UINT16 offset, UINT8 data);

static DEVDEF_RWFUNC devFunc[] = {
    {RWF_REGISTER | RWF_WRITE, DEVRW_A8D16, 0, k005289_write},
    {RWF_MEMORY | RWF_WRITE, DEVRW_A16D8, 0, k005289_write_prom}, // PROM writes
    {RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, k005289_set_mute_mask},
    {0x00, 0x00, 0, NULL}
};

static DEV_DEF devDef = {
    "K005289", "MAME", FCC_MAME,
    device_start_k005289,
    device_stop_k005289,
    device_reset_k005289,
    k005289_update,

    NULL,	// SetOptionBits
    k005289_set_mute_mask,
    NULL,	// SetPanning
    NULL,	// SetSampleRateChangeCallback
    NULL,	// SetLoggingCallback
    NULL,	// LinkDevice

    devFunc
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "K005289";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 2;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_K005289 =
{
	DEVID_K005289,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	{	// cores
		&devDef,
		NULL
	}
};


// Device start with internal PROM
static UINT8 device_start_k005289(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf) {
    k005289_state* info = calloc(1, sizeof(k005289_state));
    if (!info) return 0xFF;
    
    info->clock = cfg->clock;
    info->rate = info->clock; // Use full clock rate
    memset(info->prom, 0, PROM_SIZE);
    
    device_reset_k005289(info);
    info->mute_mask = 0x00;
    info->_devData.chipInf = info;
    INIT_DEVINF(retDevInf, &info->_devData, info->rate, &devDef);
    return 0x00;
}

// Device stop
static void device_stop_k005289(void* chip) {
    free(chip);
}

// Device reset
static void device_reset_k005289(void* chip) {
    k005289_state* info = (k005289_state*)chip;
    memset(info->voice, 0, sizeof(info->voice));
}

// Sound generation
// PROM data write handler
static void k005289_write_prom(void* chip, UINT16 offset, UINT8 data) {
    k005289_state* info = (k005289_state*)chip;
    if (offset < PROM_SIZE)
        info->prom[offset] = data & 0x0F; // Store only 4 bits
}

// Modified update function using internal PROM
static void k005289_update(void* param, UINT32 samples, DEV_SMPL** outputs) {
    k005289_state* info = (k005289_state*)param;
    DEV_SMPL* buffer = outputs[0];
    
    for (UINT32 i = 0; i < samples; i++) {
        INT32 mix = 0;
        
        for (int ch = 0; ch < 2; ch++) {
            if (info->mute_mask & (1 << ch)) continue;
            
            if (--info->voice[ch].counter < 0) {
                info->voice[ch].addr = (info->voice[ch].addr + 1) & 0x1F;
                info->voice[ch].counter = info->voice[ch].freq;
            }
            
            UINT16 prom_addr = (ch * 0x100) | 
                            (info->voice[ch].waveform << 5) | 
                            info->voice[ch].addr;
            INT8 sample = info->prom[prom_addr] - 8; // Convert to signed
            mix += sample * info->voice[ch].volume;
        }
        
        buffer[i] = mix * 256; // Scale to 16-bit
    }
}

// Write handlers
static void k005289_write(void* chip, UINT8 address, UINT16 data) {
    k005289_state* info = (k005289_state*)chip;
    int ch = address & 1; // Channel select
    
    switch (address) {
        // Control A (Channel 1)
        case 0x00:
        // Control B (Channel 2)
        case 0x01:
            info->voice[ch].volume = data & 0x0F;
            info->voice[ch].waveform = (data >> 5) & 0x07;
            break;

        // LD1/LD2 - Latch pitch
        case 0x02:
        case 0x03:
            info->voice[ch].pitch = 0xFFF - (data & 0x0FFF);
            break;

        // TG1/TG2 - Trigger frequency update
        case 0x04:
        case 0x05:
            info->voice[ch].freq = info->voice[ch].pitch;
            break;
    }
}

// Mute mask
static void k005289_set_mute_mask(void* chip, UINT32 mute_mask) {
    k005289_state* info = (k005289_state*)chip;
    info->mute_mask = mute_mask & 0x03;
}
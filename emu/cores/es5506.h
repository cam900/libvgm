#ifndef __ES5506_H__
#define __ES5506_H__

#include "../EmuStructs.h"

// cfg.flags: output channels (ES5505: 1..4, ES5506: 1..6, downmixed to stereo)

typedef struct es5506_config
{
	DEV_GEN_CFG _genCfg;
	
	UINT8 output;		// output channels (ES5505: 1..4, ES5506: 1..6, downmixed to stereo)
} ES5506_CFG;

extern const DEV_DECL sndDev_ES5506;

#endif	// __ES5506_H__

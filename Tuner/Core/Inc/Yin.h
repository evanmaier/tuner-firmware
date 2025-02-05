#ifndef Yin_h
#define Yin_h
#define __FPU_PRESENT  1U

#include "arm_math.h"

typedef struct _Yin {
	int16_t sampleRate;
	int16_t bufferSize;
	float32_t threshold;
	float32_t probability;
	float32_t* buffer;
	arm_rfft_fast_instance_f32 *fftInstance;
} Yin;

void Yin_init(Yin *yin, int16_t bufferSize, int16_t sampleRate, float32_t threshold);

float32_t Yin_getPitch(Yin *yin, float32_t* buffer);

#endif

#ifndef Yin_h
#define Yin_h
#define __FPU_PRESENT  1U

#include "arm_math.h"

typedef struct _Yin {
	int16_t sampleRate;
	int16_t bufferSize;
	float32_t* yinBuffer;
	float32_t threshold;
	uint16_t tauMax;
	uint16_t tauMin;
} Yin;

void Yin_init(Yin *yin, int16_t bufferSize, int16_t sampleRate, float32_t threshold, float32_t fMin, float32_t fMax);

float32_t Yin_getPitch(Yin *yin, float32_t* buffer);

#endif

#include <stdint.h> /* For standard interger types (int16_t) */
#include <stdlib.h> /* For call to malloc */
#include "Yin.h"

/* ------------------------------------------------------------------------------------------
--------------------------------------------------------------------------- PRIVATE FUNCTIONS
-------------------------------------------------------------------------------------------*/

/**
 * Step 1: Calculates the squared difference of the signal with a shifted version of itself
 * @param yin: Initialized Yin structure
 * @param buffer: Buffer of samples to process
 */
void Yin_difference(Yin *yin, float32_t* buffer){
	int16_t i;
	int16_t tau;
	float32_t delta;

	/* Calculate the difference for each tau from tauMin to tauMax */
	for(tau = yin->tauMin ; tau < yin->tauMax; tau++){

		/* Take the difference of the signal with a shifted version of itself, then square it */
		for(i = 0; i < yin->bufferSize - tau; i++){
			delta = buffer[i] - buffer[i + tau];
			yin->dfBuffer[tau] += delta * delta;
		}
	}
}


/**
 * Step 2: Calculate the normalized cumulative mean of the difference calculated in step 1
 * @param yin: Initialized Yin structure
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin){
	int16_t tau;
	float32_t runningSum = 0;

	/* Normalize the values in dfBuffer and store them in cmndfBuffer */
	for (tau = yin->tauMin; tau < yin->tauMax; tau++) {
		runningSum += yin->dfBuffer[tau];
		yin->cmndfBuffer[tau] = yin->dfBuffer[tau] * (float32_t)tau / runningSum;
	}
}

/**
 * Step 3: Search through the CMNDF buffer and find values that are over the threshold
 * @param yin: Initialized Yin structure
 * @return: Tau which caused the best approximate auto-corellation or -1 if no suitable value is found
 */
int16_t Yin_absoluteThreshold(Yin *yin){
	int16_t tau;

	for (tau = yin->tauMin; tau < yin->tauMax ; tau++) {
		if (yin->cmndfBuffer[tau] < yin->threshold) {
			while (tau + 1 < yin->tauMax && yin->cmndfBuffer[tau + 1] < yin->cmndfBuffer[tau]) {
				tau++;
			}
			return tau;
		}
	}

	return -1;
}

/**
 * Step 5: Interpolate the shift value (tau) to improve the pitch estimate.
 * @param yin: Initialized Yin structure
 * @param  tauEstimate: First Tau over threshold
 * @return: Refined Tau estimate
 */
float32_t Yin_parabolicInterpolation(Yin *yin, int16_t tauEstimate) {
	float32_t a = yin->cmndfBuffer[tauEstimate - 1];
	float32_t b = yin->cmndfBuffer[tauEstimate];
	float32_t c = yin->cmndfBuffer[tauEstimate + 1];
	float32_t alpha = a + c - 2.0f * b;
	float32_t beta = (c - a) / 2.0f;
	return tauEstimate - beta / (2.0f * alpha);
}

/* ------------------------------------------------------------------------------------------
---------------------------------------------------------------------------- PUBLIC FUNCTIONS
-------------------------------------------------------------------------------------------*/
void Yin_init(Yin *yin, int16_t bufferSize, int16_t sampleRate, float32_t threshold){
	/* Initialize the fields */
	yin->bufferSize = bufferSize;
	yin->threshold = threshold;
	yin->sampleRate = sampleRate;
	yin->tauMin = 10;
	yin->tauMax = bufferSize/2;

	/* Allocate the buffers */
	yin->dfBuffer = (float32_t *) malloc(sizeof(float32_t)* yin->tauMax);
	yin->cmndfBuffer = (float32_t *) malloc(sizeof(float32_t)* yin->tauMax);
}

/**
 * Runs the Yin pitch detection algortihm
 * @param  yin    Initialised Yin object
 * @param  buffer Buffer of samples to analyse
 * @return        Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float32_t Yin_getPitch(Yin *yin, float32_t* buffer){
	int16_t tauEstimate = -1;
	float32_t pitchInHertz = -1;
	int16_t i;

	/* Clear Buffers */
	for(i = 0; i < yin -> tauMax; i++) {
		yin->dfBuffer[i] = 0;
		yin->cmndfBuffer[i] = 1;
	}

	/* Step 1: Calculates the squared difference of the signal with a shifted version of itself. */
	Yin_difference(yin, buffer);

	/* Step 2: Calculate the normalized cumulative mean on the difference calculated in step 1 */
	Yin_cumulativeMeanNormalizedDifference(yin);

	/* Step 3: Search through the normalized cumulative mean array and find values that are over the threshold */
	tauEstimate = Yin_absoluteThreshold(yin);

	/* Step 5: Interpolate the shift value (tau) to improve the pitch estimate. */
	if(tauEstimate != -1){
		pitchInHertz = yin->sampleRate / Yin_parabolicInterpolation(yin, tauEstimate);
	}

	return pitchInHertz;
}

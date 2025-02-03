#include <stdint.h> /* For standard interger types (int16_t) */
#include <stdlib.h> /* For call to malloc */
#include "Yin.h"

/* ------------------------------------------------------------------------------------------
--------------------------------------------------------------------------- PRIVATE FUNCTIONS
-------------------------------------------------------------------------------------------*/

/**
 * Step 1: Calculates the squared difference of the signal with a shifted version of itself
 * @param yin	 Initialized Yin structure
 * @param yinBuffer Buffer of samples to process
 */
void Yin_difference(Yin *yin, float32_t* dataBuffer){
	// Power term calculation
	yin->powerTerms[0] = 0;
	for (int j = 0; j < yin->bufferSize; ++j) {
		yin->powerTerms[0] += dataBuffer[j] * dataBuffer[j];
	}

	for (int tau = 1; tau < yin->bufferSize; ++tau) {
		yin->powerTerms[tau] = yin->powerTerms[tau-1]
							 - dataBuffer[tau-1] * dataBuffer[tau-1]
							 + dataBuffer[tau + yin->bufferSize]
							 * dataBuffer[tau + yin->bufferSize];
	}

	// Convert to frequency domain
	arm_rfft_fast_f32(yin->fftInstance, dataBuffer, yin->fftBuffer, 0);

	// Create time-reversed version of signal
	for (int j = 0; j < yin->bufferSize; ++j) {
		yin->kernel[j] = dataBuffer[(yin->bufferSize - 1) - j];
	}

	arm_rfft_fast_f32(yin->fftInstance, yin->kernel, yin->kernel, 0);

	// Autocorrelation in frequency domain
	yin->acfBuffer[0] = yin->fftBuffer[0] * yin->kernel[0];
	yin->acfBuffer[1] = yin->fftBuffer[1] * yin->kernel[1];

	for (int k = 1; k < yin->bufferSize/2; k++) {
		int i = 2*k;

		float32_t realA = yin->fftBuffer[i];
		float32_t imagA = yin->fftBuffer[i+1];

		float32_t realB = yin->kernel[i];
		float32_t imagB = yin->kernel[i+1];

		yin->acfBuffer[i] = realA * realB - imagA * imagB;
		yin->acfBuffer[i+1] = imagA * realB + realA * imagB;
	}

	// Convert back to time domain
	arm_rfft_fast_f32(yin->fftInstance, yin->acfBuffer, yin->acfBuffer, 1);

	// Final difference function
	for (int j = 0; j < yin->tauMax; ++j) {
		yin->yinBuffer[j] = yin->powerTerms[0] + yin->powerTerms[j] - 2.0f
						  * yin->acfBuffer[yin->bufferSize - 1 + j];
	}
}


/**
 * Step 2: Calculate the normalized cumulative mean of the difference calculated in step 1
 * @param yin	Initialized Yin structure
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin){
	int16_t tau;
	float32_t runningSum = 0;

	/* Normalize the values using the sum of previous difference values */
	for (tau = yin->tauMin; tau < yin->tauMax; tau++) {

		runningSum += yin->yinBuffer[tau];

		if (runningSum > 0) {
			yin->yinBuffer[tau] = yin->yinBuffer[tau] * (float32_t)tau / runningSum;
		} else {
			yin->yinBuffer[tau] = 1;
		}
	}
}

/**
 * Step 3: Search through the CMNDF buffer and find values that are over the threshold
 * @param yin	Initialized Yin structure
 * @return 		Tau which caused the best approximate auto-corellation or -1 if no suitable value is found
 */
int16_t Yin_absoluteThreshold(Yin *yin){
	int16_t tau;

	for (tau = yin->tauMin; tau < yin->tauMax ; tau++) {
		if (yin->yinBuffer[tau] < yin->threshold) {
			while (tau + 1 < yin->tauMax && yin->yinBuffer[tau + 1] < yin->yinBuffer[tau]) {
				tau++;
			}
			yin->probability = 1 - yin->yinBuffer[tau];
			return tau;
		}
	}
	yin->probability = 0;
	return -1;
}

/**
 * Step 5: Interpolate the shift value (tau) to improve the pitch estimate.
 * @param yin		 	Initialized Yin structure
 * @param tauEstimate	First Tau over threshold
 * @return 				Refined Tau estimate
 */
float32_t Yin_parabolicInterpolation(Yin *yin, int16_t tauEstimate) {
	float32_t a = yin->yinBuffer[tauEstimate - 1];
	float32_t b = yin->yinBuffer[tauEstimate];
	float32_t c = yin->yinBuffer[tauEstimate + 1];
	float32_t alpha = a + c - 2.0f * b;
	float32_t beta = (c - a) / 2.0f;
	return tauEstimate - beta / (2.0f * alpha);
}

/* ------------------------------------------------------------------------------------------
---------------------------------------------------------------------------- PUBLIC FUNCTIONS
-------------------------------------------------------------------------------------------*/
/**
 * Initialize the Yin structure
 * @param yin    		Uninitialized Yin structure
 * @param bufferSize 	Size of data buffer
 * @param sampleRate 	ADC sample rate
 * @param threshold 	Minimum acceptable accuracy, YIN paper recommends 0.1 to 0.15
 */
void Yin_init(Yin *yin, int16_t bufferSize, int16_t sampleRate, float32_t threshold){
	yin->bufferSize = bufferSize;
	yin->threshold = threshold;
	yin->sampleRate = sampleRate;
	yin->tauMin = 10;
	yin->tauMax = bufferSize/2;
	yin->probability = 0.0f;

	yin->yinBuffer = (float32_t *) malloc(sizeof(float32_t)* yin->tauMax);
	yin->powerTerms = (float32_t *) malloc(sizeof(float32_t)* bufferSize);
	yin->fftBuffer = (float32_t *) malloc(sizeof(float32_t)* bufferSize);
	yin->acfBuffer = (float32_t *) malloc(sizeof(float32_t)* bufferSize);
	yin->kernel = (float32_t *) malloc(sizeof(float32_t)* bufferSize);

	yin->fftInstance = (arm_rfft_fast_instance_f32 *)malloc(sizeof(arm_rfft_fast_instance_f32));
	arm_rfft_fast_init_f32(yin->fftInstance, bufferSize);
}

/**
 * Runs the Yin pitch detection algorithm
 * @param yin    Initialized Yin object
 * @param buffer Buffer of samples to analyze
 * @return       Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float32_t Yin_getPitch(Yin *yin, float32_t* buffer){
	int16_t tauEstimate = -1;
	float32_t pitchInHertz = -1;
	int16_t i;

	/* Clear Buffer */
	for(i = 0; i < yin -> tauMax; i++) {
		yin->yinBuffer[i] = 0;
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

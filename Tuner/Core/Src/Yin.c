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
void Yin_difference(Yin *yin, float32_t* dataBuffer) {
	// Misc definitions
	size_t N = yin->bufferSize;
	int16_t i;
	float32_t a,b,c,d;

	// Define Buffers
	float32_t powerTerms[N/2];
	float32_t data[N];
	float32_t reversedData[N];
	float32_t dataFFT[N];
	float32_t reversedDataFFT[N];
	float32_t convolutionResult[N];
	float32_t acfBuffer[N];

	// Copy dataBuffer into data to prevent RFFT from modifying it
	memcpy(data, dataBuffer, N*sizeof(float32_t));

    // Compute the first power term
    powerTerms[0] = 0;
    for (i = 0; i < N/2; i++) {
        powerTerms[0] += dataBuffer[i] * dataBuffer[i];
    }

    // Compute all other power terms iteratively
    for (i = 1; i < N/2; i++) {
        powerTerms[i] = powerTerms[i - 1]
                        - dataBuffer[i - 1] * dataBuffer[i - 1]
                        + dataBuffer[i + N/2 - 1] * dataBuffer[i + N/2 - 1];
    }

    // Forward RFFT of input signal
    arm_rfft_fast_f32(yin->fftInstance, data, dataFFT, 0);

    // Prepare the reversed half copy of input data for convolution
    for (i = 0; i < N/2; i++) {
    	// Reverse the first N/2 data samples
        reversedData[i] = dataBuffer[(N/2 - 1) - i];
        // Zero pad the last N/2 samples because max shift is N/2
        reversedData[i + N/2] = 0.0f;
    }

    // Forward RFFT of reversed input signal
    arm_rfft_fast_f32(yin->fftInstance, reversedData, reversedDataFFT, 0);

    // multiplication in frequency domain
    for (i = 1; i < N/2; i++) {
    	// RFFT output is [real_0, real_N/2, real_1, imag_1, real_2, imag_2, ...]
        a = dataFFT[2*i];
        b = dataFFT[2*i+1];
        c = reversedDataFFT[2*i];
        d = reversedDataFFT[2*i+1];

        // (a + bi)*(c + di) = (ac - bd) + (ad + bc)i
        convolutionResult[2*i] = a*c - b*d;
        convolutionResult[2*i+1] = a*d + b*c;
    }
    // handle 0th and N/2th edge cases
    convolutionResult[0] = dataFFT[0] * reversedDataFFT[0];
    convolutionResult[1] = dataFFT[1] * reversedDataFFT[1];

    // Inverse FFT to get autocorrelation in time domain
    arm_rfft_fast_f32(yin->fftInstance, convolutionResult, acfBuffer, 1);

    // Compute final difference function
    for (i = 0; i < N/2; i++) {
    	// undo reverse in place
        yin->buffer[i] = powerTerms[0] + powerTerms[i] - 2.0f * acfBuffer[N/2 + i];
    }
}




/**
 * Step 2: Calculate the normalized cumulative mean of the difference calculated in step 1
 * @param yin	Initialized Yin structure
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin){
	int16_t tau;
	float32_t runningSum = 0;
	yin->buffer[0] = 1;

	/* Normalize the values using the sum of previous difference values */
	for (tau = 1; tau < yin->bufferSize/2; tau++) {
		runningSum += yin->buffer[tau];
		yin->buffer[tau] *= (float32_t)tau / runningSum;
	}
}

/**
 * Step 3: Search through the CMNDF buffer and find values that are over the threshold
 * @param yin	Initialized Yin structure
 * @return 		Tau which caused the best approximate auto-corellation or -1 if no suitable value is found
 */
int16_t Yin_absoluteThreshold(Yin *yin){
	int16_t tau;

	for (tau = 2; tau < yin->bufferSize/2 ; tau++) {
		if (yin->buffer[tau] < yin->threshold) {
			while (tau + 1 < yin->bufferSize/2 && yin->buffer[tau + 1] < yin->buffer[tau]) {
				tau++;
			}
			yin->probability = 1 - yin->buffer[tau];
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
	float32_t a = yin->buffer[tauEstimate - 1];
	float32_t b = yin->buffer[tauEstimate];
	float32_t c = yin->buffer[tauEstimate + 1];
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

	yin->probability = 0.0f;

	yin->buffer = (float32_t *) malloc(sizeof(float32_t)* bufferSize/2);

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

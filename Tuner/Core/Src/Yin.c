#include <stdint.h> /* For standard interger types (int16_t) */
#include <stdlib.h> /* For call to malloc */
#include "Yin.h"

/* ------------------------------------------------------------------------------------------
--------------------------------------------------------------------------- PRIVATE FUNCTIONS
-------------------------------------------------------------------------------------------*/

/**
 * Step 1: Calculates the squared difference of the signal with a shifted version of itself.
 * @param buffer Buffer of samples to process.
 *
 * This is the Yin algorithms tweak on autocorellation. Read http://audition.ens.fr/adc/pdf/2002_JASA_YIN.pdf
 * for more details on what is in here and why it's done this way.
 */
void Yin_difference(Yin *yin, float32_t* buffer){
	int16_t i;
	int16_t tau;
	float32_t delta;

	/* Calculate the difference for difference shift values (tau) for the half of the samples */
	for(tau = yin->tauMin ; tau < yin->tauMax; tau++){

		/* Take the difference of the signal with a shifted version of itself, then square it.
		 * (This is the Yin algorithm's tweak on autocorellation) */
		for(i = 0; i < yin->bufferSize - tau; i++){
			delta = buffer[i] - buffer[i + tau];
			yin->yinBuffer[tau] += delta * delta;
		}
	}
}


/**
 * Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1
 * @param yin #Yin structure with information about the signal
 *
 * This goes through the Yin autocorellation values and finds out roughly where shift is which
 * produced the smallest difference
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin){
	int16_t tau;
	float32_t runningSum = 0;

	/* Sum all the values in the autocorellation buffer and nomalise the result, replacing
	 * the value in the autocorellation buffer with a cumulative mean of the normalised difference */
	for (tau = yin->tauMin; tau < yin->tauMax; tau++) {
		runningSum += yin->yinBuffer[tau];
		yin->yinBuffer[tau] *= (float32_t)tau / runningSum;
	}
}

/**
 * Step 3: Search through the normalised cumulative mean array and find values that are over the threshold
 * @return Shift (tau) which caused the best approximate autocorellation. -1 if no suitable value is found over the threshold.
 */
int16_t Yin_absoluteThreshold(Yin *yin){
	int16_t tau;

	for (tau = yin->tauMin; tau < yin->tauMax ; tau++) {
		if (yin->yinBuffer[tau] < yin->threshold) {
			while (tau + 1 < yin->tauMax && yin->yinBuffer[tau + 1] < yin->yinBuffer[tau]) {
				tau++;
			}
			return tau;
		}
	}

	return -1;
}

/**
 * Step 5: Interpolate the shift value (tau) to improve the pitch estimate.
 * @param  yin         [description]
 * @param  tauEstimate [description]
 * @return             [description]
 *
 * The 'best' shift value for autocorellation is most likely not an interger shift of the signal.
 * As we only autocorellated using integer shifts we should check that there isn't a better fractional
 * shift value.
 */
float32_t Yin_parabolicInterpolation(Yin *yin, int16_t tauEstimate) {
	float32_t a = yin->yinBuffer[tauEstimate - 1];
	float32_t b = yin->yinBuffer[tauEstimate];
	float32_t c = yin->yinBuffer[tauEstimate + 1];
	return tauEstimate - (c - a) / (2.0f * (a + c - 2.0f * b));
}

/* ------------------------------------------------------------------------------------------
---------------------------------------------------------------------------- PUBLIC FUNCTIONS
-------------------------------------------------------------------------------------------*/
void Yin_init(Yin *yin, int16_t bufferSize, int16_t sampleRate, float32_t threshold, float32_t fMin, float32_t fMax){
	/* Initialise the fields of the Yin structure passed in */
	yin->bufferSize = bufferSize;
	yin->threshold = threshold;
	yin->sampleRate = sampleRate;
	yin->tauMin = sampleRate / fMax;
	yin->tauMax = sampleRate / fMin;

	/* Allocate the autocorellation buffer and initialise it to zero */
	yin->yinBuffer = (float32_t *) malloc(sizeof(float32_t)* bufferSize);

	int16_t i;
	for(i = 0; i < bufferSize; i++){
		yin->yinBuffer[i] = 0;
	}
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

	/* Step 1: Calculates the squared difference of the signal with a shifted version of itself. */
	Yin_difference(yin, buffer);

	/* Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1 */
	Yin_cumulativeMeanNormalizedDifference(yin);

	/* Step 3: Search through the normalised cumulative mean array and find values that are over the threshold */
	tauEstimate = Yin_absoluteThreshold(yin);

	/* Step 5: Interpolate the shift value (tau) to improve the pitch estimate. */
	if(tauEstimate != -1){
		pitchInHertz = yin->sampleRate / Yin_parabolicInterpolation(yin, tauEstimate);
	}

	return pitchInHertz;
}

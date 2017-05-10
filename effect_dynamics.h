/* Audio Library for Teensy 3.X
 * Dynamics Processor (Gate, Compressor & Limiter)
 * Copyright (c) 2017, Marc Paquette (marc@dacsystemes.com)
 * Based on analyse_rms & mixer objects by Paul Stoffregen
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef effect_dynamics_h_
#define effect_dynamics_h_

#include "Arduino.h"
#include "AudioStream.h"

class AudioEffectDynamics : public AudioStream
{
public:
	AudioEffectDynamics(void) : AudioStream(1, inputQueueArray){
    gateThreshold = -60.0f;
    gateReleaseTime = 0.02f;    

    compThreshold = -35.0f;
    compAttackTime = 0.005f;
    compReleaseTime = 0.2f;
    compRatio = 45.0f;
    compKneeWidth = 6.0f;
    compAutoMakeupGain = true;
    compMakeupGain = 12.0f;    

    limitThreshold = -6.0f;
    limitAttackTime = 0.005f;
    limitReleaseTime = 0.01f;    
    
    computeAlphas();
	}

	//Sets the gate parameters.
	//threshold is in dbFS
	//release is in seconds
	void gate(float threshold = -60.0f, float release = 0.02f)
	{
		gateThreshold = threshold;
		gateReleaseTime = release;
		computeAlphas();
	}

	//Sets the compression parameters.
	//threshold, kneeWidth & makeupGain are in db(FS)
	//attack and release are in seconds
	//ratio is expressed as x:1 i.e. 1 for no compression, 60 for brickwall limiting
	//Set kneeWidth to 0 for hard knee
	//If autoMakeupGain is true, makeup will be computed to leave a 6db margin under the limiter threshold.
	void compression(float threshold = -35.0f, float attack = 0.005f, float release = 0.2f, float ratio = 45.0f, float kneeWidth = 6.0f, bool autoMakeupGain = true, float makeupGain = 12.0f)
	{
		compThreshold = threshold;
		compAttackTime = attack;
		compReleaseTime = release;    
		compRatio = ratio;
		compKneeWidth = kneeWidth;
		compMakeupGain = makeupGain;
		compAutoMakeupGain = autoMakeupGain;
		computeAlphas();
	}

	//Sets the hard limiter parameters
	//threshold is in dbFS
	//attack & release are in seconds
	void limit(float threshold = -6.0f, float attack = 0.005f, float release = 0.01f)
	{
		limitThreshold = threshold;
		limitAttackTime = attack;
		limitReleaseTime = release;    
		computeAlphas();
	}
  
	virtual void update(void);

  
private:
	audio_block_t *inputQueueArray[1];

	float gateThreshold;
	float gateReleaseTime;
	float gateThresholdClose;

	float compThreshold;
	float compAttackTime;
	float compReleaseTime;
	float compRatio;
	float compKneeWidth;
	float compMakeupGain;
	bool compAutoMakeupGain;

	float limitThreshold;
	float limitAttackTime;
	float limitReleaseTime;

	float gateGain;
	float compdb;
	float limitdb;
	float gain;

	float aGateRelease;
	float aHalfKneeWidth;
	float aTwoKneeWidth;
	float aKneeRatio;
	float aCompAttack;
	float aOneMinusCompAttack;
	float aCompRelease;
	float aOneMinusCompRelease;
	float aLimitAttack;
	float aOneMinusLimitAttack;
	float aLimitRelease;

	//Computes all constants & alphas
	void computeAlphas(void) {
		aGateRelease = timeToAlpha(gateReleaseTime);
		gateThresholdClose = gateThreshold - 6.0f;

		aCompAttack = timeToAlpha(compAttackTime);
		aOneMinusCompAttack = 1.0f - aCompAttack;
		aCompRelease = timeToAlpha(compReleaseTime);
		aOneMinusCompRelease = 1.0f - aCompRelease;
		aHalfKneeWidth = compKneeWidth / 2.0f;
		aTwoKneeWidth = compKneeWidth * 2.0f;
		aKneeRatio = (1.0f / compRatio) - 1.0f;

		aLimitAttack = timeToAlpha(limitAttackTime);
		aOneMinusLimitAttack = 1.0f - aLimitAttack;
		aLimitRelease = timeToAlpha(limitReleaseTime);

		if (compAutoMakeupGain) {
			compMakeupGain = -(compThreshold - (compThreshold / compRatio)) + limitThreshold - 6.0f;
		}
	}

	//Computes smoothing time constants for a 10% to 90% change
	float timeToAlpha(float time) {
		return expf(-0.9542f / (((float)AUDIO_SAMPLE_RATE_EXACT / (float)AUDIO_BLOCK_SAMPLES) * time));
	}
};

#endif

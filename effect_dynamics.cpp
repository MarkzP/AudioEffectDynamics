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

#include "effect_dynamics.h"
#include "utility/dspinst.h"


void AudioEffectDynamics::update(void) {
	audio_block_t *block;
	
	block = receiveWritable(0);
	
	if (!block) {
    gain = 1.0f;
		return;
	}

	uint32_t *p = (uint32_t *)block->data;
	const uint32_t *end = p + AUDIO_BLOCK_SAMPLES / 2;
	int32_t mult = gain * 65536.0f;
	int64_t sum = 0;
	uint32_t n1, n2, n3, n4;
	int32_t v1b, v1t, v2b, v2t, v3b, v3t, v4b, v4t;
  
	do {
		//Grab 8 sample
		n1 = *p;
		n2 = *(p + 1);
		n3 = *(p + 2);
		n4 = *(p + 3);

		//Compute square sum of all 8 samples
		sum = multiply_accumulate_16tx16t_add_16bx16b(sum, n1, n1);
		sum = multiply_accumulate_16tx16t_add_16bx16b(sum, n2, n2);
		sum = multiply_accumulate_16tx16t_add_16bx16b(sum, n3, n3);
		sum = multiply_accumulate_16tx16t_add_16bx16b(sum, n4, n4);

		//Apply gain to all 8 samples
		v1b = signed_multiply_32x16b(mult, n1);
		v1t = signed_multiply_32x16t(mult, n1);
		v2b = signed_multiply_32x16b(mult, n2);
		v2t = signed_multiply_32x16t(mult, n2);
		v3b = signed_multiply_32x16b(mult, n3);
		v3t = signed_multiply_32x16t(mult, n3);
		v4b = signed_multiply_32x16b(mult, n4);
		v4t = signed_multiply_32x16t(mult, n4);

		v1b = signed_saturate_rshift(v1b, 16, 0);
		v1t = signed_saturate_rshift(v1t, 16, 0);
		v2b = signed_saturate_rshift(v2b, 16, 0);
		v2t = signed_saturate_rshift(v2t, 16, 0);
		v3b = signed_saturate_rshift(v3b, 16, 0);
		v3t = signed_saturate_rshift(v3t, 16, 0);
		v4b = signed_saturate_rshift(v4b, 16, 0);
		v4t = signed_saturate_rshift(v4t, 16, 0);

		//Pack & save samples
		*p++ = pack_16b_16b(v1t, v1b);
		*p++ = pack_16b_16b(v2t, v2b);
		*p++ = pack_16b_16b(v3t, v3b);
		*p++ = pack_16b_16b(v4t, v4b);
	} while (p < end);

	//Compute block RMS level in Db
	float meansq = (float)sum / (float)AUDIO_BLOCK_SAMPLES;
	float inputdb = 20.0f * log10f(sqrtf(meansq) / 32767.0f);

	//Gate
	if (inputdb >= gateThreshold) gateGain = 1.0f;
	else if (inputdb < gateThresholdClose) gateGain *= aGateRelease;

	//Compressor
	float attdb = 0; //Below knee
	if (inputdb >= compThreshold - aHalfKneeWidth && inputdb < compThreshold + aHalfKneeWidth) {
    //Knee transition
    float knee = inputdb - compThreshold + aHalfKneeWidth;
		attdb = (inputdb + ((aKneeRatio * knee * knee) / aTwoKneeWidth)) - inputdb;
	}
	else {
	  //Above knee
	  attdb = compThreshold + ((inputdb - compThreshold) / compRatio) - inputdb;
	}

	if (attdb <= compdb) compdb = (aCompAttack * compdb) + (aOneMinusCompAttack * attdb);
	else compdb = (aCompRelease * compdb) + (aOneMinusCompRelease * attdb);

	//Brickwall Limiter
	if (limitThreshold < 0.0f) {
		float outdb = inputdb + compdb + compMakeupGain;
		if (outdb >= limitThreshold) limitdb = (aLimitAttack * limitdb) + (aOneMinusLimitAttack * (limitThreshold - outdb));
		else limitdb *= aLimitRelease;
	}
	else limitdb = 0.0f;
  
  
	//Compute linear gain for next block
	gain = gateGain * powf(10.0f, (compdb + compMakeupGain + limitdb) / 20.0f);

	//Transmit & release
	transmit(block);
	release(block);
}



/*
 * biquad_coefficient_utils.h
 *
 *  Created on: 17 Jul 2019
 *      Author: timfernandez
 */

#ifndef MAIN_BIQUAD_COEFFICIENT_UTILS_H_
#define MAIN_BIQUAD_COEFFICIENT_UTILS_H_
#define PI 3.1415926535897932384

#include <math.h>


enum filter {
	low_pass,
	high_pass,
	band_pass,

};

int f_s = 48000;

float * calculate_coefficients(char filter, float gain, float Q, float cutFreq){

	static float coefficient_array[5] = {0,0,0,0,0};

	switch(filter){
	case low_pass:{
		float theta_c = 2*PI*(cutFreq/f_s);
		float K = tan(theta_c/2);
		float W = K*K;
		float alpha = 1 + (K/Q) + W;
		float linearGain = pow(10, (gain/20));

//		float a1 = 2*(W-1)/alpha;
//		float a2 = (1 - (K/Q) + W)/alpha;
//		float b0 = W/alpha;
//		float b1 = 2*W/alpha;
//		float b2 = b0;

		float a1 = 2*(W-1)/alpha;
		float a2 = (1 - (K/Q) + W)/alpha;
		float b0 = (W/alpha)*linearGain/2;
		float b1 = (2*W/alpha)*linearGain;
		float b2 = b0;

		coefficient_array[0] = b0;
		coefficient_array[1] = b1;
		coefficient_array[2] = b2;
		coefficient_array[3] = -a1;
		coefficient_array[4] = -a2;


		break;
		}//end of low_pass
	default :
		printf("Filter type not recognised.\n");
		break;
	}
	printf("coefficient_array[0] = %f\n", coefficient_array[0]);
	printf("coefficient_array[1] = %f\n", coefficient_array[1]);
	printf("coefficient_array[2] = %f\n", coefficient_array[2]);
	printf("coefficient_array[3] = %f\n", coefficient_array[3]);
	printf("coefficient_array[4] = %f\n", coefficient_array[4]);

	return coefficient_array;
}
#endif /* MAIN_BIQUAD_COEFFICIENT_UTILS_H_ */

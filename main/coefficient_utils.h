/*
 * coefficient_utils.h
 *
 *  Created on: 17 Jul 2019
 *      Author: timfernandez
 */

#include <math.h>
#include "SigmaStudioFW.h"

#define PI 3.14159265358979323846

#ifndef MAIN_COEFFICIENT_UTILS_H_
#define MAIN_COEFFICIENT_UTILS_H_

float * calculate_coefficients(int filter, float gain, float Q, float cutfreq);
void update_biquad_coefficients(float gain, float Q, float cutfreq);

enum filter {
	low_pass,
	high_pass,
	all_pass,
	notch,
	chebyshev_low,
	chebyshev_high,
};


static const int sampling_frequency = 48000;


float * calculate_coefficients(int filter, float gain, float Q, float cutfreq){
	//Move this into the case low_pass if doesn't work
	double theta_c = 2 * PI * (cutfreq / sampling_frequency);
	double K = tan(theta_c/2);
	double W = K*K;
	double alpha = 1 + (K/Q) + W;

	switch(filter){
		case low_pass:
		{

		//	float a0 = 1;
			float a1 = -(2*(W-1)/alpha);
			float a2 = -((1- (K/Q)+W)/alpha);
			float b0 = W/alpha;
			float b1 = 2*(W/alpha);  //Could possibly shorten this to b1 = 2*b0; ????
			float b2 = b0;

			static float coefficients_as_floats[5];

			coefficients_as_floats[0] = b0;
			coefficients_as_floats[1] = b1;
			coefficients_as_floats[2] = b2;
			coefficients_as_floats[3] = a1;
			coefficients_as_floats[4] = a2;
printf("Q: %f  |   Cut-off: %f\n", Q, cutfreq);
printf("Low_pass coefficients  b0: %f  |  b1: %f  |  b2: %f  |  a1: %f  |  a2: %f\n",
		coefficients_as_floats[0], coefficients_as_floats[1], coefficients_as_floats[2], coefficients_as_floats[3], coefficients_as_floats[4]);

			return coefficients_as_floats;
		} case high_pass:
		{
			float a1 = -(2*(W-1)/alpha);
			float a2 = -((1- (K/Q)+W)/alpha);
			float b0 = 1/alpha;
			float b1 = -2/alpha;
			float b2 = b0;

			static float coefficients_as_floats[5];

			coefficients_as_floats[0] = b0;
			coefficients_as_floats[1] = b1;
			coefficients_as_floats[2] = b2;
			coefficients_as_floats[3] = a1;
			coefficients_as_floats[4] = a2;
printf("Q: %f  |   Cut-off: %f\n", Q, cutfreq);
printf("High_pass coefficients    b0: %f  |  b1: %f  |  b2: %f  |  a1: %f  |  a2: %f\n",
		coefficients_as_floats[0], coefficients_as_floats[1], coefficients_as_floats[2], coefficients_as_floats[3], coefficients_as_floats[4]);

			return coefficients_as_floats;

		}
		default:
		{
			printf("filter not recognised.\n");
			return NULL;
		}
	}
}

//float * Q_adjust(int order){
//	static float Qs_as_floats[order];
//
//
//}




#endif /* MAIN_COEFFICIENT_UTILS_H_ */

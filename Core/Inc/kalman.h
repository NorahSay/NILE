/*
 * kalman.h
 *
 *  Created on: Jun 7, 2025
 *      Author: norah
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

typedef struct {
    float q;     // Process noise covariance
    float r;     // Measurement noise covariance
    float x;     // Estimated value
    float p;     // Estimation error covariance
    float k;     // Kalman gain
} kalman_filter_t;

#endif /* INC_KALMAN_H_ */

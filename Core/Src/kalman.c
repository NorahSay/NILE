#include "kalman.h"

// Initialize the Kalman filter
void kalman_init(kalman_filter_t *kf, float q, float r, float initial_value)
{
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1.0f; // Can be set to another initial uncertainty
    kf->k = 0.0f;
}

// Update with a new measurement
float kalman_update(kalman_filter_t *kf, float measurement)
{
    // Predict
    kf->p += kf->q;

    // Update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1.0f - kf->k);

    return kf->x;
}



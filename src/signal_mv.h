#ifndef SIGNAL_MV_H
#define SIGNAL_MV_H

#include <stdint.h>

/** Convert mean SAADC counts (12-bit path) to signal uV at the electrode front-end. */
int32_t signal_uv_from_saadc_mean(int32_t mean_counts);

#endif /* SIGNAL_MV_H */

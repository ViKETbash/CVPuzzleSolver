#pragma once

#include <libimages/image.h>


// returns mask that has 0 if < threshold, 255 otherwise
image8u threshold_masking(const image32f &image, float threshold);

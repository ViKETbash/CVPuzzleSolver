#include "threshold_masking.h"

#include <libbase/runtime_assert.h>


image8u threshold_masking(const image32f &image, float threshold) {
    rassert(image.channels() == 1, 2321431421, image.channels());
    image8u mask(image.size());
    for (int j = 0; j < image.height(); ++j) {
        for (int i = 0; i < image.width(); ++i) {
            mask(j, i) = (image(j, i) < threshold) ? 0 : 255;
        }
    }
    return mask;
}

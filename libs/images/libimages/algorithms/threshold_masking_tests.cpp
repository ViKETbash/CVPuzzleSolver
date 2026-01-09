#include "threshold_masking.h"

#include <gtest/gtest.h>

#include <libbase/configure_working_directory.h>
#include <libbase/runtime_assert.h>
#include <libimages/algorithms/grayscale.h>
#include <libimages/debug_io.h>
#include <libimages/image_io.h>
#include <libimages/tests_utils.h>

TEST(threshold_masking, thresholdByConstant100) {
    configureWorkingDirectory();

    image8u img = load_image("data/00_photo_six_parts_downscaled_x4.jpg");
    image32f grayscale = to_grayscale_float(img);
    image8u is_foreground_mask = threshold_masking(grayscale, 100);
    debug_io::dump_image(getUnitCaseDebugDir() + "is_foreground_by_100.jpg", is_foreground_mask);
}

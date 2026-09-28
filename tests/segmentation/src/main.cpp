#include "../../common/tests_access.hpp"
#include "thermal_Camera.h"
#include <zephyr/ztest.h>

// ZTEST_SUITE(SUITE_NAME, PREDICATE, setup_fn, before_fn, after_fn, teardown_fn)
ZTEST_SUITE(segmentation, NULL, NULL, NULL, NULL, NULL);



ZTEST_F(segmentation, test_uniform_frame)
{


    ThermalFrame uniFrame{};
    fill(uniFrame.pixels,50.0f);

    float outputBuf[CAMERA_PROCESSING_SEGMENTS];

    testAccess::segment(uniFrame, outputBuf);
    // ASSERT: all seven segment averages should be 50.
    zassert_equal(result, 0);

    for (float temperature : outputBuf) {
        zassert_within(temperature, 50.0f, 0.001f);
    }
}
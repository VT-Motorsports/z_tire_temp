
#pragma once

#include "thermal_Camera.h"
#include "thermal_pipeline.h"
#include <cstdint>
class testAccess
{

  public:
    // static int ThermalPipeline::segementCameraData(ThermalFrame &frame, float
    // (&buf)[CAMERA_PROCESSING_SEGMENTS],uint8_t seg_height)

    static int segment(ThermalFrame &frame, float (&output)[CAMERA_PROCESSING_SEGMENTS], uint8_t height)
    {
      return ThermalPipeline::segmentCameraData(frame, output, height);
    }

    static int segment(ThermalFrame &frame, float (&output)[CAMERA_PROCESSING_SEGMENTS])
    {
      return ThermalPipeline::segmentCameraData(frame, output);
    }

};
#ifndef TENSORRT_LIGHTNET__TENSORRT_LIGHTNET_HPP_
#define TENSORRT_LIGHTNET__TENSORRT_LIGHTNET_HPP_

#include "class_detector.h"
#include "yolo_config_parser.h"

#include <opencv2/opencv.hpp>

#include <vector>

namespace tensorrt_lightnet
{
    class TrtLightNet
    {
    public:
        TrtLightNet(const std::string &model_cfg, const std::string &model_weights);

        bool doInference(std::vector<cv::Mat> images);

        std::unique_ptr<Detector> detector_;
        const bool cuda = false;
    };

} // namespace tensorrt_lightnet

#endif // TENSORRT_LIGHTNET__TENSORRT_LIGHTNET_HPP_
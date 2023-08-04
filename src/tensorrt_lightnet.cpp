#include "tensorrt_lightnet.hpp"

namespace tensorrt_lightnet
{
    TrtLightNet::TrtLightNet(const std::string &model_cfg, const std::string &model_weights)
    {
        const bool cuda = get_cuda_flg();

        // Initialize a detector
        ::Config config;

        config.net_type = YOLOV4;
        config.file_model_cfg = model_cfg;
        config.file_model_weights = model_weights;
        config.inference_precison = FP32;
        config.batch = 1;
        config.width = 1280;
        config.height = 960;
        config.dla = -1;
        detector_ = std::make_unique<::Detector>();
        detector_->init(config);
    }

    bool TrtLightNet::doInference(std::vector<cv::Mat> images)
    {
        std::vector<BatchResult> batch_res;

        detector_->detect(images, batch_res, cuda);
        detector_->segment(images, "");

        for (int i = 0; i < images.size(); i++)
        {
            for (const auto &r : batch_res[i])
            {
                detector_->draw_BBox(images[i], r);
            }
        }

        return true;
    }

} // namespace tensorrt_lightnet
#include "tensorrt_lightnet.hpp"

namespace tensorrt_lightnet
{
    TrtLightNet::TrtLightNet()
    {
        // Initialize a detector
        ::Config config;

        // TODO: パスを引数として渡す
        config.net_type = YOLOV4;
        config.file_model_cfg = "/home/tishizuka/Workspace/edge-auto-jetson/src/tensorrt_lightnet/lightNet-TRT/configs/lightNet-BDD100K-det-semaseg-1280x960.cfg";
        config.file_model_weights = "/home/tishizuka/Workspace/edge-auto-jetson/src/tensorrt_lightnet/lightNet-TRT/configs/lightNet-BDD100K-det-semaseg-1280x960.weights";
        config.inference_precison = FP32;
        config.batch = 1;
        config.width = 1280;
        config.height = 960;
        config.dla = -1;
        detector_ = std::make_unique<::Detector>();
        detector_->init(config);
    }

    bool TrtLightNet::doInference(const std::vector<cv::Mat> &images)
    {
        detector_->segment(images);
        return true;
    }

} // namespace tensorrt_lightnet
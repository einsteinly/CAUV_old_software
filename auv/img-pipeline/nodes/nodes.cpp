#include "../nodeFactory.h"

#include "copyNode.h"
#include "fileInputNode.h"
#include "fileOutputNode.h"
#include "resizeNode.h"
#include "localDisplayNode.h"
#include "cameraInputNode.h"
#include "netInputNode.h"
#include "houghLinesNode.h"
#include "cannyNode.h"
#include "convertColourNode.h"
#include "guiOutputNode.h"
#include "houghCirclesNode.h"
#include "gaussianBlurNode.h"
#include "medianFilterNode.h"
#include "bilateralFilterNode.h"
#include "splitRGBNode.h"
#include "combineRGBNode.h"
#include "splitHSVNode.h"
#include "splitYUVNode.h"
#include "combineYUVNode.h"
#include "combineHSVNode.h"
#include "levelsNode.h"
#include "mixNode.h"
#include "percentileNode.h"
#include "sonarInputNode.h"
#include "broadcastImageNode.h"
#include "videoFileOutputNode.h"
#include "invertNode.h"
#include "kmeansNode.h"
#include "mixValueNode.h"
#include "cropNode.h"
#include "grabcutNode.h"
#include "histogramNode.h"
#include "histogramSegmentationNode.h"
#include "centreFinderNode.h"
#include "quickSegmentNode.h"
#include "thresholdMaskNode.h"
#include "cornerHarrisNode.h"
#include "histogramSegmentationArbNode.h"
#include "runningAverageNode.h"
#include "copyNodeMask.h"
#include "fastMedianNode.h"
#include "fastCornersNode.h"
#include "drawCornersNode.h"
#include "broadcastCornersNode.h"
#include "shiTomasiCornersNode.h"
#include "pyramidNode.h"
#include "drawLinesNode.h"
#include "broadcastLinesNode.h"
#include "nullParamNode.h"
#include "drawHistogramNode.h"
#include "valueInputNode.h"
#include "broadcastHistogramNode.h"
#include "nopNode.h"
#include "throttleNode.h"
#include "delayNode.h"
#include "stitchNode.h"
#include "recogniserNode.h"
#include "surfCornersNode.h"
#include "clampNode.h"
#include "broadcastKeypointsNode.h"

namespace cauv{
namespace imgproc{

// Register node types (actually definitions of static data members)
DEFINE_NFR(CopyNode, NodeType::Copy);
DEFINE_NFR(FileInputNode, NodeType::FileInput);
DEFINE_NFR(FileOutputNode, NodeType::FileOutput);
DEFINE_NFR(ResizeNode, NodeType::Resize);
DEFINE_NFR(LocalDisplayNode, NodeType::LocalDisplay);
DEFINE_NFR(CameraInputNode, NodeType::CameraInput);
DEFINE_NFR(NetInputNode, NodeType::NetInput);
DEFINE_NFR(HoughLinesNode, NodeType::HoughLines);
DEFINE_NFR(CannyNode, NodeType::Canny);
DEFINE_NFR(ConvertColourNode, NodeType::ConvertColour);
DEFINE_NFR(GuiOutputNode, NodeType::GuiOutput);
DEFINE_NFR(HoughCirclesNode, NodeType::HoughCircles);
DEFINE_NFR(GaussianBlurNode, NodeType::GaussianBlur);
DEFINE_NFR(MedianFilterNode, NodeType::MedianFilter);
DEFINE_NFR(BilateralFilterNode, NodeType::BilateralFilter);
DEFINE_NFR(SplitRGBNode, NodeType::SplitRGB);
DEFINE_NFR(CombineRGBNode, NodeType::CombineRGB);
DEFINE_NFR(SplitHSVNode, NodeType::SplitHSV);
DEFINE_NFR(SplitYUVNode, NodeType::SplitYUV);
DEFINE_NFR(CombineYUVNode, NodeType::CombineYUV);
DEFINE_NFR(CombineHSVNode, NodeType::CombineHSV);
DEFINE_NFR(LevelsNode, NodeType::Levels);
DEFINE_NFR(MixNode, NodeType::Mix);
DEFINE_NFR(PercentileNode, NodeType::Percentile);
DEFINE_NFR(SonarInputNode, NodeType::SonarInput);
DEFINE_NFR(BroadcastImageNode, NodeType::BroadcastImage);
DEFINE_NFR(VideoFileOutputNode, NodeType::VideoFileOutput);
DEFINE_NFR(InvertNode, NodeType::Invert);
DEFINE_NFR(KMeansNode, NodeType::KMeans);
DEFINE_NFR(MixValueNode, NodeType::MixValue);
DEFINE_NFR(CropNode, NodeType::Crop);
DEFINE_NFR(GrabCutNode, NodeType::GrabCut);
DEFINE_NFR(HistogramNode, NodeType::Histogram);
DEFINE_NFR(HistogramSegmentationNode, NodeType::HistogramSegmentation);
DEFINE_NFR(CentreFinderNode, NodeType::Centre);
DEFINE_NFR(QuickSegmentNode, NodeType::QuickSegment);
DEFINE_NFR(ThresholdMaskNode, NodeType::ThresholdMask);
DEFINE_NFR(CornerHarrisNode, NodeType::CornerHarris);
DEFINE_NFR(HistogramSegmentationArbNode, NodeType::HistogramSegmentationArb);
DEFINE_NFR(RunningAverageNode, NodeType::RunningAverage);
DEFINE_NFR(CopyNodeMask, NodeType::CopyMask);
DEFINE_NFR(FastMedianNode, NodeType::FastMedian);
DEFINE_NFR(FASTCornersNode, NodeType::FASTCorners);
DEFINE_NFR(DrawCornersNode, NodeType::DrawCorners);
DEFINE_NFR(BroadcastCornersNode, NodeType::BroadcastCorners);
DEFINE_NFR(ShiTomasiCornersNode, NodeType::ShiTomasiCorners);
DEFINE_NFR(PyramidNode, NodeType::Pyramid);
DEFINE_NFR(DrawLinesNode, NodeType::DrawLines);
DEFINE_NFR(BroadcastLinesNode, NodeType::BroadcastLines);
DEFINE_NFR(NullParamNode, NodeType::NullParam);
DEFINE_NFR(DrawHistogramNode, NodeType::DrawHistogram);
template<> DEFINE_NFR(ValueInputNode<int32_t>, NodeType::IntInput);
template<> DEFINE_NFR(ValueInputNode<float>, NodeType::FloatInput);
template<> DEFINE_NFR(ValueInputNode<bool>, NodeType::BoolInput);
template<> DEFINE_NFR(ValueInputNode<std::string>, NodeType::StringInput);
DEFINE_NFR(BroadcastHistogramNode, NodeType::BroadcastHistogram);
DEFINE_NFR(NopNode, NodeType::Nop);
DEFINE_NFR(ThrottleNode, NodeType::Throttle);
DEFINE_NFR(DelayNode, NodeType::Delay);
DEFINE_NFR(StitchNode, NodeType::Stitch);
DEFINE_NFR(RecogniserNode, NodeType::Recogniser);
template<> DEFINE_NFR(ClampNode<int>, NodeType::ClampInt);
template<> DEFINE_NFR(ClampNode<float>, NodeType::ClampFloat);
DEFINE_NFR(SURFCornersNode, NodeType::SURFCorners);
DEFINE_NFR(BroadcastKeypointsNode, NodeType::BroadcastKeypoints);

boost::try_mutex CameraInputNode::m_capture_lock[MAX_DEVICES];
const std::string DelayNode::Delay_Param_Name = "delay (frames)";

} // namespace imgproc
} // namespace cauv




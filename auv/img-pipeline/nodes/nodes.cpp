// !!! add new nodes to nodes2.cpp, not here, this file takes too long to
// compile already!

#include "../nodeFactory.h"

#include "bilateralFilterNode.h"
#include "broadcastCornersNode.h"
#include "broadcastHistogramNode.h"
#include "broadcastImageNode.h"
#include "broadcastKeypointsNode.h"
#include "broadcastLinesNode.h"
#include "cameraInputNode.h"
#include "cameraInputNode.h"
#include "cannyNode.h"
#include "centreFinderNode.h"
#include "combineHSVNode.h"
#include "combineRGBNode.h"
#include "combineYUVNode.h"
#include "convertColourNode.h"
#include "copyNode.h"
#include "copyNodeMask.h"
#include "cornerHarrisNode.h"
#include "cropNode.h"
#include "delayNode.h"
#include "directCameraInputNode.h"
#include "drawCornersNode.h"
#include "drawHistogramNode.h"
#include "drawLinesNode.h"
#include "fileInputNode.h"
#include "fileOutputNode.h"
#include "grabcutNode.h"
#include "guiOutputNode.h"
#include "histogramNode.h"
#include "histogramSegmentationArbNode.h"
#include "histogramSegmentationNode.h"
#include "houghCirclesNode.h"
#include "houghLinesNode.h"
#include "invertNode.h"
#include "kmeansNode.h"
#include "localDisplayNode.h"
#include "medianFilterNode.h"
#include "mergeSimilarLinesNode.h"
#include "mixNode.h"
#include "mixValueNode.h"
#include "netInputNode.h"
#include "nopNode.h"
#include "nullParamNode.h"
#include "percentileNode.h"
#include "pyramidNode.h"
#include "quickSegmentNode.h"
#include "recogniserNode.h"
#include "runningAverageNode.h"
#include "shiTomasiCornersNode.h"
#include "splitHSVNode.h"
#include "splitRGBNode.h"
#include "splitYUVNode.h"
#include "stitchNode.h"
#include "surfCornersNode.h"
#include "thresholdMaskNode.h"
#include "throttleNode.h"
#include "videoFileOutputNode.h"

namespace cauv{
namespace imgproc{

// Register node types (actually definitions of static data members)
DEFINE_NFR(CopyNode, NodeType::Copy);
DEFINE_NFR(FileInputNode, NodeType::FileInput);
DEFINE_NFR(FileOutputNode, NodeType::FileOutput);
DEFINE_NFR(LocalDisplayNode, NodeType::LocalDisplay);
DEFINE_NFR(CameraInputNode, NodeType::CameraInput);
DEFINE_NFR(DirectCameraInputNode, NodeType::DirectCameraInput);
DEFINE_NFR(NetInputNode, NodeType::NetInput);
DEFINE_NFR(HoughLinesNode, NodeType::HoughLines);
DEFINE_NFR(CannyNode, NodeType::Canny);
DEFINE_NFR(ConvertColourNode, NodeType::ConvertColour);
DEFINE_NFR(GuiOutputNode, NodeType::GuiOutput);
DEFINE_NFR(HoughCirclesNode, NodeType::HoughCircles);
DEFINE_NFR(MedianFilterNode, NodeType::MedianFilter);
DEFINE_NFR(BilateralFilterNode, NodeType::BilateralFilter);
DEFINE_NFR(SplitRGBNode, NodeType::SplitRGB);
DEFINE_NFR(CombineRGBNode, NodeType::CombineRGB);
DEFINE_NFR(SplitHSVNode, NodeType::SplitHSV);
DEFINE_NFR(SplitYUVNode, NodeType::SplitYUV);
DEFINE_NFR(CombineYUVNode, NodeType::CombineYUV);
DEFINE_NFR(CombineHSVNode, NodeType::CombineHSV);
DEFINE_NFR(MixNode, NodeType::Mix);
DEFINE_NFR(PercentileNode, NodeType::Percentile);
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
DEFINE_NFR(DrawCornersNode, NodeType::DrawCorners);
DEFINE_NFR(BroadcastCornersNode, NodeType::BroadcastCorners);
DEFINE_NFR(ShiTomasiCornersNode, NodeType::ShiTomasiCorners);
DEFINE_NFR(PyramidNode, NodeType::Pyramid);
DEFINE_NFR(DrawLinesNode, NodeType::DrawLines);
DEFINE_NFR(BroadcastLinesNode, NodeType::BroadcastLines);
DEFINE_NFR(NullParamNode, NodeType::NullParam);
DEFINE_NFR(DrawHistogramNode, NodeType::DrawHistogram);
DEFINE_NFR(BroadcastHistogramNode, NodeType::BroadcastHistogram);
DEFINE_NFR(NopNode, NodeType::Nop);
DEFINE_NFR(ThrottleNode, NodeType::Throttle);
DEFINE_NFR(DelayNode, NodeType::Delay);
DEFINE_NFR(StitchNode, NodeType::Stitch);
DEFINE_NFR(RecogniserNode, NodeType::Recogniser);
DEFINE_NFR(SURFCornersNode, NodeType::SURFCorners);
DEFINE_NFR(BroadcastKeypointsNode, NodeType::BroadcastKeyPoints);
DEFINE_NFR(MergeSimilarLinesNode, NodeType::MergeSimilarLines);


boost::try_mutex DirectCameraInputNode::m_capture_lock[MAX_DEVICES];
const std::string DelayNode::Delay_Param_Name = "delay (frames)";

} // namespace imgproc
} // namespace cauv




#include "../nodeFactory.h"

#include "resizeNode.h"
#include "levelsNode.h"
#include "fastCornersNode.h"
#include "drawKeyPointsNode.h"
#include "gaussianBlurNode.h"
#include "fastMedianNode.h"
#include "bearingRangeToXYNode.h"
#include "transformKeyPointsNode.h"
#include "correlation1DNode.h"
#include "rotateNode.h"
#include "renderStringNode.h"
#include "globalMaximumNode.h"
#include "localMaximaNode.h"
#include "firstAboveThresholdNode.h"
#include "broadcastPointsNode.h"
#ifdef CAUV_HAVE_TBB
#include "fitEllipseNode.h"
#endif
#include "kmeansPointsNode.h"
#include "drawEllipsesNode.h"
#include "sobelNode.h"
#include "blobNode.h"
#include "drawCirclesNode.h"
#include "broadcastCirclesNode.h"
#include "relabelClustersNode.h"
#include "fitGaussianNode.h"
#include "receiveLinesNode.h"
#include "meanStdNode.h"
#include "broadcastEllipsesNode.h"
#include "colourSimilarityNode.h"
#include "backgroundSubtractorNode.h"
#include "getNthNode.h"
#include "sumSquaredNode.h"
#include "broadcastFloatNode.h"
#include "meanShiftFilterNode.h"

namespace cauv{
namespace imgproc{

DEFINE_NFR(ResizeNode, NodeType::Resize);
DEFINE_NFR(LevelsNode, NodeType::Levels);
DEFINE_NFR(FASTCornersNode, NodeType::FASTCorners);
DEFINE_NFR(DrawKeyPointsNode, NodeType::DrawKeyPoints);
DEFINE_NFR(GaussianBlurNode, NodeType::GaussianBlur);
DEFINE_NFR(FastMedianNode, NodeType::FastMedian);
DEFINE_NFR(BearingRangeToXYNode, NodeType::BearingRangeToXY);
DEFINE_NFR(TransformKeyPointsNode, NodeType::TransformKeyPoints);
DEFINE_NFR(Correlation1DNode, NodeType::Correlation1D);
DEFINE_NFR(RotateNode, NodeType::Rotate);
DEFINE_NFR(RenderStringNode, NodeType::RenderString);
DEFINE_NFR(GlobalMaximumNode, NodeType::GlobalMaximum);
DEFINE_NFR(LocalMaximaNode, NodeType::LocalMaxima);
DEFINE_NFR(FirstAboveThresholdNode, NodeType::FirstAboveThreshold);
DEFINE_NFR(BroadcastPointsNode, NodeType::BroadcastPoints);
#ifdef CAUV_HAVE_TBB
DEFINE_NFR(FitEllipseNode, NodeType::FitEllipse);
#endif
DEFINE_NFR(KMeansPointsNode, NodeType::KMeansPoints);
DEFINE_NFR(DrawEllipsesNode, NodeType::DrawEllipses);
DEFINE_NFR(SobelNode, NodeType::Sobel);
DEFINE_NFR(BlobNode, NodeType::Blob);
DEFINE_NFR(DrawCirclesNode, NodeType::DrawCircles);
DEFINE_NFR(BroadcastCirclesNode, NodeType::BroadcastCircles);
DEFINE_NFR(RelabelClustersNode, NodeType::RelabelClusters);
DEFINE_NFR(FitGaussianNode, NodeType::FitGaussian);
DEFINE_NFR(ReceiveLinesNode, NodeType::ReceiveLines);
DEFINE_NFR(MeanStdNode, NodeType::MeanStd);
DEFINE_NFR(BroadcastEllipsesNode, NodeType::BroadcastEllipses);
DEFINE_NFR(ColourSimilarityNode, NodeType::ColourSimilarity);
DEFINE_NFR(BackgroundSubtractorNode, NodeType::BackgroundSubtractor);
template<> DEFINE_NFR(GetNthNode<Colour>, NodeType::GetNthColour);
DEFINE_NFR(SumSquaredNode, NodeType::SumSquared);
DEFINE_NFR(BroadcastFloatNode, NodeType::BroadcastFloat);
DEFINE_NFR(MeanShiftFilterNode, NodeType::MeanShiftFilter);

} // namespace imgproc
} // namespace cauv



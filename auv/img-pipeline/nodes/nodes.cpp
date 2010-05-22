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
#include "blurNode.h"

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
DEFINE_NFR(BlurNode, NodeType::Blur);


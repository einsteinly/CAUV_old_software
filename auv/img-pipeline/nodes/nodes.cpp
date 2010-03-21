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

// Register node types (actually definitions of static data members)
DEFINE_NFR(CopyNode, NodeType::Copy);
DEFINE_NFR(FileInputNode, NodeType::File_input);
DEFINE_NFR(FileOutputNode, NodeType::File_output);
DEFINE_NFR(ResizeNode, NodeType::Resize);
DEFINE_NFR(LocalDisplayNode, NodeType::Local_display);
DEFINE_NFR(CameraInputNode, NodeType::Camera_input);
DEFINE_NFR(NetInputNode, NodeType::Net_input);
DEFINE_NFR(HoughLinesNode, NodeType::Hough_lines);
DEFINE_NFR(CannyNode, NodeType::Canny);
DEFINE_NFR(ConvertColourNode, NodeType::Convert_colour);


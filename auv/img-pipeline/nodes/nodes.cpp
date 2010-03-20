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
DEFINE_NFR(CopyNode, nt_copy);
DEFINE_NFR(FileInputNode, nt_file_input);
DEFINE_NFR(FileOutputNode, nt_file_output);
DEFINE_NFR(ResizeNode, nt_resize);
DEFINE_NFR(LocalDisplayNode, nt_local_display);
DEFINE_NFR(CameraInputNode, nt_camera_input);
DEFINE_NFR(NetInputNode, nt_net_input);
DEFINE_NFR(HoughLinesNode, nt_hough_lines);
DEFINE_NFR(CannyNode, nt_canny);
DEFINE_NFR(ConvertColourNode, nt_convert_colour);


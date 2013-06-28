#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

int main() {

    CvCapture* capture = cvCaptureFromCAM( cv::CAP_ANY );
    if ( !capture ) {
        fprintf( stderr, "ERROR: capture is NULL \n" );
        getchar();
        return -1;
    }

    // Create a window in which the captured images will be presented
    cvNamedWindow( "CAUV OpenCV test", cv::WINDOW_AUTOSIZE );

    // Show the image captured from the camera in the window and repeat
    while ( 1 ) {
        // Get one frame
        IplImage* frame = cvQueryFrame( capture );
        if ( !frame ) {
            fprintf( stderr, "ERROR: frame is null...\n" );
            getchar();
            break;
        }

        cvShowImage( "CAUV OpenCV test", frame );
        // Do not release the frame!

        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if ( (cvWaitKey(10) & 255) == 27 ) break;
    }
    // Release the capture device housekeeping
    cvReleaseCapture( &capture );
    cvDestroyWindow( "CAUV OpenCV test" );
    return 0;
}


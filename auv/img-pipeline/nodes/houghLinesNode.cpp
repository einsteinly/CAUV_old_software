#include "houghLinesNode.h"

#include <map>
#include <string>
#include <limits>
#include <cmath>

#include <boost/variant.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions/hypot.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <generated/types/floatXY.h>
#include <generated/types/Line.h>

#include "../pipelineTypes.h"

using namespace cauv;

namespace {

    Line houghToLine(const cv::Point2f& hough_result, float rho, float theta, float width, float height)
    {
        float r = hough_result.x * rho;
        float t = hough_result.y * theta;
        floatXY centre(r * std::cos(t - M_PI_2) / width, r * std::sin(t - M_PI_2) / height);

        return Line(centre, (t > M_PI ? t - M_PI : t), std::numeric_limits<float>::infinity(), 0);
    }
    Line segmentToLine(const cv::Vec4i& segment, float width, float height)
    {
        floatXY top(segment[0], segment[1]);
        floatXY btm(segment[2], segment[3]);
        floatXY centre((top.x + btm.x) / (2 * width), (top.y + btm.y) / (2 * height));

        Line l;
        l.angle = std::atan2(top.y - btm.y, top.x - btm.x);
        if (l.angle < 0)
            l.angle += M_PI;
        l.centre = centre;
        l.length = boost::math::hypot(top.y - btm.y, top.x - btm.x)/width;
        l.width = 0;

        return l;
    }
    
    template<int n, typename T>
    struct is_not_max_predicate_t {
        const cv::Mat_<T>& mat;
        is_not_max_predicate_t(const cv::Mat_<T>& mat) : mat(mat) {}

        bool operator()(const cv::Point2i& p) const {
            T m = mat(p.x, p.y);
            for (int i = std::max(0,p.x - n), imax = std::min(mat.rows, p.x + n); i < imax; ++i)
                for (int j = std::max(0,p.y - n), jmax = std::min(mat.cols, p.y + n); j < jmax; ++j) {
                    if (i != p.x && j != p.y && mat(i,j) > m)
                        return true;
                }
            return false;
        }

    };

    template<int n, typename T>
    std::vector<cv::Point2i> findLocalMaxima(cv::Mat_<T>& mat, const T& threshold)
    {
        std::vector<cv::Point2i> block_maxima;
        for (int bi = 0, bimax = mat.rows; bi < bimax; bi+=n+1)
            for (int bj = 0, bjmax = mat.cols; bj < bjmax; bj+=n+1)
            {
                int mi = bi, mj = bj;
                T& m = mat(bi,bj);

                for (int i = bi, imax = std::min(bi+(n+1), mat.rows); i < imax; ++i)
                    for (int j = bj, jmax = std::min(bj+(n+1), mat.cols); j < jmax; ++j)
                    {
                        if (mat(i,j) > m) {
                            mi = i;
                            mj = j;
                            m = mat(i,j);
                        }
                    }

                if (m > threshold)
                    block_maxima.push_back(cv::Point2i(mi,mj));
            }
        
        std::remove_if(block_maxima.begin(), block_maxima.end(), is_not_max_predicate_t<n,T>(mat));
        return block_maxima;
    }

    struct HoughLinesVisitor : public boost::static_visitor< std::vector<Line> > {
        
        HoughLinesVisitor (bool probabilistic, float rho, float theta, int threshold, int min_ll, int max_lg, int srn, int stn)
            : probabilistic(probabilistic), rho(rho), theta(theta), threshold(threshold), min_ll(min_ll), max_lg(max_lg), srn(srn), stn(stn)
        {}

        std::vector<Line> operator()(const cv::Mat& m) const {

            std::vector<Line> lines;

            if (probabilistic) {
                cv::vector<cv::Vec4i> segments;
                // (const cast for backwards compatibility with older OpenCV versions)
                cv::HoughLinesP(const_cast<cv::Mat&>(m), segments, rho, theta, threshold, min_ll, max_lg);

                // Convert [x1,y1,x2,y2] segments to our lines
                std::transform(segments.begin(), segments.end(), std::back_inserter(lines), boost::bind(segmentToLine, _1, m.cols, m.rows));
            } else {
                // cv::vector<cv::Vec2f> hough_lines;
                // cv::HoughLines(m, hough_lines, rho, theta, threshold, srn, stn);
                // 
                // std::transform(hough_lines.begin(), hough_lines.end(), std::back_inserter(lines), boost::bind(opencvHoughToLine, _1, m.cols, m.rows));

                float rhoMax = boost::math::hypot(m.rows, m.cols);

                int numrho = std::ceil(rhoMax/rho);
                int numangle = std::ceil(3*M_PI_2/theta);

                cv::AutoBuffer<int> accum_buf;
                accum_buf.allocate(numangle * numrho);
                memset( &accum_buf[0], 0, sizeof(accum_buf[0]) * numangle * numrho );
                cv::Mat_<int> accum(numrho, numangle, &accum_buf[0]);

                cv::AutoBuffer<float> anglecos, anglesin;
                anglecos.allocate(numangle);
                anglesin.allocate(numangle);
                for (int n = 0; n < numangle; ++n) {
                    anglecos[n] = std::cos(n*theta - M_PI_2)/rho;
                    anglesin[n] = std::sin(n*theta - M_PI_2)/rho;
                }

                if (!m.isContinuous())
                    throw cauv::imgproc::parameter_error("Matrix needs to be continuous");

                uint8_t* pData = m.datastart;
                for (int y = 0, yend = m.rows; y < yend; ++y) {
                    for (int x = 0, xend = m.cols; x < xend; ++x)
                    {
                        if (*pData > 0) {
                            int r;
                            int n = 0;
                            while (n < numangle && (r = round( x * anglecos[n] + y * anglesin[n] )) < 0) {
                                ++n;
                            }
                            while (n < numangle && (r = round( x * anglecos[n] + y * anglesin[n] )) >= 0) {
                                if (r < numrho)
                                    ++accum(r,n);
                                ++n;
                            }
                        }
                        ++pData;
                    }
                }

                std::vector<cv::Point2i> maxima = findLocalMaxima<2>(accum, threshold);
                std::transform(maxima.begin(), maxima.end(), std::back_inserter(lines), boost::bind(houghToLine, _1, rho, theta, m.cols, m.rows));
            }
            
            return lines;
        }

        std::vector<Line> operator()(const NonUniformPolarMat& pm) const {
            const cv::Mat& m = pm.mat;

            float rhoMax = pm.ranges->back() * 2 * M_SQRT2;

            int numrho = std::ceil(rhoMax/rho);
            int numangle = std::ceil(3*M_PI_2/theta);

            cv::AutoBuffer<int> accum_buf;
            accum_buf.allocate(numangle * numrho);
            memset( &accum_buf[0], 0, sizeof(accum_buf[0]) * numangle * numrho );
            cv::Mat_<int> accum(numrho, numangle, &accum_buf[0]);

            cv::AutoBuffer<float> anglecos, anglesin;
            anglecos.allocate(numangle);
            anglesin.allocate(numangle);
            for (int n = 0; n < numangle; ++n) {
                anglecos[n] = std::cos(n*theta - M_PI_2)/rho;
                anglesin[n] = std::sin(n*theta - M_PI_2)/rho;
            }

            if (!m.isContinuous())
                throw cauv::imgproc::parameter_error("Matrix needs to be continuous");

            uint8_t* pData = m.datastart;
            for (int i = 0, iend = m.rows; i < iend; ++i) {
                for (int j = 0, jend = m.cols; j < jend; ++j)
                {
                    if (*pData > 0) {
                        cv::Point2f xy = pm.xyAt(i,j);
                        
                        int r;
                        int n = 0;
                        while (n < numangle && (r = round( xy.x * anglecos[n] + xy.y * anglesin[n] )) < 0) {
                            ++n;
                        }
                        while (n < numangle && (r = round( xy.x * anglecos[n] + xy.y * anglesin[n] )) > 0) {
                            if (r < numrho)
                                ++accum(r,n);
                            ++n;
                        }
                    }
                    ++pData;
                }
            }

            std::vector<cv::Point2i> maxima = findLocalMaxima<1>(accum, threshold);

            std::vector<Line> lines;
            std::transform(maxima.begin(), maxima.end(), std::back_inserter(lines), boost::bind(houghToLine, _1, rho, theta, m.cols, m.rows));

            return lines;
        }

        std::vector<Line> operator()(const PyramidMat&) const {
            error() << "HoughLinesNode: PyramidMat is unsupported";
            return std::vector<Line>();
        }

        bool probabilistic;
        float rho;
        float theta;
        int threshold;
        int min_ll, max_lg;
        int srn, stn;
    };

}

void cauv::imgproc::HoughLinesNode::doWork(in_image_map_t& inputs, out_map_t& r){

    image_ptr_t img = inputs[Image_In_Name];
    
    const bool probabilistic = param<bool>("probabilistic");
    const float rho = param<float>("rho");
    const float theta = param<float>("theta");
    const int threshold = param<int>("threshold");
    const int min_ll = param<int>("minLineLength");
    const int max_lg = param<int>("maxLineGap");
    const int srn = param<int>("srn");
    const int stn = param<int>("stn");

    try{
        r["lines"] = img->apply_visitor(HoughLinesVisitor(probabilistic, rho, theta, threshold, min_ll, max_lg, srn, stn));
    }catch(cv::Exception& e){
        error() << "HoughLinesNode:\n\t"
                << e.err << "\n\t"
                << "in" << e.func << "," << e.file << ":" << e.line;
    }
}


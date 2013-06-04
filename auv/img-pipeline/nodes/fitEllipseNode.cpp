/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "fitEllipseNode.h"

#define TBB_IMPLEMENT_CPP0X 1 //since clang doesn't implement is_trivially_copyable yet...
#include <tbb/tbb.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <utility/conic_section.h>
#include <utility/random.h>

namespace { // Anonymous namespace

    struct EllipseRansac_params {
        int minR;
        int maxR;
        int K;
        int inlierIterations;
        double maxRatio;
        int earlyTerminationPercentage; 

        EllipseRansac_params(
                int minR,
                int maxR,
                int K,
                int inlierIterations,
                double maxRatio,
                int earlyTerminationPercentage)
            : minR(minR), maxR(maxR), K(K), inlierIterations(inlierIterations), maxRatio(maxRatio), earlyTerminationPercentage(earlyTerminationPercentage)
        {}
    };
    struct EllipseRansac_out {
        std::vector<cv::Point> bestInliers;
        cv::RotatedRect bestEllipse;
        double bestEllipseGoodness;
        bool earlyTermination;

        EllipseRansac_out() : bestEllipseGoodness(-std::numeric_limits<double>::infinity()) {}
    };
    struct EllipseRansac {
        EllipseRansac_params params;

        const std::vector<cv::Point>& edgePoints;
        size_t n;
        const cv::Rect& bb;
        const cv::Mat_<float>& mDX;
        const cv::Mat_<float>& mDY;

        EllipseRansac_out out;

        EllipseRansac(
                EllipseRansac_params params,
                const std::vector<cv::Point>& edgePoints,
                size_t n,
                const cv::Rect& bb,
                const cv::Mat_<float>& mDX,
                const cv::Mat_<float>& mDY)
            : params(params), edgePoints(edgePoints), n(n), bb(bb), mDX(mDX), mDY(mDY)
        {
        }

        EllipseRansac(EllipseRansac& other, tbb::split)
            : params(other.params), edgePoints(other.edgePoints), n(other.n), bb(other.bb), mDX(other.mDX), mDY(other.mDY)                    {
                //std::cout << "Ransac split" << std::endl;
            }

        void operator()(const tbb::blocked_range<size_t>& r)
        {
            if (out.earlyTermination)
                return;
            //std::cout << "Ransac start (" << (r.end()-r.begin()) << " elements)" << std::endl;
            for( size_t i=r.begin(); i!=r.end(); ++i )
            {
                // Ransac Iteration
                // ----------------
                std::vector<cv::Point> sample;
                sample = cauv::randomSubset(edgePoints, n);

                cv::RotatedRect ellipseSampleFit = cv::fitEllipse(cv::Mat(sample));
                // Normalise ellipse to have width as the major axis.
                if (ellipseSampleFit.size.height > ellipseSampleFit.size.width)
                {
                    ellipseSampleFit.angle = std::fmod(ellipseSampleFit.angle + 90, 180);
                    std::swap(ellipseSampleFit.size.height, ellipseSampleFit.size.width);
                }

                cv::Size s = ellipseSampleFit.size;

                // Discard useless ellipses early
                if (!ellipseSampleFit.center.inside(bb)
                        || s.height > params.maxR*2 || s.width > params.maxR*2
                        || (s.height < params.minR*2 && s.width < params.minR*2)
                        || s.height > params.maxRatio*s.width
                        || s.width > params.maxRatio*s.height
                   )
                {
                    // Bad ellipse! Go to your room!
                    continue;
                }

                // Use conic section's algebraic distance as an error measure
                ConicSection conicSampleFit(ellipseSampleFit);

                // Check if sample's gradients are correctly oriented
                bool gradientCorrect = true;
                BOOST_FOREACH(const cv::Point2f& p, sample)
                {
                    cv::Point2f grad = conicSampleFit.algebraicGradientDir(p);
                    float dx = mDX(p);
                    float dy = mDY(p);

                    float dotProd = dx*grad.x + dy*grad.y;

                    gradientCorrect &= dotProd > 0;
                }
                if (!gradientCorrect)
                {
                    continue;
                }

                // Assume that the sample is the only inliers

                cv::RotatedRect ellipseInlierFit = ellipseSampleFit;
                ConicSection conicInlierFit = conicSampleFit;
                std::vector<cv::Point> inliers, prevInliers;

                // Iteratively find inliers, and re-fit the ellipse
                for (int i = 0; i < params.inlierIterations; ++i)
                {
                    // Get error scale for 1px out on the minor axis
                    cv::Point2f minorAxis(-std::sin(M_PI/180.0*ellipseInlierFit.angle), std::cos(M_PI/180.0*ellipseInlierFit.angle));
                    cv::Point2f minorAxisPlus1px = ellipseInlierFit.center + (ellipseInlierFit.size.height/2 + 1)*minorAxis;
                    float errOf1px = conicInlierFit.distance(minorAxisPlus1px);
                    float errorScale = 1.0f/errOf1px;

                    // Find inliers
                    inliers.reserve(edgePoints.size());
                    const float MAX_ERR = 2;
                    BOOST_FOREACH(const cv::Point& p, edgePoints)
                    {
                        float err = errorScale*conicInlierFit.distance(p);

                        if (err*err < MAX_ERR*MAX_ERR)
                            inliers.push_back(p);
                    }

                    if (inliers.size() < n) {
                        inliers.clear();
                        continue;
                    }

                    // Refit ellipse to inliers
                    ellipseInlierFit = cv::fitEllipse(cv::Mat(inliers));
                    conicInlierFit = ConicSection(ellipseInlierFit);

                    // Normalise ellipse to have width as the major axis.
                    if (ellipseInlierFit.size.height > ellipseInlierFit.size.width)
                    {
                        ellipseInlierFit.angle = std::fmod(ellipseInlierFit.angle + 90, 180);
                        std::swap(ellipseInlierFit.size.height, ellipseInlierFit.size.width);
                    }
                }
                if (inliers.empty())
                    continue;


                // Calculate ellipse goodness
                double ellipseGoodness = 0;
                BOOST_FOREACH(cv::Point& p, inliers)
                {
                    cv::Point2f grad = conicInlierFit.algebraicGradientDir(p);
                    float dx = mDX(p);
                    float dy = mDY(p);

                    double edgeStrength = dx*grad.x + dy*grad.y;

                    ellipseGoodness += edgeStrength;
                }

                if (ellipseGoodness > out.bestEllipseGoodness)
                {
                    std::swap(out.bestEllipseGoodness, ellipseGoodness);
                    std::swap(out.bestInliers, inliers);
                    std::swap(out.bestEllipse, ellipseInlierFit);

                    // Early termination, if 90% of points match
                    if (params.earlyTerminationPercentage > 0 && out.bestInliers.size() > params.earlyTerminationPercentage*edgePoints.size()/100)
                    {
                        out.earlyTermination = true;
                        break;
                    }
                }

            }
            //std::cout << "Ransac end" << std::endl;
        }

        void join(EllipseRansac& other)
        {
            //std::cout << "Ransac join" << std::endl;
            if (other.out.bestEllipseGoodness > out.bestEllipseGoodness)
            {
                std::swap(out.bestEllipseGoodness, other.out.bestEllipseGoodness);
                std::swap(out.bestInliers, other.out.bestInliers);
                std::swap(out.bestEllipse, other.out.bestEllipse);
            }

            out.earlyTermination |= other.out.earlyTermination;
        }
    };
}

void cauv::imgproc::FitEllipseNode::doWork(in_image_map_t& inputs, out_map_t& r){
    cv::Mat_<float> dx = inputs["imageDx"]->mat();
    cv::Mat_<float> dy = inputs["imageDy"]->mat();
    cv::Mat_<uchar> edges = inputs["edge_image"]->mat();

    const int minR = param<int>("minRadius");
    const int maxR = param<int>("maxRadius");
    const int K = param<int>("maxIters");
    const int inlierIterations = 2;
    const double maxRatio = 2;
    const int earlyTerminationPercentage = 90;

    if(dx.depth() != CV_32F || dx.channels() > 1)
        throw(parameter_error("Image x gradient must be one channel float."));
    if(dy.depth() != CV_32F || dy.channels() > 1)
        throw(parameter_error("Image y gradent must be one channel float."));
    if(edges.depth() != CV_8U || edges.channels() > 1)
        throw(parameter_error("Edge image must be one channel unsigned bytes."));

    if (dx.rows != dy.rows || dx.rows != edges.rows
            || dx.cols != dy.cols || dx.cols != edges.cols)
        throw(parameter_error("Images must be same size."));

    cv::Rect bb(0,0,dx.rows,dx.cols);

    // -----------------------------------------------
    // Find edge points in edge image (non-zero values)
    // -----------------------------------------------
    std::vector<cv::Point> edgePoints;
    for(int y = 0; y < edges.rows; y++)
    {
        uchar* val = edges[y];
        for(int x = 0; x < edges.cols; x++, val++)
        {
            if(*val == 0)
                continue;

            edgePoints.push_back(cv::Point(x,y));
        }
    } 

    // -----------
    // Fit ellipse
    // -----------

    // Number of points needed for a model
    const size_t n = 5;

    if (edgePoints.size() >= n) // Minimum points for ellipse
    {
        // RANSAC!!!

        // Use TBB for RANSAC

        EllipseRansac ransac(EllipseRansac_params(minR,maxR,K,inlierIterations,maxRatio,earlyTerminationPercentage), edgePoints, n, bb, dx, dy);
        try
        { 
            tbb::parallel_reduce(tbb::blocked_range<size_t>(0,K,K/8), ransac);
        }
        catch (std::exception& e)
        {
            error() << e.what();
        }

        cv::RotatedRect el = ransac.out.bestEllipse;
        std::vector<Ellipse> ret;
        ret.push_back(Ellipse(floatXY(el.center.x/edges.cols, el.center.y/edges.rows), el.size.width / 2 / edges.cols, el.size.height / 2 / edges.cols, el.angle));
        r["ellipse"] = ret;
    }
}

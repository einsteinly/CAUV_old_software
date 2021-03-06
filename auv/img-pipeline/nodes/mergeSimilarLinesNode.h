/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __MERGE_SIMILAR_LINESNODE_H__
#define __MERGE_SIMILAR_LINESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <utility/math.h>

#include <generated/types/Line.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class MergeSimilarLinesNode: public Node{
    public:
        MergeSimilarLinesNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = slow;
            
            // one output
            registerOutputID("lines", std::vector<Line>());
            
            // parameters:
            registerParamID< float >("angle epsilon", 0.3f);
            registerParamID< float >("distance epsilon", 0.1f);
            registerParamID< std::vector<Line> >("lines", std::vector<Line>(), "", Must_Be_New);
        }

    protected:
        float crossProdVal(Eigen::Vector2f const& v1, Eigen::Vector2f const& v2) {
            return v1[0] * v2[1] - v1[1] * v2[0];
        }
        Eigen::Vector2f lineIntersection(Line const& line1, Line const& line2) {
            Eigen::Vector2f p1(line1.centre.x, line1.centre.y);
            Eigen::Vector2f p2(line2.centre.x, line2.centre.y);
            Eigen::Vector2f v1(std::cos(line1.angle), std::sin(line1.angle));
            Eigen::Vector2f v2(std::cos(line2.angle), std::sin(line2.angle));
            
            if (line1.angle == line2.angle)
                return p1;

            float v1xv2 = crossProdVal(v1,v2);
            float p2p1xv2 = crossProdVal(p2 - p1, v2);

            float s = p2p1xv2 / v1xv2;

            return p1 + s*v1;
        }

        float pointLineDistance(Eigen::Vector2f const& p, Line const& l) {
            Eigen::Vector2f c(l.centre.x, l.centre.y);
            Eigen::Vector2f v(std::cos(l.angle), std::sin(l.angle)); 
            return std::abs((p - c).dot(v.unitOrthogonal()));
        }
        float pointLineDistance(floatXY const& p, Line const& l) {
            return pointLineDistance(Eigen::Vector2f(p.x,p.y), l);
        }

        Line mergeLines(Line const& line1, Line const& line2) {
            using std::abs;

            Eigen::Vector2f p1(line1.centre.x, line1.centre.y);
            Eigen::Vector2f p2(line2.centre.x, line2.centre.y);
            Eigen::Vector2f v1(std::cos(line1.angle), std::sin(line1.angle));
            Eigen::Vector2f v2(std::cos(line2.angle), std::sin(line2.angle));
            float l1 = line1.length;
            float l2 = line2.length;
            bool is_infinite = boost::math::isinf(l1) || boost::math::isinf(l1);
            if (is_infinite) {
                l1 = 1;
                l2 = 1;
            }

            float t1 = angle(line1.angle);
            float t2 = angle(line2.angle);

            Eigen::Vector2f a = p1 - v1*l1/2;
            Eigen::Vector2f b = p1 + v1*l1/2;
            Eigen::Vector2f c = p2 - v2*l2/2;
            Eigen::Vector2f d = p2 + v2*l2/2;

            // Centroid weighted by line length
            Eigen::Vector2f G = (l1*(a+b) + l2*(c+d))/(2*(l1+l2));

            float newAngle;
            if (abs(t1- t2) <= M_PI_2)
                newAngle = (l1*t1 + l2*t2)/(l1+l2);
            else
                newAngle = (l1*t1 + l2*(t2 - (t2>0?M_PI:-M_PI)))/(l1+l2);
            newAngle = angle(newAngle);
            Eigen::Vector2f v(std::cos(newAngle), std::sin(newAngle));
            
            Eigen::Vector2f endPoints[] = {a,b,c,d};
            float newLength = 0;
            Eigen::Vector2f newCentre = G;
            for (int i = 0; i < 4; ++i)
                for(int j = i+1; j < 4; ++j)
                {
                    float length = is_infinite ? std::numeric_limits<float>::infinity() : abs((endPoints[i] - endPoints[j]).dot(v));
                    if (length > newLength) {
                        newLength = length;
                        
                        Eigen::Vector2f proj1 ((endPoints[j] - G).dot(v.unitOrthogonal()), (endPoints[i] - G).dot(v));
                        Eigen::Vector2f proj2 ((endPoints[j] - G).dot(v.unitOrthogonal()), (endPoints[j] - G).dot(v));

                        newCentre = G + v*(proj1[0] + proj2[0])/2;
                    }
                }

            Line l(floatXY(newCentre[0],newCentre[1]), newAngle, newLength, 0);

            //Eigen::Vector2f c = lineIntersection(line1, line2);

            // new angle and length weighted by line length
            
            //float newAngle = line1.angle + angle(line2.angle - line1.angle) / 2;
            //float newLength = (line1.length*line1.length + line2.length*line2.length) / (line1.length + line2.length);

            //Line l(floatXYZ(c[0],c[1],0), newAngle, newLength);
    
            //Eigen::Vector2f c2 = lineIntersection(l, Line(line1.centre, std::atan2(line2.centre.y-line1.centre.y,line2.centre.x-line1.centre.x),0));
            //l.centre.x = c2[0];
            //l.centre.y = c2[1];
            
            return l;
        }
        float centreErr(Line const& line1, Line const& line2) {
            float line1d = pointLineDistance(line2.centre, line1);
            float line2d = pointLineDistance(line1.centre, line2);
            if (boost::math::isinf(line1.length) || boost::math::isinf(line2.length))
                return (line1d*line2d)/2;
            else
                return (line1d*line1.length+line2d*line1.length)/(line1.length + line2.length);
        }
        float angleErr(Line const& line1, Line const& line2) {
            return angleErr(line1.angle, line2.angle);
        }
        float angleErr(float a1, float a2) {
            using std::sin;
            using std::cos;
            using std::acos;
            float x1 = cos(a1), y1 = sin(a1);
            float x2 = cos(a2), y2 = sin(a2);
            return acos(x1*y2 - y1*x2);
            //return std::abs(angle(a1 - a2));
        }
        float angle(float a) {
            return mod<float>(a, M_PI);
        }

        void doWork(in_image_map_t&, out_map_t& r){
            const float angleEpsilon = param< float >("angle epsilon");
            const float distanceEpsilon = param< float >("distance epsilon");
            const std::vector<Line> lines = param< std::vector<Line> >("lines");

            try{
                std::vector<Line> mergedLines;
                for (const Line& line1 : lines)
                {
                    bool merged = false;
                    for (Line& line2 : mergedLines)
                    {
                        if (angleErr(line1,line2) < angleEpsilon && centreErr(line1,line2) < distanceEpsilon)
                        {
                            line2 = mergeLines(line1,line2);
                            merged = true;
                            break;
                        }
                    }
                    if (!merged)
                        mergedLines.push_back(line1);
                }
                r["lines"] = mergedLines;
            }catch(cv::Exception& e){
                error() << "MergeSimilarLinesNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MERGE_SIMILAR_LINESNODE_H__


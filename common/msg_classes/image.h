/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_IMAGE_H__
#define __CAUV_IMAGE_H__

#include <boost/cstdint.hpp>
#include <boost/variant.hpp>
#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>

#include <debug/cauv_debug.h>

#include "base_image.h"

namespace cauv{

// these extra-information structures are passed around by value - for the sake
// of thread safety: so lumps of data should be included via indirection with
// locking where appropriate
// Also note that they must be copy-constructible, and that they must have a
// clone() method that deep-copies all associated data
// Serialising augmented data is TODO...
struct NonUniformPolarMat{
    // these choices are motivated my the data order out of the gemini
    // (isodistant lines)
    // all elements in row[i] have range range[i]
    // all elements in column[j] have bearing bearing[j]
    cv::Mat mat;
    boost::shared_ptr< std::vector<float> > ranges;   // in metres
    boost::shared_ptr< std::vector<float> > bearings; // in radians please!

    NonUniformPolarMat clone() const;

    cv::Point2f xyAt(int r, int phi) const;
    cv::Point2f xyAt(int r, int phi, float& range, float& bearing) const;
    cv::Point2f xyAt(float r, float phi) const;
    cv::Point2f xyAt(float r, float phi, float& range, float& bearing) const;
};

struct PyramidMat{
    std::vector<cv::Mat> levels;
    PyramidMat clone() const;    
};

// keep this corresponding to the variant - the Right Thing To Do is to use a
// visitor rather than indexing though...
namespace AugmentedType{
enum e{
    NotAugmented,
    NonUniformPolar,
    Pyramid
};
} // namespace AugmentedType

typedef boost::variant<
    cv::Mat,
    NonUniformPolarMat,
    PyramidMat
> augmented_mat_t;

template<typename Func, typename TRet>
struct FuncVisitor : boost::static_visitor< TRet >{
    public:
        FuncVisitor(const Func& func) : func(func) {
        }

        TRet operator()(cv::Mat& a) const {
            return func(a);
        }
        TRet operator()(NonUniformPolarMat& a) const {
            return func(a.mat);
        }
        TRet operator()(PyramidMat&) const {
            error() << "no support for pyramids";
            return TRet();
        }

    private:
        Func func;
};

template<typename Func>
struct FuncVisitor<Func,void> : boost::static_visitor< void >{
    public:
        FuncVisitor(const Func& func) : func(func) {
        }

        void operator()(cv::Mat& a) const {
            func(a);
        }
        void operator()(NonUniformPolarMat& a) const {
            func(a.mat);
        }
        void operator()(PyramidMat& a) const {
            for (std::vector<cv::Mat>::iterator it = a.levels.begin(), itend = a.levels.end(); it != itend; ++it)
                func(*it);
        }

    private:
        Func func;
};

template<typename Func>
struct FuncVisitor<Func,cv::Mat> : boost::static_visitor<augmented_mat_t>{
    public:
        FuncVisitor(const Func& func) : func(func) {
        }

        augmented_mat_t operator()(cv::Mat& a) const {
            return func(a);
        }
        augmented_mat_t operator()(NonUniformPolarMat& a) const {
            NonUniformPolarMat ret;
            ret.mat = func(a.mat);
            ret.ranges = boost::make_shared<std::vector<float> >(*a.ranges);
            ret.bearings = boost::make_shared<std::vector<float> >(*a.bearings);
            return ret;
        }
        augmented_mat_t operator()(PyramidMat& a) const {
            PyramidMat ret;
            for (std::vector<cv::Mat>::iterator it = a.levels.begin(), itend = a.levels.end(); it != itend; ++it)
                ret.levels.push_back(func(*it));
            return ret;
        }

    private:
        Func func;
};

class Image : public BaseImage {
    public:
        Image();
        Image(augmented_mat_t const& augmented_image);
        Image(augmented_mat_t const& augmented_image, TimeStamp const& ts);
        Image(augmented_mat_t const& augmented_image, TimeStamp const& ts, UID const& id);
        Image(Image const& other);
        Image& operator=(Image const& other);
        ~Image();

        // Nodes that wish to support pyramid images (not implemented yet), or
        // polar images should use only the augmentedMat functions
        cv::Mat mat() const;
        void mat(cv::Mat const& mat);
        
        // NB: this can always be used, cv::Mat is one of the possible types of
        // the returned variant, and it will be returned if this image is, in
        // fact
        augmented_mat_t augmentedMat() const;
        void augmentedMat(augmented_mat_t const& mat);
        
        // return by value: the compiler will optimise this to a move
        virtual svec_t encodeBytes() const;
        virtual void encodedBytes(svec_t const&);

        virtual uint32_t channels() const;

        //why is this float?
        float bits() const;

        template<typename Func>
        typename FuncVisitor<Func, typename Func::result_type>::result_type apply(const Func& func)
        {
            return boost::apply_visitor(FuncVisitor<Func, typename Func::result_type>(func), m_img);
        }

        template<typename TRet, typename Func>
        TRet apply(const Func& func)
        {
            return boost::apply_visitor(FuncVisitor<Func, TRet>(func), m_img);
        }

        template<typename Visitor>
        typename Visitor::result_type apply_visitor(const Visitor& visitor)
        {
            return boost::apply_visitor(visitor, m_img);
        }

    private:
        augmented_mat_t m_img;
};

} // namespace cauv

#endif // ndef __CAUV_IMAGE_H__

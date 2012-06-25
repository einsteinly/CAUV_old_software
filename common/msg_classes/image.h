/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_IMAGE_H__
#define __CAUV_IMAGE_H__

#include <boost/cstdint.hpp>
#include <boost/variant.hpp>
#include <boost/utility/result_of.hpp>

#include <opencv2/core/core.hpp>

#include <debug/cauv_debug.h>
#include <utility/streamops.h>

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

        TRet operator()(cv::Mat a) const {
            return func(a);
        }
        TRet operator()(NonUniformPolarMat a) const {
            return operator()(a.mat);
        }
        TRet operator()(PyramidMat) const {
            error() << "no support for pyramids";
            return TRet();
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

        virtual svec_t &bytes(void) const;
        virtual void bytes(svec_t&);

        virtual uint32_t channels(void) const;

        //why is this float?
        float bits() const;

        template<typename Func>
        typename Func::result_type apply(const Func& func)
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

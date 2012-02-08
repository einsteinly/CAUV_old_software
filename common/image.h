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

#include <opencv2/core/core.hpp>

#include <utility/streamops.h>
#include <utility/serialisation-types.h>

#include <generated/types/TimeStamp.h>
#include <generated/types/UID.h>

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

class Image{
        friend void serialise(svec_ptr, Image const&);
        friend int32_t deserialise(const_svec_ptr, uint32_t i, Image&);
    public:
        Image();
        Image(augmented_mat_t const& augmented_image);
        Image(augmented_mat_t const& augmented_image, TimeStamp const& ts);
        Image(augmented_mat_t const& augmented_image, TimeStamp const& ts, UID const& id);
        Image(Image const& other);
        Image& operator=(Image const& other);
        ~Image();

        TimeStamp ts() const;
        void ts(TimeStamp const&);
        
        // UID is unique to the instance in which a sensor records data, and
        // can be used to associate derived data with other data deriving from
        // the same sensor reading.
        // If data derives from multiple sensors or multiple sensor readings or
        // multiple sensors, then it should probably get a new UID at that
        // point.
        cauv::UID id() const;
        void id(cauv::UID const& uid);
        
        // Nodes that wish to support pyramid images (not implemented yet), or
        // polar images should use only the augmentedMat functions
        cv::Mat mat() const;
        void mat(cv::Mat const& mat);
        
        // NB: this can always be used, cv::Mat is one of the possible types of
        // the returned variant, and it will be returned if this image is, in
        // fact
        augmented_mat_t augmentedMat() const;
        void augmentedMat(augmented_mat_t const& mat);

        void serializeQuality(int32_t);

    private:
        void setDefaultCompressParams();

        augmented_mat_t m_img;
        TimeStamp m_ts;
        std::string m_compress_fmt;
        std::vector<int32_t> m_compress_params;
        cauv::UID m_uid;
};

/* image serialisation */
void serialise(svec_ptr p, Image const& v);
int32_t deserialise(const_svec_ptr p, uint32_t i, Image& v);
std::string chil(Image const&);

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Image const&){
    os << "{Image}";
    return os;
}

} // namespace cauv

#endif // ndef __CAUV_IMAGE_H__

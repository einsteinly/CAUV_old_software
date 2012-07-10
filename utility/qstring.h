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

#ifndef __CAUV_QT_STRING_H__
#define __CAUV_QT_STRING_H__

#include <QTextStream>
#include <QString>

namespace cauv{

// somewhat simpler than the std::string version
class MakeQString{
    public:
        MakeQString()
            : m_string(),
              m_stream(&m_string){
        }

        template<typename T>
        MakeQString& operator<<(T const& t){
            m_stream << t;
            return *this;
        }

        operator QString() const{
            return m_string;
        }

    private:
        QString m_string;
        QTextStream m_stream;
};

typedef MakeQString mkQStr;

} // namespace cauv

#endif // ndef __CAUV_QT_STRING_H__


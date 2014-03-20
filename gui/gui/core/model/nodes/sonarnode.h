/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GUI_SONARNODE_H
#define GUI_SONARNODE_H

#include <QMetaType>

#include <gui/core/model/nodes/imagenode.h>
#include <gui/core/model/nodes/numericnode.h>

namespace cauv {
    namespace gui {

        class SonarNode : public ImageNode {
            Q_OBJECT

        public:
            SonarNode(std::string const& id) : ImageNode(id){
                type = nodeType<SonarNode>();
            }

            boost::shared_ptr<NumericNode<int> > direction() {
                if(!m_direction) {
                    m_direction = findOrCreate<NumericNode<int> >("direction");
                    m_direction->setMutable(true);
                }
                return m_direction;
            }

            boost::shared_ptr<NumericNode<int> > width() {
                if(!m_width) {
                    m_width = findOrCreate<NumericNode<int> >("width");
                    m_width->setMutable(true);
                }
                return m_width;
            }

            boost::shared_ptr<NumericNode<int> > gain() {
                if(!m_gain) {
                    m_gain = findOrCreate<NumericNode<int> >("gain");
                    m_gain->setMutable(true);
                }
                return m_gain;
            }

            boost::shared_ptr<NumericNode<unsigned int> > range() {
                if(!m_range) {
                    m_range = findOrCreate<NumericNode<unsigned int> >("range");
                    m_range->setMutable(true);
                }
                return m_range;
            }

            boost::shared_ptr<NumericNode<unsigned int> > rangeRes() {
                if(!m_rangeRes){
                    m_rangeRes = findOrCreate<NumericNode<unsigned int> >("rangeRes");
                    m_rangeRes->setMutable(true);
                }
                return m_rangeRes;
            }

            boost::shared_ptr<NumericNode<unsigned int> > angularRes() {
                if(!m_angularRes) {
                    m_angularRes = findOrCreate<NumericNode<unsigned int> >("angularRes");
                    m_angularRes->setMutable(true);
                }
                return m_angularRes;
            }

        protected:
            boost::shared_ptr<NumericNode<int> > m_direction;
            boost::shared_ptr<NumericNode<int> > m_width;
            boost::shared_ptr<NumericNode<int> > m_gain;
            boost::shared_ptr<NumericNode<unsigned int> > m_range;
            boost::shared_ptr<NumericNode<unsigned int> > m_rangeRes;
            boost::shared_ptr<NumericNode<unsigned int> > m_angularRes;
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_SONARNODE_H

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

#ifndef GUI_AUTOPILOTNODE_H
#define GUI_AUTOPILOTNODE_H

#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/groupingnode.h>

namespace cauv {
    namespace gui {

        typedef NumericNode<int> MotorNode;

        class AutopilotParamsNode : public GroupingNode {
            Q_OBJECT

        public:
            AutopilotParamsNode(nid_t const& id) : GroupingNode(id)
            {}

            boost::shared_ptr<NumericNode<float> > kP() {
                if(!m_kP){
                    m_kP = findOrCreate<NumericNode<float> >("kP");
                    m_kP->setMutable(true);
                }
                return m_kP;
            }
            boost::shared_ptr<NumericNode<float> > kI() {
                if(!m_kI) {
                    m_kI = findOrCreate<NumericNode<float> >("kI");
                    m_kI->setMutable(true);
                }
                return m_kI;
            }
            boost::shared_ptr<NumericNode<float> > kD() {
                if(!m_kD) {
                    m_kD = findOrCreate<NumericNode<float> >("kD");
                    m_kD->setMutable(true);
                }
                return m_kD;
            }
            boost::shared_ptr<NumericNode<float> > scale() {
                if(!m_scale) {
                    m_scale = findOrCreate<NumericNode<float> >("scale");
                    m_scale->setMutable(true);
                }
                return m_scale;
            }
            boost::shared_ptr<NumericNode<float> > aP() {
                if(!m_aP) {
                    m_aP = findOrCreate<NumericNode<float> >("aP");
                    m_aP->setMutable(true);
                }
                return m_aP;
            }
            boost::shared_ptr<NumericNode<float> > aI() {
                if(!m_aI) {
                    m_aI = findOrCreate<NumericNode<float> >("aI");
                    m_aI->setMutable(true);
                }
                return m_aI;
            }
            boost::shared_ptr<NumericNode<float> > aD() {
                if(!m_aD) {
                    m_aD = findOrCreate<NumericNode<float> >("aD");
                    m_aD->setMutable(true);
                }
                return m_aD;
            }
            boost::shared_ptr<NumericNode<float> > thr() {
                if(!m_thr) {
                    m_thr = findOrCreate<NumericNode<float> >("thr");
                    m_thr->setMutable(true);
                }
                return m_thr;
            }
            boost::shared_ptr<NumericNode<float> > maxError() {
                if(!m_maxError) {
                    m_maxError = findOrCreate<NumericNode<float> >("maxError");
                    m_maxError->setMutable(true);
                }
                return m_maxError;
            }

        protected:
            boost::shared_ptr<NumericNode<float> > m_kP;
            boost::shared_ptr<NumericNode<float> > m_kI;
            boost::shared_ptr<NumericNode<float> > m_kD;
            boost::shared_ptr<NumericNode<float> > m_scale;
            boost::shared_ptr<NumericNode<float> > m_aP;
            boost::shared_ptr<NumericNode<float> > m_aI;
            boost::shared_ptr<NumericNode<float> > m_aD;
            boost::shared_ptr<NumericNode<float> > m_thr;
            boost::shared_ptr<NumericNode<float> > m_maxError;
        };


        class AutopilotNode : public BooleanNode {
            Q_OBJECT

        public:
            AutopilotNode(nid_t const& id) : BooleanNode(id)
            {}

            boost::shared_ptr<AutopilotParamsNode> getParams() {
                if(!m_params) m_params = findOrCreate<AutopilotParamsNode >("params");
                return m_params;
            }

        protected:
            boost::shared_ptr<AutopilotParamsNode> m_params;
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_AUTOPILOTNODE_H

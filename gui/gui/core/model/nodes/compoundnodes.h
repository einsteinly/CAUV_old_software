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

#ifndef GUI_COMPOUNDNODE_H
#define GUI_COMPOUNDNODE_H

#include "../node.h"
#include "numericnode.h"

#include <generated/types/floatXYZ.h>
#include <generated/types/floatYPR.h>

namespace cauv {
    namespace gui {

        template<class T>
        class CompoundNode : public Node<T> {

        public:
            typedef T type;

            CompoundNode(GuiNodeType::e GuiNodeType, id_variant_t const& id) :
                    Node<T>(GuiNodeType, id) {
            }

            virtual void forceSet() = 0;
        };



        class FloatYPRNode : public CompoundNode <floatYPR> {
            Q_OBJECT

        public:
            FloatYPRNode(id_variant_t const& id) : CompoundNode<floatYPR>(GuiNodeType::FloatYPRNode, id)
            {
                this->connect(this, SIGNAL(changed()), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            virtual void update(floatYPR const& value){
                CompoundNode<floatYPR>::update(value);
                this->findOrCreate<TypedNumericNode<float> >("yaw")->update(value.yaw);
                this->findOrCreate<TypedNumericNode<float> >("pitch")->update(value.pitch);
                this->findOrCreate<TypedNumericNode<float> >("roll")->update(value.roll);
            }

            virtual void set(type const& value){
                CompoundNode<floatYPR>::set(value);
            }

            virtual void forceSet() {
                float y = this->findOrCreate<TypedNumericNode<float> >("yaw")->get();
                float p = this->findOrCreate<TypedNumericNode<float> >("pitch")->get();
                float r = this->findOrCreate<TypedNumericNode<float> >("roll")->get();
                set(type(y, p, r));
            }

        Q_SIGNALS:
            void onUpdate(floatYPR const& value);
            void onSet(floatYPR const& value);
        };



        class FloatXYZNode : public CompoundNode <floatXYZ> {
            Q_OBJECT

        public:
            FloatXYZNode(id_variant_t const& id) : CompoundNode<floatXYZ>(GuiNodeType::FloatXYZNode, id)
            {
                this->connect(this, SIGNAL(changed()), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            virtual void update(floatXYZ const& value){
                CompoundNode<floatXYZ>::update(value);
                this->findOrCreate<TypedNumericNode<float> >("x")->update(value.x);
                this->findOrCreate<TypedNumericNode<float> >("y")->update(value.y);
                this->findOrCreate<TypedNumericNode<float> >("z")->update(value.z);
            }

            virtual void set(type const& value){
                CompoundNode<floatXYZ>::set(value);
            }

            virtual void forceSet() {
                float x = boost::get<float>(this->findOrCreate<TypedNumericNode<float> >("x")->get());
                float y = boost::get<float>(this->findOrCreate<TypedNumericNode<float> >("y")->get());
                float z = boost::get<float>(this->findOrCreate<TypedNumericNode<float> >("z")->get());
                set(type(x, y, z));
            }

        Q_SIGNALS:
            void onUpdate(floatXYZ const& value);
            void onSet(floatXYZ const& value);
        };


    } //namespace gui
} // namespace cauv

#endif // GUI_COMPOUNDNODE_H

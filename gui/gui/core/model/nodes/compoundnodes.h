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

            CompoundNode(GuiNodeType::e GuiNodeType, const id_variant_t id) :
                    Node<T>(GuiNodeType, id) {
            }

            virtual void forceSet() = 0;
        };



        class FloatYPRNode : public CompoundNode <floatYPR> {
            Q_OBJECT

        public:
            FloatYPRNode(const id_variant_t id) : CompoundNode<floatYPR>(GuiNodeType::FloatYPRNode, id)
            {
                this->connect(this, SIGNAL(changed()), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            virtual void update(const floatYPR & value){
                CompoundNode<floatYPR>::update(value);
                Q_EMIT onUpdate(value);
                this->findOrCreate<TypedNumericNode<float> >("yaw")->update(value.yaw);
                this->findOrCreate<TypedNumericNode<float> >("pitch")->update(value.pitch);
                this->findOrCreate<TypedNumericNode<float> >("roll")->update(value.roll);
            }

            virtual void set(const floatYPR & value){
                CompoundNode<floatYPR>::set(value);
                Q_EMIT onSet(value);
            }

            virtual void forceSet() {
                float y = this->findOrCreate<TypedNumericNode<float> >("yaw")->get();
                float p = this->findOrCreate<TypedNumericNode<float> >("pitch")->get();
                float r = this->findOrCreate<TypedNumericNode<float> >("roll")->get();
                set(floatYPR(y, p, r));
            }

        Q_SIGNALS:
            void onUpdate(const floatYPR value);
            void onSet(const floatYPR value);
        };



        class FloatXYZNode : public CompoundNode <floatXYZ> {
            Q_OBJECT

        public:
            FloatXYZNode(const id_variant_t id) : CompoundNode<floatXYZ>(GuiNodeType::FloatXYZNode, id)
            {
                this->connect(this, SIGNAL(changed()), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            virtual void update(const floatXYZ & value){
                CompoundNode<floatXYZ>::update(value);
                Q_EMIT onUpdate(value);
                this->findOrCreate<TypedNumericNode<float> >("x")->update(value.x);
                this->findOrCreate<TypedNumericNode<float> >("y")->update(value.y);
                this->findOrCreate<TypedNumericNode<float> >("z")->update(value.z);
            }

            virtual void set(const floatXYZ & value){
                CompoundNode<floatXYZ>::set(value);
                Q_EMIT onSet(value);
            }

            virtual void forceSet() {
                float x = boost::get<float>(this->findOrCreate<TypedNumericNode<float> >("x")->get());
                float y = boost::get<float>(this->findOrCreate<TypedNumericNode<float> >("y")->get());
                float z = boost::get<float>(this->findOrCreate<TypedNumericNode<float> >("z")->get());
                set(type(x, y, z));
            }

        Q_SIGNALS:
            void onUpdate(const floatXYZ value);
            void onSet(const floatXYZ value);
        };


    } //namespace gui
} // namespace cauv

#endif // GUI_COMPOUNDNODE_H

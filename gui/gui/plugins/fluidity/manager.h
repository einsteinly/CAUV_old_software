/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_F_MANAGER_H__
#define __CAUV_F_MANAGER_H__

#include <QObject>
#include <QPointF>
#include <QGraphicsSceneMouseEvent>

#include <ros/subscriber.h>

#include <stack>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <utility/bimap.h>
#include <utility/time.h>

#include <common/pipeline_model/pipeline.h>
#include <std_msgs/Bool.h>

#include <drag/nodeDragging.h>
#include <fluidity/types.h>

// - Forward Declarations in ::
class QGraphicsScene;
class QTimer;

namespace cauv{

namespace gui{
// - Forward Declarations in cauv::gui
class NodeScene;
class NodeItemModel;

namespace f{
// - Forward Declarations in cauv::gui::f
class ImageSource;

class Manager: public pipeline_model::PipelineModel,
               public QObject,
               //public DropHandlerInterface<QGraphicsItem*>,
               public boost::enable_shared_from_this<Manager>,
               public boost::noncopyable{
    Q_OBJECT
    public:
        Manager(NodeScene *scene,
                boost::shared_ptr<Node> model_parent,
                const std::string& pipeline_name);
        ~Manager();
        
        // a shared pointer to this must be held when this is called!
        void init();
        
        // deletion is normally managed by shared pointers, but teardown
        // ensures that this is removed as an observer
        void teardown();

        bool animationPermitted() const;

        void setFocusPosition(QPointF p);

        // DropHandlerInterface: create things in response to drag-drop
        // interaction with the associated scene
        //virtual bool accepts(boost::shared_ptr<cauv::gui::Node> const& node);
        //virtual QGraphicsItem* handle(boost::shared_ptr<cauv::gui::Node> const& node);
        
        QList<QGraphicsItem *> rootNodes() const;

        boost::shared_ptr<Node> model() const;
        gui::NodeItemModel * itemModel() const;
        
        // these methods are called from the messaging thread(s), they MUST NOT
        // modify anything directly: the general pattern is that these emit a
        // corresponding signal, which is connected to a slot called on the
        // main thread via a queued connection
        

    Q_SIGNALS:
        void receivedPipelineUpdatedMessage(const std_msgs::Bool::ConstPtr& up);

    public Q_SLOTS:
//         void onGraphDescription(GraphDescriptionMessage_ptr);
//         void onNodeParameters(NodeParametersMessage_ptr);
//         void onNodeAdded(NodeAddedMessage_ptr);
//         void onNodeRemoved(NodeRemovedMessage_ptr);
//         void onArcAdded(ArcAddedMessage_ptr);
//         void onArcRemoved(ArcRemovedMessage_ptr);
//         void onGuiImage(GuiImageMessage_ptr);
//         void onStatus(StatusMessage_ptr);

        /* When an arc is added: 
         * 1) GUI element emits arcRequested in response to drag & drop, or
         *    something
         *
         * 2) slot requestArc called, sends AddArc message, fills in pipeline
         *    name
         *
         * 3) onArcAddedMessage
         *
         */
//         void requestArc(NodeOutput from, NodeInput to);
//         void requestRemoveArc(NodeOutput from, NodeInput to);
//         void requestNode(NodeType::e const& type,
//                          std::vector<NodeInputArc> const& inputs = std::vector<NodeInputArc>(),
//                          std::vector<NodeOutputArc> const& outputs = std::vector<NodeOutputArc>()); 
//         void requestRemoveNode(node_id_t const& id);
        void requestRefresh();
//         void requestForceExec(node_id_t const& id);

        void pushAnimationPermittedState(bool permitted);
        void popAnimationPermittedState();

        void considerUpdatingLayout();
        
        void delayLayout();

    protected:
//         void removeNode(node_id_t const& id);
//         fnode_ptr addNode(NodeType::e const& type, node_id_t const& id);
//         fnode_ptr addNode(NodeAddedMessage_ptr);
//         void clearNodes();
    
    protected Q_SLOTS:
        void updateLayoutNow();

    private:
        template<typename message_T>
        bool _nameMatches(boost::shared_ptr<const message_T> msg);
        
        // if this is called at a high frequency animations will be disabled
        // for a while
        void _animAutoDisableTick();

        void _layoutSoonIfNothingHappens();

        void _checkAddImageSource(node_id_t);

    protected:
        NodeScene *m_scene;
        ros::NodeHandle *m_ros_node;
        ros::Subscriber m_update_sub;
        
        std::stack<bool> m_animation_permitted;
        
        QPointF m_focus_scenepos;

        TimeStamp m_last_anim_auto_disable_check;

        QTimer* m_layout_soon_timer;

        boost::shared_ptr<Node> m_model_parent;

        boost::shared_ptr<NodeItemModel> m_item_model;
};

class FocusPositionForwarder: public QObject{
     Q_OBJECT
    public:
        FocusPositionForwarder(Manager& m)
            : m_manager(m){
        }

    protected:
        bool eventFilter(QObject *obj, QEvent *event){
            if(event->type() == QEvent::GraphicsSceneMouseMove){    
                m_manager.setFocusPosition(static_cast<QGraphicsSceneMouseEvent*>(event)->scenePos());
            }
            return QObject::eventFilter(obj, event);
        }

    private:
        Manager& m_manager;
};

class AnimationPermittedState{
    public:
        AnimationPermittedState(Manager& m, bool animation_permitted)
            : m_manager(m){
            m_manager.pushAnimationPermittedState(animation_permitted);
        }
        ~AnimationPermittedState(){
            m_manager.popAnimationPermittedState();
        }
    private:
        Manager& m_manager;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_MANAGER_H__


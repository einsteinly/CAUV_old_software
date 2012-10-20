/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#ifndef __LIQUID_ITEM_FRIDGE_H___
#define __LIQUID_ITEM_FRIDGE_H___

#include <set>
#include <cassert>

#include <boost/static_assert.hpp>
#include <boost/type_traits/is_base_of.hpp>

#include <QGraphicsItem>
#include <QPainter>

namespace liquid{

/* ItemFridge overloads paint() and boundingRect() of QGraphicsItem to act as
 * if this item and all it's children (and their children, recursively) are
 * part of this item. This means that children will be cached as part of the
 * parent item (if caching is enabled).
 *
 * Use of this class is appropriate if a parent item has lots of children that
 * do not behave independently (i.e. they are not movable, and do not interact
 * with mouse events).
 *
 * If the child items are positioned far outside the bounding rect of the
 * parent using this class may reduce performance.
 *
 * Use:
 *  QGraphicsItem* myitem = new ItemFridge<MyGraphicsItem>()
 * instead of:
 *  QGraphicsItem* myitem = new MyGraphicsItem()
 *
 */
template<typename ItemT>
class ItemFridge: public ItemT{
        BOOST_STATIC_ASSERT((boost::is_base_of<QGraphicsItem,ItemT>::value));

    public:
        ItemFridge(QGraphicsItem* parent=0)
            : ItemT(parent){
            commonInit();
        }

        template<typename... Types>
        ItemFridge(Types&... a)
            : ItemT(a...){
            commonInit();
        }

        template<typename... Types>
        ItemFridge(Types const&... a)
            : ItemT(a...){
            commonInit();
        }

        // Freeze after adding children
        void freeze(){
            m_saved_itemt_has_no_contents = this->flags() & QGraphicsItem::ItemHasNoContents;
            this->setFlag(QGraphicsItem::ItemHasNoContents, false);
            this->setCacheMode(QGraphicsItem::ItemCoordinateCache);
            foreach(QGraphicsItem* child, this->childItems())
                freezeChildrenRecursive(child);
            this->prepareGeometryChange();
            m_boundingrect = boundingRectRecursive(this);
            this->update(this->boundingRect());
        }

        // Thaw if children need to be removed or modified
        void thaw(){
            this->setFlag(QGraphicsItem::ItemHasNoContents, m_saved_itemt_has_no_contents);
            foreach(QGraphicsItem* child, this->childItems())
                thawChildrenRecursive(child);
            this->update(this->boundingRect());
        }


        // - QGraphicsItem implementation
        virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){
            paintRecursive(this, painter, option, widget);
        }

        virtual QRectF boundingRect() const{
            //std::cout << "Fridge:" << this << ":boundingRect:"
            //          << m_boundingrect.x() << "," << m_boundingrect.y() << ":"
            //          << m_boundingrect.width() << "," << m_boundingrect.height() << std::endl;
            return m_boundingrect;
        }

        virtual void hoverEnterEvent(QGraphicsSceneHoverEvent* event){
            Q_UNUSED(event);
            thaw();
        }

        virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent* event){
            Q_UNUSED(event);
            freeze();
        }

    private:
        void commonInit(){
            m_saved_itemt_has_no_contents = false;
            this->setAcceptHoverEvents(true);
        }

        void paintRecursive(QGraphicsItem* item,
                            QPainter* painter,
                            const QStyleOptionGraphicsItem* option,
                            QWidget* widget){
            foreach(QGraphicsItem* child, item->childItems()){
                if((child->flags() & QGraphicsItem::ItemStacksBehindParent) && m_hidden_children.count(child)){
                    painter->save();
                    //painter->setWorldTransform(child->sceneTransform());
                    // !!! FIXME: only supporting translation
                    painter->translate(child->pos());
                    //std::cout << "paint child behind " << child << " parent="<< child->parentItem() << std::endl;
                    paintRecursive(child, painter, option, widget);
                    painter->restore();
                }
            }
            if(item == this){
                ItemT::paint(painter, option, widget);
            }else{
                item->paint(painter, option, widget);
            }
            foreach(QGraphicsItem* child, item->childItems()){
                if((!(child->flags() & QGraphicsItem::ItemStacksBehindParent)) && m_hidden_children.count(child)){
                    painter->save();
                    //painter->setWorldTransform(child->sceneTransform());
                    // !!! FIXME: only supporting translation
                    painter->translate(child->pos());
                    //std::cout << "paint child in front " << child << " parent="<< child->parentItem() << std::endl;
                    paintRecursive(child, painter, option, widget);
                    painter->restore();
                }
            }
        }

        QRectF boundingRectRecursive(QGraphicsItem const* item) const{
            // !!!FIXME doesn't take account of non-translation transformations
            QRectF r;
            if(item != this)
                r = item->boundingRect();
            else
                r = ItemT::boundingRect();

            foreach(QGraphicsItem* child, item->childItems())
                r |= boundingRectRecursive(child).translated(child->pos());
            return r;
        }

        void freezeChildrenRecursive(QGraphicsItem* of_item){
            freezeChild(of_item);
            m_saved_cache_modes[of_item] = of_item->cacheMode();
            of_item->setCacheMode(QGraphicsItem::NoCache);
            foreach(QGraphicsItem* child, of_item->childItems())
                freezeChildrenRecursive(child);
        }

        void thawChildrenRecursive(QGraphicsItem* of_item){
            thawChild(of_item);
            of_item->setCacheMode(m_saved_cache_modes[of_item]);
            foreach(QGraphicsItem* child, of_item->childItems())
                thawChildrenRecursive(child);
        }

        void freezeChild(QGraphicsItem* child){
            if(child->isVisibleTo(0)){
                m_hidden_children.insert(child);
                child->setFlag(QGraphicsItem::ItemHasNoContents, true);
            }
        }

        void thawChild(QGraphicsItem* child){
            m_hidden_children.erase(child);
            child->setFlag(QGraphicsItem::ItemHasNoContents, false);
        }

        QRectF m_boundingrect;
        std::set<QGraphicsItem*> m_hidden_children;
        std::map<QGraphicsItem*, QGraphicsItem::CacheMode> m_saved_cache_modes;
        bool m_saved_itemt_has_no_contents;
};

} // namespace liquid

#endif // ndef __LIQUID_ITEM_FRIDGE_H___

#include <QStringList>

#include "treeitem.h"

TreeItem::TreeItem(TreeItem *parent) :
        parentItem(parent)
{
    if(parent)
        parent->appendChild(this);
}

TreeItem::~TreeItem()
{
    qDeleteAll(childItems);
}

void TreeItem::appendChild(TreeItem *item)
{
    childItems.append(item);
}

TreeItem *TreeItem::child(int row)
{
    return childItems.value(row);
}

int TreeItem::childCount() const
{
    return childItems.count();
}

int TreeItem::columnCount() const
{
    return itemData.count();
}

QVariant TreeItem::data(int column) const
{
    return itemData.value(column);
}

void TreeItem::setData(int column, QVariant data)
{
    itemData[column] = data;
}

void TreeItem::appendData(QVariant data)
{
    itemData.append(data);
}

Qt::ItemFlags TreeItem::flags(int) const{
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

TreeItem *TreeItem::parent()
{
    return parentItem;
}

int TreeItem::row() const
{
    if (parentItem)
        return parentItem->childItems.indexOf(const_cast<TreeItem*>(this));

    return 0;
}

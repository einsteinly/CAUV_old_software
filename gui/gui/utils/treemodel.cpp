
 #include <QtGui>

 #include "treeitem.h"
 #include "treemodel.h"

TreeModel::TreeModel(TreeItem * root):
        rootItem(root)
 {}

 TreeModel::~TreeModel()
 {
     delete rootItem;
 }

 int TreeModel::columnCount(const QModelIndex &parent) const
 {
     if (parent.isValid())
         return static_cast<TreeItem*>(parent.internalPointer())->columnCount();
     else
         return rootItem->columnCount();
 }

 QVariant TreeModel::data(const QModelIndex &index, int role) const
 {
     if (!index.isValid())
         return QVariant();

     if (role != Qt::DisplayRole)
         return QVariant();

     TreeItem *item = static_cast<TreeItem*>(index.internalPointer());

     return item->data(index.column());
 }

 Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const
 {
     if (!index.isValid())
         return Qt::ItemIsEnabled;

     TreeItem *item = static_cast<TreeItem*>(index.internalPointer());

     return item->flags(index.column());
 }

 QVariant TreeModel::headerData(int section, Qt::Orientation orientation,
                                int role) const
 {
     if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
         return rootItem->data(section);

     return QVariant();
 }

 QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent)
             const
 {
     TreeItem *parentItem;

     if (!parent.isValid())
         parentItem = rootItem;
     else
         parentItem = static_cast<TreeItem*>(parent.internalPointer());

     TreeItem *childItem = parentItem->child(row);
     if (childItem)
         return createIndex(row, column, childItem);
     else
         return QModelIndex();
 }

 QModelIndex TreeModel::parent(const QModelIndex &index) const
 {
     if (!index.isValid())
         return QModelIndex();

     TreeItem *childItem = static_cast<TreeItem*>(index.internalPointer());
     TreeItem *parentItem = childItem->parent();

     if (parentItem == rootItem)
         return QModelIndex();

     return createIndex(parentItem->row(), 0, parentItem);
 }

 int TreeModel::rowCount(const QModelIndex &parent) const
 {
     TreeItem *parentItem;

     if (!parent.isValid())
         parentItem = rootItem;
     else
         parentItem = static_cast<TreeItem*>(parent.internalPointer());

     return parentItem->childCount();
 }

 TreeItem * TreeModel::root(){
     return rootItem;
 }

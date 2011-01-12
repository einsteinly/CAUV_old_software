 #ifndef TREEITEM_H
 #define TREEITEM_H

 #include <QList>
 #include <QVariant>

 class TreeItem
 {
 public:
     TreeItem(TreeItem *parent = 0);
     ~TreeItem();


     virtual TreeItem *child(int row);
     virtual int childCount() const;
     virtual int columnCount() const;
     virtual QVariant data(int column) const;
     virtual int row() const;
     virtual Qt::ItemFlags flags(int column) const;
     virtual TreeItem *parent();
     virtual void setData(int column, QVariant data);
     virtual void appendData(QVariant data);

 private:
     QList<TreeItem*> childItems;
     QList<QVariant> itemData;
     TreeItem *parentItem;
     void appendChild(TreeItem *child);
 };

 #endif

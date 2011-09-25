#ifndef __LIQUID_EPHEMERAL_ARC_END__
#define __LIQUID_EPHEMERAL_ARC_END__

class EphemeralArcEnd: public QGraphicsObject{
    Q_OBJECT
    public:
        EphemeralArcEnd()
            :{
            setFlag(ItemIsMovable);
            connect(this, SIGNAL(xChanged()), this, SIGNAL(geometryChanged));
            connect(this, SIGNAL(yChanged()), this, SIGNAL(geometryChanged));
        }

    public Q_SIGNALS:
        geometryChanged();

    protected:
        void setFill(bool pressed);
        
        QGraphicsPolygonItem *m_back_poly;
        QGraphicsPolygonItem *m_front_poly;
        
        liquid::ArcStyle const& m_style;
};

#endif // ndef __LIQUID_EPHEMERAL_ARC_END__

#include "HoughTransformer.h"

#include <primitives/NormalPlane.h>

HoughTransformer::HoughTransformer()
{
    //TODO: actually implement populating the map
    shapes = QMap<PrimitiveType, PrimitiveShape>();
    // shapes.insert(PrimitiveType::NORM/ALPLANE, NormalPlane)
}


QList<PrimitiveShape> HoughTransformer::applyTransform(QVector<QVector3D> points, QList<PrimitiveType> types) const
{
    QList<PrimitiveShape> output;
    for (PrimitiveType type : types)
    {
        PrimitiveShape* shape = getShape(type);
        shape->getBestFit(points);
    }
    return output;
}

PrimitiveShape* HoughTransformer::getShape(PrimitiveType const type) const
{
    // TODO: adapt to multiple shapes
    return new NormalPlane();
}

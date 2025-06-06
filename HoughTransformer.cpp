#include "HoughTransformer.h"

HoughTransformer::HoughTransformer() {}


QList<PrimitiveShape> HoughTransformer::applyTransform(QVector<QVector3D> points, QList<PrimitiveType> types) const
{
    for (PrimitiveType type : types)
    {
        // Apply hough transform for type

        // filter votes

        // pick best one(s)?
    }
}

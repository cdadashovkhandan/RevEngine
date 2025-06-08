#include "HoughTransformer.h"

HoughTransformer::HoughTransformer() {}


QList<PrimitiveShape> HoughTransformer::applyTransform(QVector<QVector3D> points, QList<PrimitiveType> types) const
{
    QList<PrimitiveShape> output;
    for (PrimitiveType type : types)
    {
        // Apply hough transform for type

        // filter votes

        // pick best one(s)?

        for (QVector3D point : points)
        {
            // go through all possible parameter combos and see if they satisfy the equation


        }
    }
    return output;
}

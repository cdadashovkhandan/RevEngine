#include "HoughTransformer.h"

HoughTransformer::HoughTransformer() {}


QList<PrimitiveShape> HoughTransformer::applyTransform(QVector<QVector3D> points, QList<PrimitiveType> types) const
{
    QList<PrimitiveShape> output;
    for (PrimitiveType type : types)
    {
        // Instantiate//Retrieve shape by enum





        // Build hough space



        // vote for hough space



        for (QVector3D point : points)
        {
            // go through all possible parameter combos and see if they satisfy the equation

        }
    }
    return output;
}

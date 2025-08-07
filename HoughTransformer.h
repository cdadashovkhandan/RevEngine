#ifndef HOUGHTRANSFORMER_H
#define HOUGHTRANSFORMER_H

#include "primitives/PrimitiveShape.h"
#include <QVector>
#include <QVector3D>
#include <QMap>
#include <pcl/impl/point_types.hpp>
class HoughTransformer
{
public:
    HoughTransformer();



    // QList<PrimitiveShape*> applyTransform(QVector<QVector3D> const points, QList<PrimitiveType> const types) const;

    //TODO: might want to make static?
    //TODO: template might be completely unnecessary, could just feed shape in directly.
    std::vector<float> getBestFit(PrimitiveShape* const primitiveType, std::vector<pcl::PointXYZ> const points) const;
};

#endif // HOUGHTRANSFORMER_H

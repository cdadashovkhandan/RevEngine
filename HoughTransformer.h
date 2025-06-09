#ifndef HOUGHTRANSFORMER_H
#define HOUGHTRANSFORMER_H

#include "primitives/PrimitiveShape.h"
#include "primitives/PrimitiveType.h"
#include <QVector>
#include <QVector3D>
#include <QMap>
class HoughTransformer
{
public:
    HoughTransformer();

    //QMap<PrimitiveType, PrimitiveShape>

    QList<PrimitiveShape> applyTransform(QVector<QVector3D> points, QList<PrimitiveType> types) const;
};

#endif // HOUGHTRANSFORMER_H

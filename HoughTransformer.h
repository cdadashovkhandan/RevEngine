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

    //TODO: rethink this. Template magic can probably remove the need for this and the enum entirely.
    QMap<PrimitiveType, PrimitiveShape> shapes;

    QList<PrimitiveShape> applyTransform(QVector<QVector3D> const points, QList<PrimitiveType> const types) const;
private:

    PrimitiveShape* getShape(PrimitiveType const type) const;
};

#endif // HOUGHTRANSFORMER_H

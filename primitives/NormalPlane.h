#ifndef NORMALPLANE_H
#define NORMALPLANE_H

#include "PrimitiveShape.h"

class NormalPlane : public PrimitiveShape
{

public:
    QVector<ParamPair> buildParameters(QVector<QVector3D> const points) const override;
    bool isIntersecting(QVector3D const point, QVector<float> const params) const override;

    NormalPlane();
};

#endif // NORMALPLANE_H

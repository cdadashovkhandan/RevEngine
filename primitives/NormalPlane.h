#ifndef NORMALPLANE_H
#define NORMALPLANE_H

#include "PrimitiveShape.h"

class NormalPlane : public PrimitiveShape
{

protected:
    QVector<ParamPair> buildParameters() override;
    bool isIntersecting(QVector3D point, QVector<float> params) const override;
public:
    void getBestFit(QVector<QVector3D> points) override;
    NormalPlane();
};

#endif // NORMALPLANE_H

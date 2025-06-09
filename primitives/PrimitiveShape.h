#ifndef PRIMITIVESHAPE_H
#define PRIMITIVESHAPE_H

#include <QVector3D>
#include <QVector>
#include <QPair>

using ParamPair = QPair<unsigned int, QVector<float>>;

class PrimitiveShape
{
//TODO: Separate hough transform logic from shape instances
public:
    PrimitiveShape();

    QVector3D position;
    QVector3D orientation;

    // TODO: return type probably wrong
    virtual void getBestFit(QVector<QVector3D> points) = 0;
    // Build the initial matrix of parameters and flatten them to a 2D vector of floats
protected:
    virtual QVector<ParamPair> buildParameters() = 0;
    virtual bool isIntersecting(QVector3D point, QVector<float> params) const = 0;
};

#endif // PRIMITIVESHAPE_H

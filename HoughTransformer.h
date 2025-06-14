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


    // QList<PrimitiveShape*> applyTransform(QVector<QVector3D> const points, QList<PrimitiveType> const types) const;

    //TODO: might want to make static?
    //TODO: template might be completely unnecessary, could just feed shape in directly.
    template <typename Shape>
    QVector<float> getBestFit(QVector<QVector3D> const points) const
    {
        Shape shape;
        QVector<ParamPair> params = shape.buildParameters();

        // Value for best fit chosen by having max votes
        // TODO: this is naive.
        ParamPair* bestFit = nullptr;

        for (QVector3D point : points)
        {
            for (ParamPair& pair : params)
            {
                if (shape.isIntersecting(point, pair.second))
                {
                    ++pair.first;

                    // Check if this is the best fit.
                    if (bestFit == nullptr || pair.first > bestFit->first)
                        bestFit = &pair;
                }
            }
        }
        return bestFit->second;
    }
private:
    //TODO: might be useless
    PrimitiveShape* getShape(PrimitiveType const type) const;
};

#endif // HOUGHTRANSFORMER_H

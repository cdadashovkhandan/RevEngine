#ifndef HOUGHTRANSFORMER_H
#define HOUGHTRANSFORMER_H

#include "primitives/PrimitiveShape.h"
#include "primitives/PrimitiveType.h"
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
    template <typename Shape>
    std::vector<float> getBestFit(std::vector<pcl::PointXYZ> const points) const
    {
        Shape shape;

        //TODO: pick a better place for this
        float maxMagnitude = std::max_element(points.begin(),
                                              points.end(),
                                              [](pcl::PointXYZ const a, pcl::PointXYZ const b)
                                                {
                                                    return a.getVector3fMap().norm() < b.getVector3fMap().norm();
                                              })->getVector3fMap().norm();

        std::vector<ParamPair> params = shape.buildParameters(points, maxMagnitude);

        // Value for best fit chosen by having max votes
        // TODO: this is naive.
        ParamPair* bestFit = nullptr;

        for (pcl::PointXYZ const point : points)
        {
            for (ParamPair& pair : params)
            {
                if (shape.isIntersecting(point, pair.second, maxMagnitude))
                {
                    ++pair.first;

                    // Check if this is the best fit.
                    if (bestFit == nullptr || pair.first > bestFit->first)
                        bestFit = &pair;
                }
            }
        }

        // If no best fit was found
        if (bestFit == nullptr)
        {
            qDebug("Best Hough Transform fit not found");
            return std::vector<float>({0,0,0});
            //throw; //TODO: proper error handling
        }


        return bestFit->second;
    }
private:
    //TODO: might be useless
    PrimitiveShape* getShape(PrimitiveType const type) const;
};

#endif // HOUGHTRANSFORMER_H

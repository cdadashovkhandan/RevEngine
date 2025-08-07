#include "PrimitiveShape.h"

std::vector<float> PrimitiveShape::getBestFit(const std::vector<pcl::PointXYZ> points) const
{
    //TODO: pick a better place for this
    float maxMagnitude = std::max_element(points.begin(),
                                          points.end(),
                                          [](pcl::PointXYZ const a, pcl::PointXYZ const b)
                                          {
                                              return a.getVector3fMap().norm() < b.getVector3fMap().norm();
                                          })->getVector3fMap().norm();

    std::vector<ParamPair> params = buildParameters(points, maxMagnitude);

    // Value for best fit chosen by having max votes
    // TODO: this is naive.
    ParamPair* bestFit = nullptr;

    for (pcl::PointXYZ const point : points)
    {
        for (ParamPair& pair : params)
        {
            if (isIntersecting(point, pair.second, maxMagnitude))
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

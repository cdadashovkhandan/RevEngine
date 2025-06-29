#ifndef NORMALPLANE_H
#define NORMALPLANE_H

#include "PrimitiveShape.h"

class NormalPlane : public PrimitiveShape
{

public:
    std::vector<ParamPair> buildParameters(std::vector<pcl::PointXYZ> const points, float const maxMagnitude) const override;
    bool isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const override;

    NormalPlane();
};

#endif // NORMALPLANE_H

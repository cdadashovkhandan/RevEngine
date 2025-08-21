#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <Eigen/Dense>

struct BoundingBox
{
    BoundingBox(Eigen::Vector3f min, Eigen::Vector3f max)
    {
        this->min = min;
        this->max = max;
    }

    Eigen::Vector3f min;
    Eigen::Vector3f max;
};

#endif // BOUNDINGBOX_H

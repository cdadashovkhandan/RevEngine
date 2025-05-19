#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "QVector"
#include <QVector3D>


// Raw PointCloud data.
//TODO: should this be a struct or class?
struct PointCloud
{
public:
    PointCloud();
    QVector<QVector3D> points { };
};

//TODO: parser based on this:
// https://github.com/chiararomanengo/Fit4CAD/blob/main/dataset/

#endif // POINTCLOUD_H

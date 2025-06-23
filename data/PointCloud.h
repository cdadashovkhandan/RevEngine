#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "QVector"
#include <QVector3D>


// Raw PointCloud data.
//TODO: should this be a struct or class?

struct PointCloud
{
public:
    [[deprecated]]
    PointCloud(QVector<QVector3D>);
    [[deprecated]]
    QVector<QVector3D> points { };
};



#endif // POINTCLOUD_H

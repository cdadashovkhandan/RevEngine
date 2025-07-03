#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include "CADConverter.h"
#include "data/Model.h"
#include "Settings.h"
#include <QString>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// Handle creation and storage of Models
class ModelManager
{

private:
    QVector<Model*> models; // make public?
    PointCloud::Ptr parsePointCloud(QString fileName) const;
    CADConverter* cadConverter;
public:
    ModelManager(Settings* s);

    Model* getActiveModel() const;

    Settings* settings;
    Model* createModel(QString fileName);
    void recalculateClusters(Model* model);
    //TODO: return type is questionable
    Model* generateMesh(Model& model) const;
};

#endif // MODELMANAGER_H

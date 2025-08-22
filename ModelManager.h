#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include "CADConverter.h"
#include "data/Model.h"
#include "Settings.h"
#include "data/ModelStatus.h"
#include <QString>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// Handle creation and storage of Models
class ModelManager
{

private:
    QVector<Model*> models;
    PointCloud::Ptr parsePointCloud(QString fileName) const;
    CADConverter* cadConverter;
public:
    ModelManager(Settings* s);
    ~ModelManager();

    ModelStatus modelStatus = ModelStatus::EMPTY;

    Model* getActiveModel() const;

    Settings* settings;
    Model* createModel(QString fileName);
    void recalculateClusters(Model* model);
    void recalculateNormals(Model* model);
    void recalculateDownsample(Model *model);
    Model* preprocessModel(Model& model);
    Model* recognizeShapes(Model &model);
};

#endif // MODELMANAGER_H

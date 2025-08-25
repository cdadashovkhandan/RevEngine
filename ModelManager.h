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


public:
    ModelManager(Settings* s);
    ~ModelManager();
    Model* model = nullptr;

    ModelStatus modelStatus = ModelStatus::EMPTY;

    Settings* settings;
    Model* createModel(QString fileName);

    // Individual steps.
    void recalculateClusters() const;
    void recalculateNormals() const;
    void recalculateDownsample() const;

    // Main controls.
    void preprocessModel();
    void recognizeShapes();
    void finalizeModel();

private:
    PointCloud::Ptr parsePointCloud(QString fileName) const;
    CADConverter* cadConverter;
};

#endif // MODELMANAGER_H

#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include "data/Model.h"
#include "Settings.h"
#include <QString>

// Handle creation and storage of Models
class ModelManager
{

private:
    QVector<Model> models; // make public?
    PointCloud* parsePointCloud(QString fileName) const;
public:
    ModelManager();

    Settings* settings;
    Model* createModel(QString fileName);
};

#endif // MODELMANAGER_H

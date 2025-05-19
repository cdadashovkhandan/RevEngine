#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include "data/Model.h"
#include <QString>

// Handle creation and storage of Models
class ModelManager
{

private:
    QVector<Model> models; // make public?
    PointCloud parsePointCloud(QString filename);
public:
    ModelManager();
    Model createModel(QString filename);
};

#endif // MODELMANAGER_H

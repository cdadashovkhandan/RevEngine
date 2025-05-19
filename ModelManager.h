#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include "data/Model.h"
#include <QString>

// Handle creation and storage of Models
class ModelManager
{

private:
    QVector<Model> models; // make public?
    PointCloud parsePointCloud(QString fileName);
public:
    ModelManager();
    Model createModel(QString fileName);
};

#endif // MODELMANAGER_H

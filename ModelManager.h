#ifndef MODELMANAGER_H
#define MODELMANAGER_H

#include "CADConverter.h"
#include "data/Model.h"
#include "Settings.h"
#include <QString>

// Handle creation and storage of Models
class ModelManager
{

private:
    QVector<Model*> models; // make public?
    PointCloud* parsePointCloud(QString fileName) const;
    CADConverter* cadConverter;
public:
    ModelManager();

    Model* getActiveModel() const;

    Settings* settings;
    Model* createModel(QString fileName);
    //TODO: return type is questionable
    Model* generateMesh(Model& model) const;
};

#endif // MODELMANAGER_H

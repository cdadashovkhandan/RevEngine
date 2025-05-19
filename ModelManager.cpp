#include "ModelManager.h"

#include <QDir>
#include <QFile>

ModelManager::ModelManager() {}

PointCloud ModelManager::parsePointCloud(QString fileName)
{
    QFile cloudFile(fileName);
    Mesh newMesh;
    QVector<QVector3D> verts{};
    if (cloudFile.open(QIODevice::ReadOnly))
    {
        QTextStream fileText(&cloudFile);
        while (!fileText.atEnd())
        {
            QStringList line = fileText.readLine().split(",");
        }
    }

    PointCloud pointCloud(verts);
    return pointCloud;
}


Model ModelManager::createModel(QString filename)
{
    PointCloud pointCloud = parsePointCloud(filename);
    Model model(pointCloud);
    models.push_back(model);
}


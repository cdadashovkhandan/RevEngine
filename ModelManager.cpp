#include "ModelManager.h"

#include <QDir>
#include <QFile>

ModelManager::ModelManager() {}

PointCloud* ModelManager::parsePointCloud(QString fileName) const
{
    QFile cloudFile(fileName);
    Mesh newMesh;
    QVector<QVector3D> verts{};
    if (cloudFile.open(QIODevice::ReadOnly))
    {
        QTextStream fileText(&cloudFile);
        while (!fileText.atEnd())
        {
            QString rawLine = fileText.readLine();
            rawLine = rawLine.trimmed().replace("[", "").replace("]",""); // remove excess chars
            QStringList line = rawLine.split(",");

            if (line.size() == 1) // Either a beginning or end line
                continue;

            QVector3D newPoint(line[0].toFloat(), line[1].toFloat(),line[2].toFloat());

            verts.push_back(newPoint);
        }
    }

    return new PointCloud(verts);
}


Model* ModelManager::createModel(QString filename)
{
    PointCloud* pointCloud = parsePointCloud(filename);
    Model model(pointCloud);
    models.push_back(model);
    return &models.last();
}


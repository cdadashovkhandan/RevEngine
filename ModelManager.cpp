#include "ModelManager.h"

#include <QDir>
#include <QFile>

ModelManager::ModelManager()
{
    cadConverter = new CADConverter();
}

/**
 * @brief ModelManager::parsePointCloud Parse a point cloud from file.
 * @param fileName
 * @return
 */
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

            newPoint /= 100.0f;

            verts.push_back(newPoint);
        }
    }

    return new PointCloud(verts);
}

/**
 * @brief ModelManager::createModel Parse a point cloud from a File and generate a point cloud from it
 * @param filename
 * @return
 */
Model* ModelManager::createModel(QString filename)
{
    PointCloud* pointCloud = parsePointCloud(filename);
    Model model(pointCloud);
    models.push_back(model);
    return &models.last();
}

/**
 * @brief ModelManager::generateMesh Generate a B-Rep representation out of a Model's Point Cloud.
 * @param model
 * @return
 */
Mesh *ModelManager::generateMesh(Model *model) const
{
    cadConverter->convertModel(model);
}


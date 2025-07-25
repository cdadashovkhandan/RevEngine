#include "ModelManager.h"

#include <QDir>
#include <QFile>

ModelManager::ModelManager(Settings* s)
{
    settings = s;
    cadConverter = new CADConverter(s);
}

Model* ModelManager::getActiveModel() const
{
    return models.last();
}

/**5
 * @brief ModelManager::parsePointCloud Parse a point cloud from file.
 * @param fileName
 * @return
 */
PointCloud::Ptr ModelManager::parsePointCloud(QString fileName) const
{
    QFile cloudFile(fileName);
    Mesh newMesh;

    PointCloud::Ptr cloud(new PointCloud());
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


            //TODO: make this neat
            pcl::PointXYZ newPoint(line[0].toFloat(), line[1].toFloat(), line[2].toFloat());

            cloud->push_back(newPoint);
        }
    }
    return cloud;
}

/**
 * @brief ModelManager::createModel Parse a point cloud from a File and generate a point cloud from it
 * @param filename
 * @return
 */
Model* ModelManager::createModel(QString filename)
{
    PointCloud::Ptr pointCloud = parsePointCloud(filename);
    Model* model = new Model(pointCloud);
    models.append(model);
    return models.last();
}

void ModelManager::recalculateClusters(Model *model)
{
    model->pointIndices = cadConverter->cluster(model->pointCloud);
}

/**
 * @brief ModelManager::generateMesh Generate a B-Rep representation out of a Model's Point Cloud.
 * @param model
 * @return
 */
Model *ModelManager::generateMesh(Model& model) const
{
    cadConverter->convertModel(model);

    return &model;
}


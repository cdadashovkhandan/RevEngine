#include "ModelManager.h"

#include <QDir>
#include <QException>
#include <QFile>

#include "exceptions/FileReadException.h"

ModelManager::ModelManager(Settings* s)
{
    settings = s;
    cadConverter = new CADConverter(s);
}

/**
 * @brief ModelManager::getActiveModel Get the currently active model.
 * @return The active model or nullptr if there isn't one.
 */
Model* ModelManager::getActiveModel() const
{
    return models.isEmpty() ? nullptr
                            : models.last();
}

/**
 * @brief ModelManager::parsePointCloud Parse a point cloud from file.
 * @param fileName
 * @return
 */
PointCloud::Ptr ModelManager::parsePointCloud(QString fileName) const
{
    QFile cloudFile(fileName);
    PointCloud::Ptr cloud(new PointCloud());
    if (cloudFile.open(QIODevice::ReadOnly))
    {
        QTextStream fileText(&cloudFile);
        while (!fileText.atEnd())
        {
            QString rawLine = fileText.readLine();
            rawLine = rawLine.trimmed().replace("[", "").replace("]",""); // remove excess chars
            QStringList line = rawLine.split(",");

            if (line.size() < 3) // Either a beginning or end line
                continue;

            pcl::PointXYZ newPoint(line[0].toFloat(),
                                   line[1].toFloat(),
                                   line[2].toFloat());

            cloud->push_back(newPoint);
        }
    }

    if (cloud->points.size() < 1)
        throw FileReadException();

    return cloud;
}

/**
 * @brief ModelManager::createModel Parse a point cloud from a File and generate a point cloud from it
 * @param filename
 * @return
 */
Model* ModelManager::createModel(QString filename)
{
    try
    {
        PointCloud::Ptr pointCloud = parsePointCloud(filename);
        Model* model = new Model(pointCloud);
        models.append(model);
        return models.last();
    }
    catch (FileReadException& e)
    {
        throw e; // re-throw to UI level.
    }
}

/**
 * @brief ModelManager::recalculateClusters Recalculate all the clusters in the model.
 * @param model
 */
void ModelManager::recalculateClusters(Model *model)
{
    model->pointIndices = cadConverter->cluster(model->pointCloud);
}

/**
 * @brief ModelManager::recalculateNormals Recalculate all the normals in the model.
 * @param model
 */
void ModelManager::recalculateNormals(Model *model)
{
    model->normals = settings->highPrecisionNormals
        ? cadConverter->getNormals(model->pointCloud)
        : cadConverter->getNormals(model->pointCloudDownsampled);
}

/**
 * @brief ModelManager::generateMesh Generate a B-Rep representation out of a Model's Point Cloud.
 * @param model
 * @return
 */
Model *ModelManager::preprocessModel(Model& model) const
{
    cadConverter->preprocess(model);

    return &model;
}

/**
 * @brief ModelManager::generateMesh Generate a B-Rep representation out of a Model's Point Cloud.
 * @param model
 * @return
 */
Model *ModelManager::recognizeShapes(Model& model) const
{
    cadConverter->recognizeShapes(model);

    return &model;
}


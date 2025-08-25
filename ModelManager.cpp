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
ModelManager::~ModelManager()
{
    delete model;
}

/**
 * @brief ModelManager::parsePointCloud Parse a text file of point cloud points.
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
        model = new Model(pointCloud);
        modelStatus = ModelStatus::RAW;
        return model;
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
void ModelManager::recalculateClusters() const
{
    PointCloud::Ptr cloudPtr = settings->useDownsampledVersion
                                   ? model->pointCloudDownsampled
                                   : model->pointCloud;

    model->clusterIndices = cadConverter->cluster(cloudPtr, false);
}

/**
 * @brief ModelManager::recalculateNormals Recalculate all the normals in the model.
 * @param model
 */
void ModelManager::recalculateNormals() const
{
    model->normals = settings->highPrecisionNormals
        ? cadConverter->getNormals(model->pointCloud)
        : cadConverter->getNormals(model->pointCloudDownsampled);
}

/**
 * @brief ModelManager::recalculateDownsample Downsample the point cloud again.
 * @param model
 */
void ModelManager::recalculateDownsample() const
{
    cadConverter->downsample(model->pointCloud, model->pointCloudDownsampled, model->scaleFactor);
}

/**
 * @brief ModelManager::preprocessModel Execute the preprocessing stage for a model.
 * @param model
 * @return
 */
void ModelManager::preprocessModel()
{
    cadConverter->preprocess(*model);
    modelStatus = ModelStatus::PREPROCESSED;
}

/**
 * @brief ModelManager::recognizeShapes Execute the shape recognition stage for a model.
 * @param model
 * @return
 */
void ModelManager::recognizeShapes()
{
    cadConverter->recognizeShapes(*model);
    modelStatus = ModelStatus::ANALYZED;
}

/**
 * @brief ModelManager::finalizeModel Reverse all the transformation operations to bring the shape model to its original state.
 */
void ModelManager::finalizeModel()
{
    cadConverter->finalize(*model);
    modelStatus = ModelStatus::FINALIZED;
}


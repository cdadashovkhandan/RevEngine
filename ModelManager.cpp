#include "ModelManager.h"

#include <QDir>
#include <QException>
#include <QFile>

#include "exceptions/FileReadException.h"

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
    qDeleteAll(models);
    models.clear();
}

/**
 * @brief ModelManager::parsePointCloud Parse a point cloud from file.
 * @param fileName
 * @return
 */
Model* ModelManager::getActiveModel() const
{
    return models.isEmpty() ? nullptr
                            : models.last();
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
        modelStatus = ModelStatus::RAW;
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
    PointCloud::Ptr cloudPtr = settings->useDownsampledVersion
                                   ? model->pointCloudDownsampled
                                   : model->pointCloud;

    model->clusterIndices = cadConverter->cluster(cloudPtr, false);
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
 * @brief ModelManager::recalculateDownsample Downsample the point cloud again.
 * @param model
 */
void ModelManager::recalculateDownsample(Model *model)
{
    cadConverter->downsample(model->pointCloud, model->pointCloudDownsampled, model->scaleFactor);
}

/**
 * @brief ModelManager::preprocessModel Execute the preprocessing stage for a model.
 * @param model
 * @return
 */
Model *ModelManager::preprocessModel(Model& model)
{
    cadConverter->preprocess(model);
    modelStatus = ModelStatus::PREPROCESSED;

    return &model;
}

/**
 * @brief ModelManager::recognizeShapes Execute the shape recognition stage for a model.
 * @param model
 * @return
 */
Model *ModelManager::recognizeShapes(Model& model)
{
    cadConverter->recognizeShapes(model);
    modelStatus = ModelStatus::ANALYZED;

    return &model;
}

Model* ModelManager::finalizeModel(Model &model)
{
    cadConverter->finalize(model);
    modelStatus = ModelStatus::FINALIZED;

    return &model;
}


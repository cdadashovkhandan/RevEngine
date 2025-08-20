#include "CADConverter.h"
#include "dbscan/dbscan.hpp"

#include <primitives/Plane.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include "Util.h"

CADConverter::CADConverter(Settings* s)
{
    settings = s;
}

/**
 * @brief preprocess Apply pre-processing steps to Point Cloud (downsampling, normal alignment, clustering)
 * @param model
 * @return
 */
Model* CADConverter::preprocess(Model& model) const
{
    qDebug("Begin preprocessing...");

    PointCloud::Ptr cloudPtr = model.pointCloud;
    PointCloud::Ptr cloudPtrDownsampled(new PointCloud());

    downsample(cloudPtr, cloudPtrDownsampled);

    model.pointCloudDownsampled = cloudPtrDownsampled;

    qDebug("Centering Point Cloud...");

    Eigen::Matrix<float, 4, 1> centroid;
    int result = pcl::compute3DCentroid(*cloudPtrDownsampled, centroid);

    if (result != 0)
    {
        Eigen::Matrix4f tMatrix = Eigen::Matrix4f::Identity();
        tMatrix(0, 3) = -centroid.x();
        tMatrix(1, 3) = -centroid.y();
        tMatrix(2, 3) = -centroid.z();

        pcl::transformPointCloud(*cloudPtr, *cloudPtr, tMatrix);
        // Transform downsampled version as well for displaying normals.
        pcl::transformPointCloud(*cloudPtrDownsampled, *cloudPtrDownsampled, tMatrix);
        qDebug("Centering successful");
    }
    else
        qWarning("Centering failed, continuing...");


    model.normals = settings->highPrecisionNormals
            ? getNormals(cloudPtr)
            : getNormals(cloudPtrDownsampled);

    // if (model.normals->size() > 0)
    // {
    //     alignCloudWithZAxis(cloudPtr, *model.normals);
    // }

    shrink(cloudPtr);
    shrink(cloudPtrDownsampled);

    // temp code to get indices of all points
    model.pointIndices = cluster(cloudPtr); // TODO: temporary!

    // model.pointIndices = cluster(cloudPtr);
    qDebug("Preprocessing completed.");
    return &model;
}

/**
 * @brief CADConverter::recognizeShapes Attempt to recognize all the primitives present in the Model
 * @param model
 * @return
 */
Model* CADConverter::recognizeShapes(Model& model) const
{
    qDebug("Begin shape recognition...");
    PointCloud::Ptr cloudPtr = model.pointCloud;

    // Recognize shapes for each family
    std::vector<PrimitiveShape*> shapeCandidates;
    model.shapes = new std::vector<PrimitiveShape*>();
    if (model.pointIndices->size() > 0) // TODO: this is temporary. Should be a for loop going through clusters, inside the for loop below
    {
        for (std::pair<const PrimitiveType, bool> pair : settings->primitiveTypes)
        {
            if (pair.second) // The primitive is active
            {
                PrimitiveShape* shape = getShape(pair.first);

                std::vector<float> params = shape->getBestFit(cloudPtr, model.pointIndices->at(0));

                shapeCandidates.push_back(shape);

                shape->calculateMFE(cloudPtr);
                model.shapes->push_back(shape); // TODO: temporary

                qDebug() << "Shape found. Indices: " << shape->pointIndices->indices.size() << " Params: " << shape->parameters << "MFE: " << shape->mfe;
            }
        }
    }

    qDebug("Shape recognition completed.");
    return &model;
}

/**
 * @brief CADConverter::shrink Scale a point cloud down to a unit cube
 * @param cloud
 */
void CADConverter::shrink(PointCloud::Ptr cloud) const
{
    qDebug("Downsizing pointcloud to unit cube.");
    float scaleFactor = 0.01f;
    // Find the scaling factor.
    Eigen::Vector3f maxPoint(0,0,0);
    Eigen::Vector3f minPoint(0,0,0);
    // Get min and max values of each axis.
    Util::getMinMax(cloud->points, minPoint, maxPoint);

    Eigen::Vector3f difference = (maxPoint - minPoint).cwiseAbs();
    float maxRange = qMax(qMax(difference.x(), difference.y()), difference.z());

    //TODO: unused
    size_t index = (maxRange == difference.x()) ? 0 :
                    (maxRange == difference.y()) ? 1 :
                    2;


    scaleFactor = 1.0f / maxRange;
    qDebug() << "Detected scale factor: " << scaleFactor;
    Eigen::Transform<float, 3, Eigen::Affine> tMatrix =
        Eigen::Transform<float, 3, Eigen::Affine>{Eigen::Transform<float, 3, Eigen::Affine>::Identity()}
            .scale(scaleFactor);

    pcl::transformPointCloud(*cloud, *cloud, tMatrix);

    qDebug("Downsizing complete.");

    // For debug only:

    Eigen::Vector3f newMaxPoint(0,0,0);
    Eigen::Vector3f newMinPoint(0,0,0);

    Util::getMinMax(cloud->points, newMinPoint, newMaxPoint);

    // Eigen::Vector3f ranges = newMaxPoint - newMinPoint;

    qDebug() << "New min point: (" << newMinPoint.x() << ", " <<  newMinPoint.y() << ", " << newMinPoint.z() << ")";
    qDebug() << "New max point: (" << newMaxPoint.x() << ", " <<  newMaxPoint.y() << ", " << newMaxPoint.z() << ")";
}

/**
 * @brief CADConverter::downsample Perform voxel-grid downsampling on an input point cloud
 * @param input Input point cloud pointer
 * @param target target point cloud pointer (NB: input and target can be the same)
 */
void CADConverter::downsample(PointCloud::Ptr input, PointCloud::Ptr target) const
{
    qDebug("Creating downsampled copy...");
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;

    float downSampleFactor = settings->downSampleFactor;

    voxelGrid.setInputCloud(input);
    voxelGrid.setLeafSize(downSampleFactor, downSampleFactor, downSampleFactor);
    voxelGrid.filter(*target);

    qDebug() << "Downsampling complete. Reduced from " << input->size() << " to " << target->size() << " points.";
}

/**
 * @brief cluster Perform a clustering algorithm to group points together
 * @param input the Point Cloud to be clustered
 * @return vector of indices for each cluster
 */
std::vector<pcl::PointIndices::Ptr>* CADConverter::cluster(PointCloud::Ptr input) const
{
    bool useRansac = false; // TODO: move to settings or create proper condition

    std::vector<pcl::PointIndices::Ptr>* cluster_indices;
    if (useRansac || settings->forceRansac) // Partially adapted from https://stackoverflow.com/questions/46826720/pclransac-segmentation-get-all-planes-in-cloud
    {
        //TODO: maybe move to a separate method ?
        qDebug("Clustering with RANSAC...");

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (settings->distanceThreshold);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        PointCloud::Ptr cloudCopy(new PointCloud());
        pcl::copyPointCloud(*input, *cloudCopy);
        int original_size(cloudCopy->height*cloudCopy->width);

        // TODO: put into Settings
        float min_percentage = 2.0f;
        cluster_indices = new std::vector<pcl::PointIndices::Ptr>();

        while (cloudCopy->height * cloudCopy->width > original_size * min_percentage/100.0f)
        {
            seg.setInputCloud (cloudCopy);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            seg.segment (*inliers, *coefficients);
            cluster_indices->push_back(inliers);

            // Extract inliers
            extract.setInputCloud(cloudCopy);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloudCopy);
        }
    }
    else
    {
        qDebug("Clustering with DBSCAN...");
        cluster_indices = dbscan(input->points, settings->distanceThreshold, settings->minClusterSize);
    }

    qDebug() << "Clusters found: " << cluster_indices->size();

    return cluster_indices;
}

/**
 * @brief CADConverter::alignCloudWithZAxis Rotate the entire point cloud to be parallel to the Z axis.
 * @param cloudPtr
 * @param normals
 */
void CADConverter::alignCloudWithZAxis(PointCloud::Ptr cloudPtr, std::vector<Eigen::Vector3f> const normals) const
{
    // vote for major normal direction
    qDebug("Aligning with z-axis...");
    Eigen::Vector3f average = std::accumulate(normals.begin(), normals.end(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    average /= float(normals.size());
    // Align major normal direction with z-axis

    Eigen::Vector3f zAxis(0.0f, 0.0f, 1.0f);
    Eigen::Matrix4f rotationMatrix = buildRotationMatrix(zAxis, average.normalized());

    pcl::transformPointCloud(*cloudPtr, *cloudPtr, rotationMatrix);
    qDebug("Alignment complete");
}

Eigen::Matrix4f CADConverter::buildRotationMatrix(Eigen::Vector3f const target, Eigen::Vector3f const source) const
{
    //TODO: proper variable names, this ain't MATLAB
    Eigen::Matrix3f GG{
        {target.dot(source),             -target.cross(source).norm(),   0.0f},
        {target.cross(source).norm(),    target.dot(source),             0.0f},
        {0.0f,                           0.0f,                           1.0f}
    };

    Eigen::Matrix3f FFi;
    FFi << target,
        (source - target.dot(source)*target).normalized(),
        target.cross(source);

    Eigen::Matrix3f rotation = FFi * GG * FFi.inverse();

    //TODO: maybe there's a way to do this better?
    // rebuild in homogeneous coordinates
    Eigen::Matrix4f rotMatrix;
    rotMatrix.setIdentity();
    rotMatrix.block<3,3>(0,0) = rotation;

    return rotMatrix;

}

/**
 * @brief CADConverter::getNormals Estimate the normals for a given point cloud.
 * @param cloudPtr
 * @return
 */
std::vector<Eigen::Vector3f>* CADConverter::getNormals(PointCloud::Ptr const cloudPtr) const
{
    std::vector<Eigen::Vector3f>* normals = new std::vector<Eigen::Vector3f>();;

    qDebug("Calculating normals...");
    if (settings->normalMode == NormalMode::NEAREST_NEIGHBORS)
    {
        qDebug("Normal Method: NEAREST NEIGHBORS");
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (cloudPtr);

        size_t neighborCount = settings->normalsNeighborCount;


        pcl::PointIndices::Ptr neighborIndices(new pcl::PointIndices());
        neighborIndices->indices.resize(neighborCount);
        std::vector<float> neighborDistances(neighborCount);

        for (pcl::PointXYZ const point : cloudPtr->points)
        {
            if (kdtree.nearestKSearch(point, neighborCount, neighborIndices->indices, neighborDistances) > 0)
            {
                std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> neighbors(neighborCount);

                Plane* normalPlane = new Plane();

                std::vector<float> params = normalPlane->getBestFit(cloudPtr, neighborIndices);

                float tht = params[0]; //theta
                float phi = params[1];
                // float rho = params[2];

                // Build normal vector from chosen parameters

                float a = qCos(tht)*qSin(phi);
                float b = qSin(phi)*qSin(tht);
                float c = qCos(phi);

                Eigen::Vector3f rawNormal(a, b, c);
                rawNormal.normalize();

                // Calculate distances from each normal to the original point
                std::vector<float> normalDistances(neighborCount);

                float mfe = normalPlane->calculateMFE(cloudPtr);
                if (mfe <= settings->mfeThreshold)
                    normals->push_back(rawNormal);
            }
        }
    }
    else if (settings->normalMode == NormalMode::PCA)
    {
        qDebug("Normal Method: PCA");
        //TODO: set parallel method
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloudPtr);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        ne.setRadiusSearch(settings->normalSearchRadius);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        ne.compute(*cloud_normals);

        for (pcl::Normal const normal : cloud_normals->points)
        {
            Eigen::Vector3f normalVector = normal.getNormalVector3fMap();
            if (!std::isnan(normalVector.norm()))
                normals->push_back(normalVector);
        }
    }

    qDebug() << "Normals found: " << normals->size();
    return normals;
}

/**
 * @brief getShape Retrieve an instant of a shape based on the input Primitive Type.
 * @param type
 * @return
 */
PrimitiveShape* CADConverter::getShape(PrimitiveType const type) const
{
    // TODO: Find a more elegant solution
    switch (type)
    {
    case PrimitiveType::PLANE:
        return new Plane();
        break;
    case PrimitiveType::SPHERE:
        throw "Not Implemented";
        break;
    case PrimitiveType::CYLINDER:
        throw "Not Implemented";
        break;
    case PrimitiveType::TORUS:
        throw "Not Implemented";
        break;
    case PrimitiveType::CONE:
        throw "Not Implemented";
        break;
    default:
        throw "Unrecognized shape";
        break;
    }
}

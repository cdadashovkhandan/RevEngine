
#ifndef SETTINGS_H
#define SETTINGS_H

//TODO: maybe move to a separate file?
#include "primitives/PrimitiveType.h"
#include <map>

enum NormalMode {
    PCA = 0,
    NEAREST_NEIGHBORS = 1,
};

/**
 * @brief All user configurations are stored here.
 */
struct Settings
{
    // ----General Toggles----

    // Display the point cloud in the Viewport.
    bool showPointCloud = true;

    // Display recognized shapes in the Viewport.
    bool showShapes = true;

    // Show detected clusters in the Viewport.
    bool showClusters = true;

    // Show normals of points in the Viewport.
    bool showNormals = false;

    // Show Axis lines denoting the origin.
    bool showAxisLines = true;

    // Skip DBSCAN and segment using only RANSAC.
    bool forceRansac = false;

    // Show & use the downsampled version of the point cloud in the recognition pipeline.
    bool useDownsampledVersion = true;

    // ----Clustering----

    // Distance threshold for clustering
    float distanceThreshold = 0.05f;
    unsigned int minClusterSize = 50;
    float scaleFactor = 0.01f;
    float downSampleFactor = 0.5f;
    bool liveClusterPreview = false;

    // ----Normals----

    // Use the full pointcloud for normal detection instead of the downsampled point cloud.
    bool highPrecisionNormals = false;

    // Which Normal calculation method to use.
    NormalMode normalMode = NormalMode::PCA;

    // PCA:
    // Search radius for Normals.
    float normalSearchRadius = 0.3f;

    // Nearest Neighbors:
    int normalsNeighborCount = 3;
    // MFE threshold for normals.
    float mfeThreshold = 0.02f;

    // ----Recognition----

    // Primitive types that are available for the recognition stage.
    std::map<PrimitiveType, bool> primitiveTypes = {
        { PrimitiveType::PLANE, true },
        { PrimitiveType::SPHERE, false },
        { PrimitiveType::CYLINDER, false },
        { PrimitiveType::TORUS, false },
        { PrimitiveType::CONE, false },
    };

    //TODO: remove on release

    int lazyId = 5;

};

#endif //SETTINGS_H

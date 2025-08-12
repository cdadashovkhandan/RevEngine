
#ifndef SETTINGS_H
#define SETTINGS_H

//TODO: maybe move to a separate file?
#include "primitives/PrimitiveType.h"
#include <map>
#include <vector>
enum NormalMode {
    PCA = 0,
    NEAREST_NEIGHBORS = 1,
};

struct Settings
{
    bool showPointCloud = true;
    bool showShapes = true;
    bool showClusters = true;
    bool showNormals = false;
    bool showAxisLines = true;
    bool forceRansac = false;

    float distanceThreshold = 0.73f;
    unsigned int minClusterSize = 50;
    float scaleFactor = 0.01f;

    // Normals
    float normalSearchRadius = 0.3f;
    bool highPrecisionNormals = false;
    float mfeThreshold = 0.02f;
    int normalsNeighborCount = 3;
    NormalMode normalMode = NormalMode::PCA;

    // Recognition

    std::map<PrimitiveType, bool> primitiveTypes = {
        { PrimitiveType::PLANE, true },
        { PrimitiveType::SPHERE, false },
        { PrimitiveType::CYLINDER, false },
        { PrimitiveType::TORUS, false },
        { PrimitiveType::CONE, false },
    };

};

#endif //SETTINGS_H

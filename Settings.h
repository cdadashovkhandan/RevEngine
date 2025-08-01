
#ifndef SETTINGS_H
#define SETTINGS_H

//TODO: maybe move to a separate file?
enum NormalMode {
    PCA = 0,
    NEAREST_NEIGHBORS = 1,
};

struct Settings
{
    bool showPointCloud = true;
    bool showMesh = true;
    bool showClusters = true;
    bool showNormals = false;
    bool showAxisLines = true;

    float distanceThreshold = 0.05f;
    unsigned int minClusterSize = 50;
    float scaleFactor = 0.01f;

    // Normals
    float normalSearchRadius = 0.3f;
    bool highPrecisionNormals = false;
    float mfeThreshold = 0.02f;
    int normalsNeighborCount = 3;
    NormalMode normalMode = NormalMode::PCA;

};

#endif //SETTINGS_H

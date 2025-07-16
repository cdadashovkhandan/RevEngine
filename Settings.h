
#ifndef SETTINGS_H
#define SETTINGS_H

struct Settings
{
    bool showPointCloud = true;
    bool showMesh = true;
    bool showClusters = true;
    bool showNormals = false;

    float clusterTolerance = 0.1f;
    unsigned int minClusterSize = 50;
    unsigned int maxClusterSize = 1000;
    float scaleFactor = 0.01f;
};

#endif //SETTINGS_H

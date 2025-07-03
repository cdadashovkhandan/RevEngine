
#ifndef SETTINGS_H
#define SETTINGS_H

struct Settings
{
    bool showPointCloud = true;
    bool showMesh = true;
    bool showClusters = true;

    float clusterTolerance = 0.1f;
    unsigned int minClusterSize = 50;
    unsigned int maxClusterSize = 1000;

};

#endif //SETTINGS_H

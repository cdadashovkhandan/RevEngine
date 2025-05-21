// Adapted from the Advanced Computer Graphics Assignment Framework
#ifndef SCENE_H
#define SCENE_H

#include <qlist.h>

struct Camera;
class Mesh;

struct Scene
{
    Camera* camera { nullptr };
    Mesh* mesh { nullptr };

    Scene();
};



#endif //SCENE_H

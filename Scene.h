// Adapted from the Advanced Computer Graphics Assignment Framework
#ifndef SCENE_H
#define SCENE_H

#include <qlist.h>

struct Camera;
class Model;

struct Scene
{
    Camera* camera { nullptr };
    Model* model { nullptr };

    Scene();
};



#endif //SCENE_H

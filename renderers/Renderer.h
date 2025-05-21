// Adapted from the Advanced Computer Graphics Assignment Framework
#ifndef RENDERER_H
#define RENDERER_H

#include <QOpenGLFunctions_4_1_Core>

#include "../data/Mesh.h"

struct Settings;
struct Scene;

class Renderer
{
public:
    Renderer(QOpenGLFunctions_4_1_Core* gl, Scene* scene, Settings* settings);
    virtual ~Renderer();

    virtual void update_buffers(Mesh* mesh) = 0;
    virtual void update_uniforms() = 0;
    virtual void render() = 0;

    void init(QOpenGLFunctions_4_1_Core* f, Settings* s);

protected:
    QOpenGLFunctions_4_1_Core* gl;
    Scene* scene;
    Settings* settings;
    virtual void initBuffers() = 0;
};

#endif // RENDERER_H

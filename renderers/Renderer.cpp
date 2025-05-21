#include "Renderer.h"

Renderer::Renderer(QOpenGLFunctions_4_1_Core *gl, Scene *scene, Settings *settings) : gl(gl), scene(scene), settings(settings) {
}

Renderer::~Renderer() = default;

/**
 * @brief Renderer::init Call initialization function.
 * @param f
 * @param s
 */
void Renderer::init(QOpenGLFunctions_4_1_Core* f, Settings* s) {
    gl = f;
    settings = s;

    initBuffers();
}

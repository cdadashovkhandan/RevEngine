#ifndef MODELRENDERER_H
#define MODELRENDERER_H

#include "Renderer.h"

#include "data/Model.h"

#include <materials/Material.h>

class ModelRenderer : public Renderer
{
public:
    explicit ModelRenderer(QOpenGLFunctions_4_1_Core* gl, Scene* scene);
    ~ModelRenderer() override;

    void update_buffers(Model* model) override;
    void update_uniforms() override;
    void render() override;

protected:
    // void initShaders() override;
    void initBuffers() override;

private:
    GLuint vao;
    GLuint vbo;
    GLuint nbo;
    GLuint ibo;
    int render_size = 0;
    Material* pointCloudMat { nullptr };

    void drawMaterial(Material &material);
};

#endif // MODELRENDERER_H

#ifndef MODELRENDERER_H
#define MODELRENDERER_H

#include "Renderer.h"

#include "data/Model.h"

#include <materials/Material.h>
#include <qcolor.h>

class ModelRenderer : public Renderer
{
public:
    explicit ModelRenderer(QOpenGLFunctions_4_1_Core* gl, Scene* scene);
    ~ModelRenderer() override;

    void update_buffers(Model* model) override;
    void update_uniforms() override;
    void render() override;

protected:
    void initBuffers() override;

private:

    // Primary buffers for rendering the point cloud.
    GLuint vao;
    GLuint vbo;
    GLuint vbo_colors;
    GLuint nbo;
    GLuint ibo;
    int render_size = 0;

    // Shapes to be rendered.
    std::vector<std::shared_ptr<RenderShape>> renderShapes;

    Material* pointCloudMat { nullptr };
    Material* normalsMat { nullptr };
    Material* worldMat { nullptr };

    Material* shapeMat { nullptr };

    void drawMaterial(Material &material);

    void drawShape(std::shared_ptr<RenderShape> const renderShape);

    // Colors for visually differentiating between clusters or shapes.
    static const size_t CLUSTER_COLOR_COUNT = 5;
    QColor colors[CLUSTER_COLOR_COUNT] { QColorConstants::Svg::red, QColorConstants::Svg::green, QColorConstants::Svg::cyan, QColorConstants::Svg::magenta, QColorConstants::Svg::yellow };

    void clearRenderShapes();
};

#endif // MODELRENDERER_H

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
    // void initShaders() override;
    void initBuffers() override;

private:
    static const size_t CLUSTER_COLOR_COUNT = 5;
    GLuint vao;
    GLuint vbo;
    GLuint vbo_colors;
    GLuint nbo;
    GLuint ibo;
    int render_size = 0;

    GLuint testVao;

    std::vector<std::shared_ptr<RenderShape>> renderShapes;

    Material* pointCloudMat { nullptr };
    Material* normalsMat { nullptr };
    Material* worldMat { nullptr };

    Material* shapeMat { nullptr };

    void drawMaterial(Material &material);

    void drawShape(std::shared_ptr<RenderShape> const renderShape);

    QColor clusterColors[CLUSTER_COLOR_COUNT] { QColorConstants::Svg::red, QColorConstants::Svg::green, QColorConstants::Svg::cyan, QColorConstants::Svg::magenta, QColorConstants::Svg::yellow };

    void clearRenderShapes();
};

#endif // MODELRENDERER_H

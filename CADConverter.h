#ifndef CADCONVERTER_H
#define CADCONVERTER_H

#include "HoughTransformer.h"
#include "data/Model.h"

#include <QMatrix4x4>
class CADConverter
{
public:
    CADConverter();
    HoughTransformer* houghTransformer;
    friend class ModelManager; //TODO maybe not necessary
    Model* convertModel(Model& model) const;

    QVector<QVector3D> getNormals(const QVector<QVector3D> points) const;
private:
    /* TODO:
     * Prim families
     * discretized region T
     *
     */
    QVector3D getCentroid(QVector<QVector3D>& points) const;
    QVector<QVector3D>* transform(QVector<QVector3D>& points, QMatrix4x4 const tMatrix) const;

    float maxDistance = 0.2f / 100.0f; // TODO: put this into Settings and make it adjustable
    QVector<QVector3D> getNeighbors(const QVector3D target, const QVector<QVector3D> points) const;
};

#endif // CADCONVERTER_H

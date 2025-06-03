#ifndef CADCONVERTER_H
#define CADCONVERTER_H

#include "data/Model.h"

#include <QMatrix4x4>
class CADConverter
{
public:
    CADConverter();
    friend class ModelManager;

private:
    /* TODO:
     * Prim families
     * discretized region T
     *
     */
    QVector3D getCentroid(QVector<QVector3D>& points) const;
    QVector<QVector3D>* transform(QVector<QVector3D>& points, QMatrix4x4 tMatrix) const;

    Model* convertModel(Model& model) const;
};

#endif // CADCONVERTER_H

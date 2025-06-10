#ifndef CADCONVERTER_H
#define CADCONVERTER_H

#include "HoughTransformer.h"
#include "data/Model.h"

#include <QMatrix4x4>
class CADConverter
{
public:
    CADConverter();
    HoughTransformer houghTransformer;
    friend class ModelManager; //TODO maybe not necessary
    Model* convertModel(Model& model) const;

private:
    /* TODO:
     * Prim families
     * discretized region T
     *
     */
    QVector3D getCentroid(QVector<QVector3D>& points) const;
    QVector<QVector3D>* transform(QVector<QVector3D>& points, QMatrix4x4 tMatrix) const;


};

#endif // CADCONVERTER_H

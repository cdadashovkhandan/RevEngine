#include "HoughTransformer.h"

#include <primitives/NormalPlane.h>

HoughTransformer::HoughTransformer()
{
    //TODO: actually implement populating the map
    //shapes = QMap<PrimitiveType, PrimitiveShape*>();
    // shapes.insert(PrimitiveType::NORMALPLANE, NormalPlane)
}


// QList<PrimitiveShape*> HoughTransformer::applyTransform(QVector<QVector3D> const points, QList<PrimitiveType> const types) const
// {
//     QList<PrimitiveShape*> output;
//     for (PrimitiveType type : types)
//     {

//         PrimitiveShape* shape = getShape(type);
//         getBestFit(points);
//         output.append(shape);
//     }
//     return output;
// }

// PrimitiveShape* HoughTransformer::getShape(PrimitiveType const type) const
// {
//     // TODO: adapt to multiple shapes
//     return new NormalPlane();
// }
// template <typename Shape>


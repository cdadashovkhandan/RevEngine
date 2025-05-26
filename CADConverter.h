#ifndef CADCONVERTER_H
#define CADCONVERTER_H

#include "data/Model.h"
class CADConverter
{
public:
    CADConverter();
    friend class ModelManager;

private:
    Model* convertModel(Model* model) const;
};

#endif // CADCONVERTER_H

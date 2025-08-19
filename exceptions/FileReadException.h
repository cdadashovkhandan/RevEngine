#ifndef FILEREADEXCEPTION_H
#define FILEREADEXCEPTION_H

#include <QException>


class FileReadException : public QException
{
public:
    void raise() const override { throw *this; }
    FileReadException *clone() const override { return new FileReadException(*this); }
};

#endif //FILEREADEXCEPTION_H

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ModelManager.h"

#include <QMainWindow>
#include "Settings.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_importModelButton_clicked();

private:
    Ui::MainWindow *ui;
    ModelManager modelManager;
    Settings settings;
};
#endif // MAINWINDOW_H

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

    void on_convertModelButton_clicked();

    void on_toggleClustersCheckBox_toggled(bool checked);

    void on_minClusterSizeSpinBox_valueChanged(int arg1);

    void on_maxClusterSizeSpinBox_valueChanged(int arg1);

    void on_clusterToleranceSpinBox_valueChanged(double arg1);

    void on_recalcClusterButton_clicked();

    void on_togglePointCloudCheckBox_toggled(bool checked);

private:
    Ui::MainWindow *ui;
    ModelManager* modelManager;
    Settings settings;
};
#endif // MAINWINDOW_H

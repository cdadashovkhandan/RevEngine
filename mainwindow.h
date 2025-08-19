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

    void on_distanceThresholdSpinBox_valueChanged(double arg1);

    void on_recalcClusterButton_clicked();

    void on_togglePointCloudCheckBox_toggled(bool checked);

    void on_toggleNormalsCheckBox_toggled(bool checked);

    void on_scaleFactorSpinBox_valueChanged(double arg1);

    void on_highPrecisionNormalsCheckBox_toggled(bool checked);

    void on_normalNeighborsSpinBox_valueChanged(int arg1);

    void on_lazyImportButton_clicked();

    void on_comboBox_currentIndexChanged(int index);

    void on_normalSearchRadiusSpinBox_valueChanged(double arg1);

    void on_recalcNormalsButton_clicked();

    void on_toggleAxisLinesCheckBox_toggled(bool checked);

    void on_shapePlaneCheckBox_toggled(bool checked);

    void on_toggleShapesCheckBox_toggled(bool checked);

    void on_forceRansacCheckBox_toggled(bool checked);

    void on_recognizeShapesButton_clicked();

    void on_lazyIdSpinBox_valueChanged(int arg1);

    void on_showDownsampledCheckBox_toggled(bool checked);

    void on_downSampleFactorSpinBox_2_valueChanged(double arg1);

private:
    Ui::MainWindow *ui;
    ModelManager* modelManager;
    Settings settings;
};
#endif // MAINWINDOW_H

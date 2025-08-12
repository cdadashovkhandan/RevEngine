#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QFileDialog>
#include <omp.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , settings()
{
    ui->setupUi(this);
    ui->viewport->settings = &settings;
    modelManager = new ModelManager(&settings);
    omp_set_num_threads(8);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_importModelButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Text File...", {}, "Text Files (*.txt)");
    if (fileName.isEmpty())
        return;
    Model* model = modelManager->createModel(fileName);
    ui->viewport->showModel(model);
}


void MainWindow::on_convertModelButton_clicked()
{
    Model* model = modelManager->preprocessModel(*modelManager->getActiveModel());
    ui->viewport->showModel(model);
}


void MainWindow::on_toggleClustersCheckBox_toggled(bool checked)
{
    settings.showClusters = checked;
    ui->viewport->showModel(modelManager->getActiveModel());
}


void MainWindow::on_minClusterSizeSpinBox_valueChanged(int arg1)
{
    settings.minClusterSize = arg1;
}


void MainWindow::on_distanceThresholdSpinBox_valueChanged(double arg1)
{
    settings.distanceThreshold = arg1;
    // TODO: for the love of god don't leave this in the final code
    // on_recalcClusterButton_clicked();
}


void MainWindow::on_recalcClusterButton_clicked()
{
    Model* model = modelManager->getActiveModel();
    modelManager->recalculateClusters(model);
    ui->viewport->showModel(model);
}


void MainWindow::on_togglePointCloudCheckBox_toggled(bool checked)
{
    // TODO: This currently doesn't work because of the Renderer
    // settings.showPointCloud = checked;
    // ui->viewport->showModel(modelManager->getActiveModel());
}


void MainWindow::on_toggleNormalsCheckBox_toggled(bool checked)
{
    settings.showNormals = checked;
    ui->viewport->update();
}


void MainWindow::on_scaleFactorSpinBox_valueChanged(double arg1)
{
    settings.scaleFactor = arg1;
    ui->viewport->update();
}


void MainWindow::on_highPrecisionNormalsCheckBox_toggled(bool checked)
{
    settings.highPrecisionNormals = checked;
    //TODO: proper modelmanager updating
    ui->viewport->update();
}


void MainWindow::on_normalNeighborsSpinBox_valueChanged(int arg1)
{
    settings.normalsNeighborCount = arg1;
}


//TODO: REMOVE ON RELEASE
void MainWindow::on_lazyImportButton_clicked()
{
    // QString fileName = "/home/chingiz/Documents/uni/intproj/Fit4CAD/dataset/training_set/PC6.txt";
    QString fileName = "/home/chingiz/Documents/uni/intproj/fitting_geometric_primitives/test/pointCloud/pointCloud24.txt";
    Model* model = modelManager->createModel(fileName);
    ui->viewport->showModel(model);
}


void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    settings.normalMode = (NormalMode) index;
}


void MainWindow::on_normalSearchRadiusSpinBox_valueChanged(double arg1)
{
    settings.normalSearchRadius = arg1;
}


void MainWindow::on_recalcNormalsButton_clicked()
{
    Model* model = modelManager->getActiveModel();
    modelManager->recalculateNormals(model);
    ui->viewport->showModel(model);
}


void MainWindow::on_toggleAxisLinesCheckBox_toggled(bool checked)
{
    settings.showAxisLines = checked;
    ui->viewport->update();
}


void MainWindow::on_shapePlaneCheckBox_toggled(bool checked)
{
    settings.primitiveTypes[PrimitiveType::PLANE] = checked;
}


void MainWindow::on_toggleShapesCheckBox_toggled(bool checked)
{
    settings.showShapes = checked;

    ui->viewport->update();
}


void MainWindow::on_forceRansacCheckBox_toggled(bool checked)
{
    settings.forceRansac = checked;
}


void MainWindow::on_recognizeShapesButton_clicked()
{
    Model* model = modelManager->recognizeShapes(*modelManager->getActiveModel());
    ui->viewport->showModel(model);
}


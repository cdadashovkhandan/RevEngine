#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , settings()
{
    ui->setupUi(this);
    ui->viewport->settings = &settings;
    modelManager = new ModelManager(&settings);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_importModelButton_clicked()
{
    // QString fileName = QFileDialog::getOpenFileName(this, "Text File...", {}, "Text Files (*.txt)");
    //TODO: unhardcode
    QString fileName = "/home/chingiz/Documents/uni/intproj/Fit4CAD/dataset/training_set/PC6.txt";
    if (fileName.isEmpty())
        return;
    Model* model = modelManager->createModel(fileName);
    ui->viewport->showModel(model);
}


void MainWindow::on_convertModelButton_clicked()
{
    Model* model = modelManager->generateMesh(*modelManager->getActiveModel());
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


void MainWindow::on_maxClusterSizeSpinBox_valueChanged(int arg1)
{
    settings.maxClusterSize = arg1;
}


void MainWindow::on_clusterToleranceSpinBox_valueChanged(double arg1)
{
    settings.clusterTolerance = arg1;
    // TODO: for the love of god don't leave this in the final code
    on_recalcClusterButton_clicked();
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


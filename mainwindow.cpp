#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QErrorMessage>
#include <QFileDialog>
#include <QMessageBox>
#include <omp.h>
#include <exceptions/FileReadException.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , settings()
{
    ui->setupUi(this);
    ui->viewport->settings = &settings;
    modelManager = new ModelManager(&settings);
    omp_set_num_threads(12);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete modelManager;
}

void MainWindow::on_importModelButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Text File...", {}, "Text Files (*.txt)");
    Model* model;

    if (fileName.isEmpty())
        return; // Do nothing.

    try
    {
        model = modelManager->createModel(fileName);
    }
    catch (FileReadException& e)
    {
        QMessageBox errorMessage;
        errorMessage.critical(this, "File Read Error", "File could not be read.");
        errorMessage.show();
        return;
    }

    ui->viewport->showModel(model);
    updateInfoText();
    ui->detectedShapesListWidget->clear();
}


void MainWindow::on_convertModelButton_clicked()
{
    if (modelManager->model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
    {
        modelManager->preprocessModel();
        ui->viewport->showModel(modelManager->model);
    }

    updateInfoText();
}


void MainWindow::on_toggleClustersCheckBox_toggled(bool checked)
{
    settings.showClusters = checked;
    ui->viewport->showModel(modelManager->model);
}


void MainWindow::on_minClusterSizeSpinBox_valueChanged(int arg1)
{
    settings.minClusterSize = arg1;
}


void MainWindow::on_distanceThresholdSpinBox_valueChanged(double arg1)
{
    settings.distanceThreshold = arg1;
}


void MainWindow::on_recalcClusterButton_clicked()
{
    if (enforceStatus(ModelStatus::PREPROCESSED))
    {
        modelManager->recalculateClusters();
        ui->viewport->showModel(modelManager->model);
    }
}


void MainWindow::on_togglePointCloudCheckBox_toggled(bool checked)
{
    settings.showPointCloud = checked;
    ui->viewport->showModel(modelManager->model);
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
    // QString fileName = "/home/chingiz/Documents/uni/intproj/Fit4CAD/dataset/training_set/PC" + QString::number(settings.lazyId) + ".txt";
    QString fileName = "/home/chingiz/Documents/uni/intproj/fitting_geometric_primitives/test/pointCloud/pointCloud" + QString::number(settings.lazyId) + ".txt";
    Model* model = modelManager->createModel(fileName);
    ui->viewport->showModel(model);

    updateInfoText();
    ui->detectedShapesListWidget->clear();
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
    if (enforceStatus(ModelStatus::PREPROCESSED))
    {
        if (modelManager->model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
        {
            modelManager->recalculateNormals();
            ui->viewport->showModel(modelManager->model);
        }

        updateInfoText();
    }
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
    if (enforceStatus(ModelStatus::PREPROCESSED))
    {
        if (modelManager->model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
        {
            modelManager->recognizeShapes();
            ui->viewport->showModel(modelManager->model);
        }


        ui->detectedShapesListWidget->clear();
        for (auto shape : *modelManager->model->shapes)
        {
            new QListWidgetItem(shape->toString(), ui->detectedShapesListWidget);
        }
        updateInfoText();
    }
}

bool MainWindow::enforceStatus(ModelStatus modelStatus)
{
    if (modelManager->modelStatus < modelStatus)
    {
        QMessageBox errorMessage;
        errorMessage.warning(this, "Point Cloud not ready", "Please pre-process the point cloud first.");
        errorMessage.show();
        return false;
    }

    return true;
}

void MainWindow::on_lazyIdSpinBox_valueChanged(int arg1)
{
    settings.lazyId = arg1;
}


void MainWindow::on_showDownsampledCheckBox_toggled(bool checked)
{
    // settings.showDownsampledVersion = checked;

    // Model* model = modelManager->getActiveModel();
    // if (modelManager->model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
    //     ui->viewport->showModel(model);
}


void MainWindow::on_downSampleFactorSpinBox_2_valueChanged(double arg1)
{
    settings.downSampleFactor = arg1;
}


void MainWindow::on_recalcDownsampleButton_clicked()
{
    if (enforceStatus(ModelStatus::PREPROCESSED))
    {
        Model* model = modelManager->model;
        if (model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
        {
            modelManager->recalculateDownsample();
            // This must be cleared to avoid mismatches and segfaults.
            if (model->clusterIndices != nullptr && !model->clusterIndices->empty())
                model->clusterIndices->clear();

            if (model->normals != nullptr && !model->normals->empty())
                model->normals->clear();

            ui->viewport->showModel(model);

            updateInfoText();
        }
    }
}


void MainWindow::on_toggleRunOnDownsampledCloudCheckBox_toggled(bool checked)
{
    settings.useDownsampledVersion = checked;

    Model* model = modelManager->model;
    if (model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
    {
        //TODO: move this to Model or ModelManager.
        // This must be cleared to avoid mismatches and segfaults.
        if (model->clusterIndices != nullptr && !model->clusterIndices->empty())
            model->clusterIndices->clear();

        if (model->normals != nullptr && !model->normals->empty())
            model->normals->clear();

        ui->viewport->showModel(model);
    }
}

void MainWindow::updateInfoText()
{
    Model* model = modelManager->model;
    if (model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
    {
        ui->infoTextEdit->clear();

        ui->infoTextEdit->append(QString("Full cloud: %1 points").arg(QString::number(model->pointCloud->points.size())));
        if (modelManager->modelStatus >= ModelStatus::PREPROCESSED)
        {
            ui->infoTextEdit->append(QString("Downsampled cloud: %1 points").arg(QString::number(model->pointCloudDownsampled->points.size())));
            ui->infoTextEdit->append(QString("Downsizing scale factor: %1").arg(QString::number(model->scaleFactor, 'g', 3)));
        }

        if (modelManager->modelStatus >= ModelStatus::ANALYZED)
        {
            ui->infoTextEdit->append(QString("Detected shape count: %1").arg(QString::number(model->shapes->size())));
        }

        // ui->infoTextEdit->append(QString("Model status: %1").arg(Util::QtEnumToString(modelManager->modelStatus)));
    }

}


void MainWindow::on_finalizeButton_clicked()
{
    Model* model = modelManager->model;
    if (model != nullptr) // Redundant but wouldn't hurt to prevent a segfault.
    {
        modelManager->finalizeModel();
        ui->viewport->showModel(model);
    }
}


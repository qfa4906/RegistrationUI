#pragma once

#include <QtWidgets/QMainWindow>
#include<QThread>
#include<QSettings>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileDialog>
#include<QVTKOpenGLWidget.h>
#include <vtkOutputWindow.h>
#include<vtkGenericOpenGLRenderWindow.h>
#include<pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include "ui_RegistrationDemo.h"
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include "Registration.h"
using namespace std;
typedef pcl::PointXYZRGB PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
namespace Ui {
    class RegistrationDemo;
}
class RegistrationDemo : public QMainWindow
{
    Q_OBJECT

public:
    RegistrationDemo(QWidget *parent = nullptr);
    ~RegistrationDemo();
    Ui::RegistrationDemoClass ui;
    PointCloudT::Ptr object = nullptr;
    PointCloudT::Ptr scene = nullptr;
    PointCloudT::Ptr object_aligned = nullptr;
    void openSceneFile();
    void openObjectFile();
    void openSTLObjectFile();
    void generateData();
    void startAlign();
    void startTest();
    void colorPointCloud(pcl::PointCloud<PointNT>::Ptr cloud, vector<int> rgb);
    double pointToPlaneDistance(pcl::PointXYZRGB point, pcl::PointXYZRGB p1,
        pcl::PointXYZRGB p2, pcl::PointXYZRGB p3);
    bool atSameSide(pcl::PointXYZRGB point1, pcl::PointXYZRGB point2, pcl::PointXYZRGB p1,
        pcl::PointXYZRGB p2, pcl::PointXYZRGB p3);
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    Eigen::Matrix4f transformeMatrix;
    string filepath_source = "";
    string filepath_target = "";
public slots:
    void on_isremoval_clicked();
    void on_isfilter_clicked();
    void showResault();
    void showTestResault(QString text);
    void showRegResault(QString text);
    void setProgressBarValue(int);
signals:
    void loadScene(string filepath, Ui::RegistrationDemoClass ui);
    void loadObject(string filepath, Ui::RegistrationDemoClass ui);
    void changeSetting(bool,bool);
    void doTest(vector<double>, vector<double>);
};

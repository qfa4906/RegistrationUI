#pragma once
#include<QVTKOpenGLNativeWidget.h>
#include <string>
#include<cmath>
#include <pcl/io/ply_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>n
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/time.h>
#include <Eigen/Core>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>
#include <gr/algorithms/match4pcsBase.h>
#include <gr/algorithms/FunctorSuper4pcs.h>
#include <pcl/registration/super4pcs.h>
#include<pcl/io/obj_io.h>
#include<QThread>
#include<QSettings>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileDialog>
#include<QTextCodec>
#include <QObject>
#include <QMap>
#include "ui_RegistrationDemo.h"
#include "RegistrationDemo.h"
using namespace std;
using PointT = pcl::PointXYZ;
typedef pcl::PointXYZRGB PointNT;
typedef pcl::PointXYZRGBNormal PointNTRGB;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
namespace Ui {
	class RegistrationDemo;
}
class myRegistration : public QObject
{
	Q_OBJECT
public:
	myRegistration(QObject* parent = 0);
	~myRegistration();
	void setObject(string);
	void setScene(string);
	void setParam(double ,double);
	void setOutput(PointCloudT::Ptr, Eigen::Matrix4f&);
	void setTestParam(vector<double>, vector<double>);
	void setPresetTransformeMatrix(Eigen::Matrix4d Matrix);
	void ChangeRemoval(bool set);
	void ChangeFilter(bool set);
	void ChangeMethod(bool set);
	vector<double> paramSwap();
	void align();
public slots:
	
	void Run();
signals:
	// ·¢ËÍ½ø¶È£¨0-100£©
	void progress(int);
	void emitTestResault(QString);
	void emitRegResault(QString);
	void finish();
private:
	string filepath_source;
	string filepath_target;
	double _overlap;
	double _delta;
	bool useRemoval = true;
	bool useFilter = true;
	bool use4PCS = true;
	Eigen::Matrix4d presetTransformeMatrix;
	vector<double> testOverlap;
	vector<double> testDelta;
	vector<vector<double>> outputArray;
	//PointCloudT::Ptr object;
	//PointCloudT::Ptr scene;
	PointCloudT::Ptr object_icp_aligned;
	Eigen::Matrix4f transformeMatrix;
	void statistical_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_inliers);
	void grid_filter(pcl::PointCloud<PointNT>::Ptr cloud, pcl::PointCloud<PointNT>::Ptr cloud_filtered);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> customHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void savePointCloud(pcl::PointCloud<PointNT>::Ptr cloud, std::string outpath);
	void statistical_removal(pcl::PointCloud<PointNT>::Ptr cloud, pcl::PointCloud<PointNT>::Ptr cloud_filtered_inliers);
	void grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
	//double paramSwap(pcl::PointCloud<PointNT>::Ptr object, pcl::PointCloud<PointNT>::Ptr scenc, double begin, double end, double block);
	void fillVector(vector<double>& vec, double begin, double end, double block);
	vector<vector<double>> getOutputArray() {
		return outputArray;
	};
	Eigen::Matrix4f getTransformMatrix() {
		return transformeMatrix;
	};
};


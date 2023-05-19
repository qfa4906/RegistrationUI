#include "RegistrationDemo.h"
#include"Registration.h"

RegistrationDemo::RegistrationDemo(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	vtkOutputWindow::SetGlobalWarningDisplay(0);
	QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
	connect(ui.load_object, &QPushButton::clicked, this, &RegistrationDemo::openObjectFile);
	connect(ui.loadscene, &QPushButton::clicked, this, &RegistrationDemo::openSceneFile);
	connect(ui.start_align, &QPushButton::clicked, this, &RegistrationDemo::startAlign);
	connect(ui.pushButton, &QPushButton::clicked, this, &RegistrationDemo::startTest);
	connect(ui.test, &QPushButton::clicked, this, &RegistrationDemo::openSTLObjectFile);
	connect(ui.test_2, &QPushButton::clicked, this, &RegistrationDemo::generateData);
	connect(this, &RegistrationDemo::doTest, this, &RegistrationDemo::startTest);
	
}


pcl::PointXYZRGB operator-(pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2) {
	pcl::PointXYZRGB v1;
	v1.x = p1.x - p2.x;
	v1.y = p1.y - p2.y;
	v1.z = p1.z - p2.z;
	return v1;
}

pcl::PointXYZRGB operator+(pcl::PointXYZRGB& p1, pcl::PointXYZRGB& p2) {
	pcl::PointXYZRGB v1;
	v1.x = p1.x + p2.x;
	v1.y = p1.y + p2.y;
	v1.z = p1.z + p2.z;
	return v1;
}

pcl::PointXYZRGB scale(double s, pcl::PointXYZRGB p1) {
	pcl::PointXYZRGB v1;
	v1.x = p1.x * s;
	v1.y = p1.y * s;
	v1.z = p1.z * s;
	return v1;
}

pcl::PointXYZRGB operator/(pcl::PointXYZRGB& p1, double& s) {
	pcl::PointXYZRGB v1;
	v1.x = p1.x / s;
	v1.y = p1.y / s;
	v1.z = p1.z / s;
	return v1;
}

inline double myDot(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {
	return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

inline pcl::PointXYZRGB myCross(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {
	pcl::PointXYZRGB res;
	res.x = p1.y * p2.z - p1.z * p2.y;
	res.y = p1.z * p2.x - p1.x * p2.z;
	res.z = p1.x * p2.y - p1.y * p2.x;
	return res;
}

inline pcl::PointXYZRGB normalize(pcl::PointXYZRGB p1) {
	pcl::PointXYZRGB res = p1;
	double length = pow(myDot(p1, p1), 0.5);
	res.x /= length;
	res.y /= length;
	res.z /= length;
	return res;
}


double RegistrationDemo::pointToPlaneDistance(pcl::PointXYZRGB point, pcl::PointXYZRGB p1, 
											pcl::PointXYZRGB p2, pcl::PointXYZRGB p3) {
	pcl::PointXYZRGB v0 = point - p1;
	pcl::PointXYZRGB v1 = p2 - p1;
	pcl::PointXYZRGB v2 = p3 - p1;
	pcl::PointXYZRGB cross = myCross(v1,v2);
	cross = normalize(cross);
	return abs(myDot(cross, v0));


}

bool RegistrationDemo::atSameSide(pcl::PointXYZRGB point1, pcl::PointXYZRGB point2, pcl::PointXYZRGB p1,
	pcl::PointXYZRGB p2, pcl::PointXYZRGB p3) {
	pcl::PointXYZRGB v01 = point1 - p1;
	pcl::PointXYZRGB v02 = point2 - p1;
	pcl::PointXYZRGB v1 = p2 - p1;
	pcl::PointXYZRGB v2 = p3 - p1;
	pcl::PointXYZRGB cross = myCross(v1, v2);
	cross = normalize(cross);
	return myDot(cross, v01) * myDot(cross, v02) > 0;
}

void RegistrationDemo::openSTLObjectFile() {
	if (filepath_target == "" || object_aligned == nullptr) {
		cout << "请先执行点云配准" << endl;
		return;
	}
	QSettings s; // 为了记住上一次的路径
	QString filepath2;
	string filepath1 = filepath_target;
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileSTL(filepath1, mesh); 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZRGB>);
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata);  //将PolygonMesh对象转化为vtkPolyData对象
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud_model);
	
	map<int, vector<pcl::Vertices>> map;
	for (int i = 0; i < mesh.polygons.size(); i++) {
		for (int j = 0; j < 3; j++) {
			map[mesh.polygons[i].vertices[j]].push_back(mesh.polygons[i]);
		}
	}

	pcl::PointCloud<PointNT>::Ptr cloud_camara = object_aligned;
	/*pcl::PLYReader reader;
	reader.read(filepath2.toStdString(), *cloud_camara);*/
	pcl::PointXYZRGB center;
	double cx = 0, cy = 0, cz = 0;
	for (int i = 0; i < cloud_model->size(); i++) {
		center.x += cloud_model->points[i].x / cloud_model->size();
		center.y += cloud_model->points[i].y / cloud_model->size();
		center.z += cloud_model->points[i].z / cloud_model->size();
	}
	
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;       //建立kd tree
	kdtree.setInputCloud(cloud_model);        //设置搜索空间
	pcl::PointXYZRGB pSearch, pMin, pMax;       //搜索点，三个轴的最大值和最小值
	pcl::getMinMax3D(*cloud_model, pMin, pMax);    //需要include<pcl/common/common.h>=
	int k = 1;
	std::vector<int> ptIdxByRadius;   //存储近邻索引
	std::vector<float> ptRadius;
	double scaler = 5;
	for (int i = 0; i < cloud_camara->size(); i++) {
		ptIdxByRadius.clear();
		ptRadius.clear();
		pSearch = cloud_camara->points[i];//对于cloud_camara中的每一个点
		kdtree.nearestKSearch(pSearch, k, ptIdxByRadius, ptRadius);//找到其在cloud_model中的最近邻
		pcl::PointXYZRGB target_point = cloud_model->points[ptIdxByRadius[0]];//将找到的点记为target_point
		vector<pcl::Vertices> target_vers = map[ptIdxByRadius[0]];//
		double minDist = INT32_MAX;
		pcl::PointXYZRGB p1, p2, p3;
		for (int j = 0; j < target_vers.size(); j++) {
			pcl::PointXYZRGB p1_, p2_, p3_;
			p1_ = cloud_model->points[target_vers[j].vertices[0]];
			p2_ = cloud_model->points[target_vers[j].vertices[1]];
			p3_ = cloud_model->points[target_vers[j].vertices[2]];
			double dist = pointToPlaneDistance(cloud_camara->points[i], p1_, p2_, p3_);
			if (dist < minDist) {
				minDist = dist;
				p1 = p1_;
				p2 = p2_;
				p3 = p3_;
			}
		}
		cout << minDist << endl;
		double rate = min(scaler * minDist, 1.0);
		if (atSameSide(center, cloud_camara->points[i], p1, p2, p3)) {
			if (rate > 0.5) {
				cloud_camara->points[i].b = 0;
				cloud_camara->points[i].g = 510 - 510 * rate;
				cloud_camara->points[i].r = 255;
			}
			else {
				cloud_camara->points[i].b = 0;
				cloud_camara->points[i].g = 255;
				cloud_camara->points[i].r = 510 * rate;
			}
			
		}
		else {
			if (rate > 0.5) {
				cloud_camara->points[i].b = 255;
				cloud_camara->points[i].g = 510 - 510 * rate;
				cloud_camara->points[i].r = 0;
			}
			else {
				cloud_camara->points[i].b = 510 * rate;
				cloud_camara->points[i].g = 255;
				cloud_camara->points[i].r = 0;
			}
		}
			
	}
	auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
	auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow2->AddRenderer(renderer2);
	viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_rgb(cloud_camara);
	viewer->addPointCloud(cloud_camara, cloud_rgb, "object");
	//viewer->addPointCloud(cloud_model, ColorHandlerT(cloud_model, 0, 128.0, 0), "model");
	//viewer->addPointCloudNormals<PointNT, pcl::Normal>(cloud, cloud_normals,1, 0.5, "normals");
	vtkSmartPointer<vtkRenderWindow> renderer = viewer->getRenderWindow();
	viewer->setupInteractor(ui.object->GetInteractor(), ui.object->GetRenderWindow());
	ui.object_aligned->SetRenderWindow(renderer);
	ui.object_aligned->repaint();
}
template<class myPoint>
void myFlipNormalTowardsViewpoint(const myPoint& point,
	float 	vp_x,
	float 	vp_y,
	float 	vp_z,
	float& nx,
	float& ny,
	float& nz) {
	double f = (vp_x - point.x) * nx + (vp_y - point.y) * ny + (vp_z - point.z) * nz;
	if (f < 0)
		return;
	else {
		nx *= -1;
		ny *= -1;
		nz *= -1;
	}
}

void RegistrationDemo::generateData() {
	pcl::PointCloud<PointNT>::Ptr cloud(new pcl::PointCloud<PointNT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT>());
	pcl::NormalEstimation<PointNT, pcl::Normal> ne;
	pcl::PLYReader reader;

	reader.read(filepath_source, *cloud);
	srand(time(nullptr));
	int randnum = rand();
	int index = randnum*100 % cloud->size();
	cout << randnum << endl;
	cout << cloud->size() << endl;
	cout << index << endl;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(1);
	ne.compute(*cloud_normals);
	double cx = 0, cy = 0, cz = 0;
	for (int i = 0; i < cloud->size(); i++) {
		cx += cloud->points[i].x / cloud->size();
		cy += cloud->points[i].y / cloud->size();
		cz += cloud->points[i].z / cloud->size();
	}

	for (int i = 0; i < cloud->size(); i++) {
		myFlipNormalTowardsViewpoint(cloud->points[i], cx, cy, cz,
			cloud_normals->points[i].normal_x,
			cloud_normals->points[i].normal_y,
			cloud_normals->points[i].normal_z);
	}
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_withNormal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_withNormal);
	vector<int> rgb = { 255,255,255 };
	colorPointCloud(cloud, rgb);

	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;       //建立kd tree
	kdtree.setInputCloud(cloud_withNormal);        //设置搜索空间
	pcl::PointXYZRGBNormal pSearch = cloud_withNormal->points[index], pMin, pMax;       //搜索点，三个轴的最大值和最小值
	pcl::getMinMax3D(*cloud_withNormal, pMin, pMax);    //需要include<pcl/common/common.h>=
	PointNT tmp;       //用于存储临时点
	float r = 5;
	std::vector<int> ptIdxByRadius;   //存储近邻索引
	std::vector<float> ptRadius;
	kdtree.radiusSearch(pSearch, r, ptIdxByRadius, ptRadius);
	double sigma = 5;
	double scaler = 0.25;
	double hx_max = scaler * cloud_normals->points[ptIdxByRadius[0]].normal_x;
	double hy_max = scaler * cloud_normals->points[ptIdxByRadius[0]].normal_y;
	double hz_max = scaler * cloud_normals->points[ptIdxByRadius[0]].normal_z;
	double h_max = pow(hx_max * hx_max + hy_max * hy_max + hz_max * hz_max, 0.5);
	for (int i = 0; i < ptIdxByRadius.size(); i++) {
		int id = ptIdxByRadius[i];
		double hx = scaler * exp(-ptRadius[i] / sigma) * cloud_normals->points[id].normal_x;
		double hy = scaler * exp(-ptRadius[i] / sigma) * cloud_normals->points[id].normal_y;
		double hz = scaler * exp(-ptRadius[i] / sigma)* cloud_normals->points[id].normal_z;
		double h = pow(hx * hx  +hy * hy + hz * hz, 0.5);
		cloud->points[id].x -= hx;
		cloud->points[id].y -= hy;
		cloud->points[id].z -= hz;
		cloud->points[id].r = 255;
		cloud->points[id].g = 255 * (h_max - h) / h_max;
		cloud->points[id].b = 255 * (h_max - h) / h_max;
	}
	auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
	auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow2->AddRenderer(renderer2);
	viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_rgb(cloud);
	viewer->addPointCloud(cloud, cloud_rgb, "object");
	//viewer->addPointCloudNormals<PointNT, pcl::Normal>(cloud, cloud_normals,1, 0.5, "normals");
	vtkSmartPointer<vtkRenderWindow> renderer = viewer->getRenderWindow();
	viewer->setupInteractor(ui.object->GetInteractor(), ui.object->GetRenderWindow());
	ui.object->SetRenderWindow(renderer);
	ui.object->repaint();
	pcl::PLYWriter writer;
	writer.write("cloud.ply", *cloud, true);


}

void RegistrationDemo::openObjectFile() {
	QSettings s; // 为了记住上一次的路径
	QString filepath;
	filepath = s.value("OPEN_FILEPATH", ".ply").toString();// 不存在的话为当前应用程序路径
	filepath = QFileDialog::getOpenFileName(this, QStringLiteral("select file"), filepath);
	QTextCodec* code = QTextCodec::codecForName("GB2312");
	string selectedFile = code->fromUnicode(filepath.toStdString().c_str()).data();
	if (filepath.isEmpty())
		return;
	s.setValue("OPEN_FILEPATH", filepath); // 记住该路径，以备下次使用
	PointCloudT::Ptr tempcloud(new PointCloudT);
	object = tempcloud;
	filepath_source = selectedFile;
	if (pcl::io::loadPLYFile<PointNT>(filepath_source, *object) < 0) {
		pcl::console::print_error("Error loading object/scene file!\n");
		return;
	}
	cout << object->size() << endl;
	auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
	auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow2->AddRenderer(renderer2);
	viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2,"viewer", false));
	viewer->addPointCloud(object, ColorHandlerT(object, 0.0, 255.0, 0.0), "object");
	vtkSmartPointer<vtkRenderWindow> renderer = viewer->getRenderWindow();
	viewer->setupInteractor(ui.object->GetInteractor(), ui.object->GetRenderWindow());
	ui.object->SetRenderWindow(renderer);
	ui.object->repaint();

}

void RegistrationDemo::openSceneFile() {
	//QSettings s; // 为了记住上一次的路径
	//QString filepath;
	//filepath = s.value("OPEN_FILEPATH", ".ply").toString();// 不存在的话为当前应用程序路径
	//filepath = QFileDialog::getOpenFileName(this, QStringLiteral("select file"), filepath);
	////QTextCodec* code = QTextCodec::codecForName("GB2312");
	////string selectedFile = code->fromUnicode(filepath_source.toStdString().c_str()).data();
	//if (filepath.isEmpty())
	//	return;
	//s.setValue("OPEN_FILEPATH", filepath); // 记住该路径，以备下次使用
	//PointCloudT::Ptr tempcloud(new PointCloudT);
	//scene = tempcloud;
	//filepath_target = filepath.toStdString();
	//if (pcl::io::loadPLYFile<PointNT>(filepath_target, *scene) < 0) {
	//	pcl::console::print_error("Error loading object/scene file!\n");
	//	return;
	//}
	QSettings s; // 为了记住上一次的路径 
	QString filepath1;
	filepath1 = s.value("OPEN_FILEPATH", ".stl").toString();// 不存在的话为当前应用程序路径
	filepath1 = QFileDialog::getOpenFileName(this, QStringLiteral("select file"), filepath1);
	PointCloudT::Ptr tempcloud(new PointCloudT);
	scene = tempcloud;
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileSTL(filepath1.toStdString(), mesh);
	filepath_target = filepath1.toStdString();
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata);  //将PolygonMesh对象转化为vtkPolyData对象 
	pcl::io::vtkPolyDataToPointCloud(polydata, *scene);

	cout << scene->size() << endl;
	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
	viewer->addPointCloud(scene, ColorHandlerT(scene, 255.0, 255.0, 255.0), "scene");
	vtkSmartPointer<vtkRenderWindow> renderer1 = viewer->getRenderWindow();
	viewer->setupInteractor(ui.scene->GetInteractor(), ui.scene->GetRenderWindow());
	ui.scene->SetRenderWindow(renderer1);
	ui.scene->repaint();

}

void RegistrationDemo::showResault() {
	cout << "show resault" << endl;
	//pcl::transformPointCloud(*object, *object, transformeMatrix);
	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
	viewer->addPointCloud(object_aligned, ColorHandlerT(object_aligned, 255.0, 0.0, 0.0), "object_aligned");
	viewer->addPointCloud(scene, ColorHandlerT(scene, 255.0, 255.0, 255.0), "scene");
	vtkSmartPointer<vtkRenderWindow> renderer1 = viewer->getRenderWindow();
	viewer->setupInteractor(ui.object_aligned->GetInteractor(), ui.object_aligned->GetRenderWindow());
	ui.object_aligned->SetRenderWindow(renderer1);
	ui.object_aligned->repaint();
}

void RegistrationDemo::startAlign() {
	if (filepath_source == "" || filepath_target == "") {
		cout << "请先导入源点云和目标点云" << endl;
		return;
	}
	myRegistration* reg = new myRegistration();
	QThread* thread = new QThread();
	object_aligned = PointCloudT::Ptr(new PointCloudT);
	connect(thread, &QThread::started, reg, &myRegistration::Run);
	connect(reg, SIGNAL(progress(int)), this, SLOT(setProgressBarValue(int)));
	connect(reg, SIGNAL(finish()), this, SLOT(showResault()));// 执行完成，析构worker
	connect(reg, SIGNAL(emitRegResault(QString)), this, SLOT(showRegResault(QString)));
	connect(reg, SIGNAL(finish()), reg, SLOT(deleteLater()));// 执行完成，析构worker
	connect(reg, SIGNAL(destroyed(QObject*)), thread, SLOT(quit()));// 析构worker 完成， 推出线程
	connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
	Eigen::Matrix4d transMatrix;
	transMatrix << ui.nx->value(), ui.ox->value(), ui.ax->value(), ui.px->value(),
		ui.ny->value(), ui.oy->value(), ui.ay->value(), ui.py->value(),
		ui.nz->value(), ui.oz->value(), ui.az->value(), ui.pz->value(),
		ui.b1->value(), ui.b2->value(), ui.b3->value(), ui.b4->value(),
	reg->setObject(filepath_source);
	reg->setScene(filepath_target);
	reg->setParam(ui.overlapbox->value(), ui.deltabox->value());
	reg->setPresetTransformeMatrix(transMatrix);
	reg->ChangeRemoval(ui.isremoval->isChecked());
	reg->ChangeFilter(ui.isfilter->isChecked());
	reg->ChangeMethod(ui.isremoval_2);
	reg->setOutput(object_aligned, transformeMatrix);
	//reg->align();
	reg->moveToThread(thread); // 把worker 移动到线程
	thread->start(); // 开始线程
};

void RegistrationDemo::setProgressBarValue(int x) {
}
void RegistrationDemo::startTest() {
	if (filepath_source == "" || filepath_target == "") {
		cout << "请先导入源点云和目标点云" << endl;
		return;
	}
	vector<double> params = { ui.min_overlap->value(),ui.max_overlap->value(),ui.overlap_step->value() };
	vector<double> delta = { ui.min_delta->value(),ui.max_dela->value(),ui.delta_step->value() };
	myRegistration* reg = new myRegistration();
	QThread* thread = new QThread();
	object_aligned = PointCloudT::Ptr(new PointCloudT);
	connect(thread, &QThread::started, reg, &myRegistration::paramSwap);
	connect(reg, SIGNAL(progress(int)), this, SLOT(setProgressBarValue(int)));
	connect(reg, SIGNAL(finish()), this, SLOT(showResault()));
	connect(reg, SIGNAL(emitTestResault(QString)), this, SLOT(showTestResault(QString)));
	connect(reg, SIGNAL(finish()), reg, SLOT(deleteLater()));// 执行完成，析构worker
	connect(reg, SIGNAL(destroyed(QObject*)), thread, SLOT(quit()));// 析构worker 完成， 推出线程
	connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
	Eigen::Matrix4d transMatrix;
	transMatrix << ui.nx->value(), ui.ox->value(), ui.ax->value(), ui.px->value(),
		ui.ny->value(), ui.oy->value(), ui.ay->value(), ui.py->value(),
		ui.nz->value(), ui.oz->value(), ui.az->value(), ui.pz->value(),
		ui.b1->value(), ui.b2->value(), ui.b3->value(), ui.b4->value(),
	reg->setObject(filepath_source);
	reg->setScene(filepath_target);
	reg->setParam(ui.overlapbox->value(), ui.deltabox->value());
	reg->setPresetTransformeMatrix(transMatrix);
	reg->ChangeRemoval(ui.isremoval->isChecked());
	reg->ChangeFilter(ui.isfilter->isChecked());
	reg->ChangeMethod(ui.isremoval_2);
	reg->setOutput(object_aligned, transformeMatrix);
	reg->setTestParam(params, delta);
	reg->moveToThread(thread); // 把worker 移动到线程
	thread->start(); // 开始线程
}

void  RegistrationDemo::colorPointCloud(pcl::PointCloud<PointNT>::Ptr cloud, vector<int> rgb) {
	for (int i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].r = rgb[0];
		cloud->points[i].g = rgb[1];
		cloud->points[i].b = rgb[2];
	}
}

void RegistrationDemo::showTestResault(QString text) {
	ui.textBrowser->append(text);
}

void RegistrationDemo::showRegResault(QString text) {
	ui.textBrowser->append(text);
}

void RegistrationDemo::on_isremoval_clicked() {

}
void RegistrationDemo::on_isfilter_clicked() {

}

RegistrationDemo::~RegistrationDemo()
{}

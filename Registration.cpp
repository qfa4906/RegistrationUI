#include "Registration.h"

myRegistration::myRegistration(QObject* parent) : QObject(parent) {

}

myRegistration::~myRegistration() {

}

void myRegistration::setObject(string cloudpath) {
	this->filepath_source = cloudpath;
}

void myRegistration::setScene(string cloudpath) {
	this->filepath_target = cloudpath;
}

void myRegistration::setParam(double overlap, double delta) {
	this->_overlap = overlap;
	this->_delta = delta;
}

void myRegistration::setOutput(PointCloudT::Ptr cloud, Eigen::Matrix4f& matrix) {
	this->object_icp_aligned = cloud;
	this->transformeMatrix = matrix;
}

void myRegistration::ChangeRemoval(bool set) {
	this->useRemoval = set;
}

void myRegistration::ChangeFilter(bool set) {
	this->useFilter = set;
}

void myRegistration::ChangeMethod(bool set) {
	this->use4PCS = set;
}

void myRegistration::setPresetTransformeMatrix(Eigen::Matrix4d Matrix) {
	this->presetTransformeMatrix = Matrix;
}

void myRegistration::align() {
	/*PointCloudT object_cloud;
	PointCloudT object_aligned_cloud;
	PointCloudT scene_cloud;*/
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);
	/*PointCloudT::Ptr object; 
	*object = object_cloud;
	PointCloudT::Ptr object_aligned;
	*object_aligned = object_aligned_cloud;
	PointCloudT::Ptr scene;
	*scene = scene_cloud;*/
	//PointCloudT::Ptr object_icp_aligned(new PointCloudT);
	if (true) {
		pcl::console::print_highlight("Loading point clouds...\n");
		if (pcl::io::loadPLYFile<PointNT>(filepath_source, *object) < 0)
		{
			pcl::console::print_error("Error loading object/scene file!\n");
			return;
		}
		PointCloudT::Ptr tempcloud(new PointCloudT);
		scene = tempcloud;
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFileSTL(filepath_target, mesh);
		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		pcl::io::mesh2vtk(mesh, polydata);  //将PolygonMesh对象转化为vtkPolyData对象 
		pcl::io::vtkPolyDataToPointCloud(polydata, *scene);
	}
	else {
		pcl::console::print_highlight("Loading point clouds...\n");
		if (pcl::io::loadPLYFile<PointNT>(filepath_source, *object) < 0 || pcl::io::loadPLYFile<PointNT>(filepath_target, *scene) < 0)
		{
			pcl::console::print_error("Error loading object/scene file!\n");
			return;
		}
	}
	
	
	Eigen::Matrix4d* transformation_matrix = new Eigen::Matrix4d;
	*transformation_matrix = this->presetTransformeMatrix;
	pcl::transformPointCloud(*object, *object, this->presetTransformeMatrix);
	
	if (useRemoval) {
		statistical_removal(scene, scene);
		statistical_removal(object, object);
		cout << "w/ removal" << endl;
	}
	else {
		cout << "w/o removal" << endl;
	}

	if (useFilter) {
		grid_filter(scene, scene);
		grid_filter(object, object);
		cout << "w/ filter" << endl;
	}
	else {
		cout << "w/o filter" << endl;
	}
	cout << "\nFinal source file size: " << filepath_source << " (" << object->size() << " points" << endl;
	cout << "\nFinal target file size: " << filepath_target << " (" << scene->size() << " points" << endl;
	cout << "preset matrix:" << endl << this->presetTransformeMatrix << endl << endl << endl;
	if (this->use4PCS) {
		pcl::Super4PCS<PointNT, PointNT> align;
		align.setInputSource(object);
		align.setInputTarget(scene);
		align.setOverlap(_overlap);
		align.setDelta(_delta);
		cout << "start align...using parameter: " << _overlap << ", " << _delta << endl;
		align.align(*object_aligned);
		if (align.hasConverged())
		{
			cout << "success." << endl;
			transformeMatrix = align.getFinalTransformation();
		}
		else
			cout << "fail" << endl;
	}
	else {
		object_aligned = object;
	}


	pcl::IterativeClosestPoint<PointNT, PointNT> icp;
	icp.setMaximumIterations(200);
	icp.setMaxCorrespondenceDistance(1);
	//icp.setTransformationEpsilon(1e-10);
	//icp.setEuclideanFitnessEpsilon(1e-4); // convergence contidion is that MSE is smaller than threshold
	icp.setInputSource(object_aligned);
	icp.setInputTarget(scene);
	icp.align(*object_icp_aligned);

	if (icp.hasConverged())
	{
		cout << "ICP has converged\n";
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return;
	}
	transformeMatrix = icp.getFinalTransformation() * transformeMatrix;
	cout << "final matrix:" << endl << transformeMatrix << endl << endl << endl;
	savePointCloud(object_icp_aligned, "mesh_aligned.ply");
	QString text;
	text.append(QStringLiteral("Registration Matrix: "));
	text.append("\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			QString tmp = QString::number(double(transformeMatrix(i, j)));
			text.append(tmp);
			text.append(",\t");
		}
		text.append("\n");
	}
	emit emitRegResault(text);
	emit finish();
}

void myRegistration::Run() {
	this->align();
}

void myRegistration::fillVector(vector<double>& vec, double begin, double end, double block) {
	for (double i = begin; i < end; i += block) {
		vec.push_back(i);
	}
}

void myRegistration::savePointCloud(pcl::PointCloud<PointNT>::Ptr cloud, std::string outpath)
{
	std::cerr << "save path is :" << outpath << endl;
	//��string����·��תΪchar*
	char* path = new char[outpath.size() + 1];
	strcpy(path, outpath.c_str());
	std::cerr << "Path is : " << path << " ." << std::endl;

	//д������ͼ
	pcl::PLYWriter writer;
	writer.write(path, *cloud, true);
	std::cerr << "PointCloud has : " << cloud->width * cloud->height << " data points." << std::endl;
}

void myRegistration::statistical_removal(pcl::PointCloud<PointNT>::Ptr cloud, pcl::PointCloud<PointNT>::Ptr cloud_filtered_inliers) {
	pcl::StatisticalOutlierRemoval<PointNT> sor;   // �����˲�������
	sor.setKeepOrganized(false);
	sor.setInputCloud(cloud);                            // ���ô��˲��ĵ���
	sor.setMeanK(20);                                    // �����ڽ���ͳ��ʱ���ǲ�ѯ�ٽ�����
	sor.setStddevMulThresh(1); // �����ж��Ƿ�Ϊ��Ⱥ�����ֵ������Ϊ1����һ����ľ��볬��ƽ������һ����׼�����ϣ��򱻱��Ϊ��Ⱥ�㣬�������Ƴ�
	sor.filter(*cloud_filtered_inliers);                         // ִ���˲�������������浽cloud_filtered_inliers

}

void myRegistration::grid_filter(pcl::PointCloud<PointNT>::Ptr cloud, pcl::PointCloud<PointNT>::Ptr cloud_filtered)
{
	pcl::PointXYZRGB pMin, pMax;
	pcl::getMinMax3D(*cloud, pMin, pMax);
	double maxlen = min((pMax.x - pMin.x), min((pMax.y - pMin.y), (pMax.z - pMin.z)));
	double step = 0.5f;//maxlen / 150;
	pcl::VoxelGrid<PointNT> sor;     // �����˲�����
	sor.setInputCloud(cloud);              // ���˲�����������Ҫ���˵ĵ���
	sor.setLeafSize(step, step, step);  // �����˲�ʱ���������ش�СΪ1cm������
	sor.filter(*cloud_filtered);           // ִ���˲��������洢�����cloud_filtered
	return;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> myRegistration::customHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 255, 255);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

vector<double> myRegistration::paramSwap() {
	vector<double> pvec = this->testOverlap;
	vector<double> dvec = this->testDelta;
	pcl::PointCloud<PointNT>::Ptr object(new pcl::PointCloud<PointNT>);
	pcl::PointCloud<PointNT>::Ptr scene(new pcl::PointCloud<PointNT>);
	if (true) {
		pcl::console::print_highlight("Loading point clouds...\n");
		if (pcl::io::loadPLYFile<PointNT>(filepath_source, *object) < 0)
		{
			pcl::console::print_error("Error loading object/scene file!\n");
		}
		PointCloudT::Ptr tempcloud(new PointCloudT);
		scene = tempcloud;
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFileSTL(filepath_target, mesh);
		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		pcl::io::mesh2vtk(mesh, polydata);  //将PolygonMesh对象转化为vtkPolyData对象 
		pcl::io::vtkPolyDataToPointCloud(polydata, *scene);
	}
	else {
		if (pcl::io::loadPLYFile<PointNT>(filepath_source, *object) < 0 || pcl::io::loadPLYFile<PointNT>(filepath_target, *scene) < 0)
		{
			pcl::console::print_error("Error loading object/scene file!\n");
		}
	}
	vector<double> params, deltas;
	double best_param = 0, best_delta = 0, score = 0;
	pcl::PointCloud<PointNT>::Ptr object_aligned(new pcl::PointCloud<PointNT>);
	fillVector(params, pvec[0], pvec[1], pvec[2]);
	fillVector(deltas, dvec[0], dvec[1], dvec[2]);
	if (useRemoval) {
		cout << "w/ removal" << endl;
		statistical_removal(scene, scene);
		statistical_removal(object, object);
	}
	else {
		cout << "w/o removal" << endl;
	}

	if (useFilter) {
		cout << "w/ filter" << endl;
		grid_filter(scene, scene);
		grid_filter(object, object);
	}
	else {
		cout << "w/o filter" << endl;
	}
	int i = 0;
	QString text; 
	text.append(QStringLiteral("Parameter Test Resault:"));
	text.append("\n");
	text.append("\t");
	for (auto param : params) {
		vector<double> params_array;
		for (auto delta : deltas) {
			if (param == params[0])
			{
				QString tmp = QString::number(delta);
				text.append(tmp);
				text.append("\t");
			}
			cout << i++ << "/" << params.size() * deltas.size() << endl;
			pcl::Super4PCS<PointNT, PointNT> align;
			align.setInputSource(object);
			align.setInputTarget(scene);
			align.setOverlap(param);
			align.setDelta(delta);
			align.setSampleSize(200);
			align.align(*object_aligned);
			if (score < align.getFitnessScore()) {
				score = align.getFitnessScore();
				best_param = param;
				best_delta = delta;
			}
			/*if (score == 1) {
				cout << "find best param" << endl;
				return vector<double>{ best_param, best_delta };
			}*/
			params_array.push_back(align.getFitnessScore());
		}
		outputArray.push_back(params_array);
	}
	text.append("\n");
	for (int i = 0; i < outputArray.size(); i++) {
		for (int j = 0; j < outputArray[0].size(); j++) {
			cout << outputArray[i][j] << ", ";
		}
		cout << endl;
	}
	
	for (int i = 0; i < outputArray.size(); i++) {
		QString tmp1 = QString::number(params[i]);
		text.append(tmp1);
		text.append(":\t");
		for (int j = 0; j < outputArray[0].size(); j++) {
			QString tmp = QString::number(outputArray[i][j]);
			text.append(tmp);
			text.append(",\t");
		}
		text.append("\n");
	}
	vector<double> res = { best_param, best_delta };
	emit emitTestResault(text);
	return res;
}
void fillVector(vector<double>& vec, double begin, double end, double block) {
	for (double i = begin; i < end; i += block) {
		vec.push_back(i);
	}
}

void myRegistration::setTestParam(vector<double> v1, vector<double> v2) {
	this->testOverlap = v1;
	this->testDelta = v2;
}
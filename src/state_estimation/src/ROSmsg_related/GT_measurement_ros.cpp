#include "GT_measurement_ros.h"

GT_measurement::GT_measurement(ros::NodeHandle& nh_, int id, int mavnum)
{
    nh = nh_;
    ID = id;
    self_index = ID-1;
    mavNum = mavnum;
    formation_num = mavNum-1;

	/*=================================================================================================================================
		groundtruth
	=================================================================================================================================*/
  	groundTruth_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 30, &GT_measurement::groundTruth_cb, this);
	GTs_rate = 500;
	GTs_count = 0;
	GTs = new MAV[mavNum];

	/*=================================================================================================================================
        Lidar, position
    ===============================================================================================================================*/
	lidar_rate = 50;
	position_rate = 10;

	/*=================================================================================================================================
        Camera boundingBox
    =================================================================================================================================*/	
    bboxes_sub = nh.subscribe<std_msgs::Float64MultiArray>("synchronizer/yolov8/boundingBox", 2, &GT_measurement::bboxes_cb, this);
	bbox_count = 0;
	no_bbox_count = 0;
	checkCount = 0;
	bbox_eigen_past << 320, 240, 4;
	bbox_eigen = bbox_eigen_past;
	gotBbox  = false;
}

GT_measurement::~GT_measurement()
{
    delete[] GTs;
}

void GT_measurement::setRosRate(int rate)
{
	rosRate = rate;
}

/*=================================================================================================================================
    groundtruth
=================================================================================================================================*/

void GT_measurement::groundTruth_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    GTs_count++; // GroundTruth call back rate = 500hz

    ////////////////////////// get groundTruth model states and arrange their ID////////////////////
    std::vector<string> name = msg->name;
    
    // 初始化映射表示每個名稱應該對應的索引，跳過 ground_plane
    std::map<string, int> name_to_index;
    name_to_index["tb_1"] = 1;
    name_to_index["tb_2"] = 2;
    name_to_index["tb_3"] = 3;
    name_to_index["target_1"] = 0;
    name_to_index["target_2"] = 4;
    
    for(int i = 0; i < name.size(); i++)
    {
        if(name_to_index.find(name[i]) != name_to_index.end())
        {
            int target_index = name_to_index[name[i]];
            GTs[target_index].setPose(msg->pose[i]);
            GTs[target_index].setTwist(msg->twist[i]);
        }
    }
    
    /*  @ Now we have the groundtruth of all UAVs and targets (without ground_plane):
        @ GTs[0]: target_1
        @ GTs[1]: tb_1
        @ GTs[2]: tb_2
        @ GTs[3]: tb_3
        @ GTs[4]: target_2 */
    
    GTs_eigen = mavsMsg2Eigen(GTs, name_to_index.size());
    
    // 修正：只取 tb_1, tb_2, tb_3，它們現在是索引 1, 2, 3
    std::vector<MAV_eigen> formation_eigen_GT;
    formation_eigen_GT.push_back(GTs_eigen[1]); // tb_1
    formation_eigen_GT.push_back(GTs_eigen[2]); // tb_2
    formation_eigen_GT.push_back(GTs_eigen[3]); // tb_3
    
    // for(int i = 0; i < GTs_eigen.size(); i++) {
    //     printf("GTs_eigen[%d]: \n [%f, %f]\n", i, GTs_eigen[i].r(0), GTs_eigen[i].r(1));
    // }

    ////////////////////////// Transform from groundtruth to measurements,  ////////////////////////
    static std::default_random_engine generator;
    if(GTs_count % (GTs_rate/lidar_rate) == 0) 
    {
        lidarMeasurements = lidarMeasure(formation_eigen_GT, generator);
        
        // 修正：target_1 現在是 GTs_eigen[0]
        lidar4target = lidarmeasure4target(formation_eigen_GT, GTs_eigen[0], generator);
        CameraModel = Camera4Neighbor(formation_eigen_GT, generator);
        CameraModel4target_1 = CameraMeasure4target_1(formation_eigen_GT, GTs_eigen[0], generator);
        
        // target_2 仍然是 GTs_eigen[4]
        CameraModel4target_2 = CameraMeasure4target_2(formation_eigen_GT, GTs_eigen[4], generator);
    }
    
    // 修正：如果 ID 是 1-3 範圍內的，則需要轉換為 GTs_eigen 的索引
    if(GTs_count % (GTs_rate/position_rate) == 0) {
        // ID 從 1 開始，而 tb_1 在 GTs_eigen 中的索引是 1，所以使用 ID 即可
        positionMeasurement = positionMeasure(GTs_eigen[ID], generator);
    }
    
    if(GTs_count == GTs_rate)
        GTs_count = 0;
}

std::vector<MAV_eigen> GT_measurement::getGTs_eigen(){return GTs_eigen;}
geometry_msgs::Quaternion GT_measurement::getGTorientation(int ID){return GTs[ID].getPose().pose.orientation;}


/*=================================================================================================================================
    Lidar, position
===============================================================================================================================*/

std::vector<Eigen::Vector4d> GT_measurement::lidarMeasure(std::vector<MAV_eigen> formation_GT, std::default_random_engine generator)
{
	Eigen::Vector4d measurement;
	std::vector<Eigen::Vector4d> measurements;
	Eigen::Vector3d r_ns_B;
	Eigen::Matrix3d R_W2B = formation_GT[self_index].R_w2b;
	for(int i=0; i<formation_num; i++)
	{
		if(i != self_index)
		{
			r_ns_B = R_W2B*(formation_GT[i].r - formation_GT[self_index].r);

			measurement(0) = sqrt(pow(r_ns_B(0), 2) + pow(r_ns_B(1), 2) + pow(r_ns_B(2), 2));
			measurement(1) = acos(r_ns_B(2)/measurement(0)); // theta
			measurement(2) = atan2(r_ns_B(1), r_ns_B(0)); // phi
			measurement(3) = i+1; // ID

			std::normal_distribution<double> n_D(0.0, 0.02);
			std::normal_distribution<double> n_theta(0.0, 0.035);
			std::normal_distribution<double> n_phi(0.0, 0.035);
			measurement(0) += n_D(generator);
			measurement(1) += n_theta(generator);
			measurement(2) += n_phi(generator);

			measurements.push_back(measurement);
		}
	}
	return measurements;
}
Eigen::Vector3d GT_measurement::lidarmeasure4target(std::vector<MAV_eigen> formation_GT,MAV_eigen target_eigen, std::default_random_engine generator)
{
	Eigen::Vector3d measurement;
	Eigen::Vector3d r_ns_B;
	Eigen::Matrix3d R_W2B = formation_GT[self_index].R_w2b;

	r_ns_B = R_W2B*(target_eigen.r - formation_GT[self_index].r);

	measurement(0) = sqrt(pow(r_ns_B(0), 2) + pow(r_ns_B(1), 2) + pow(r_ns_B(2), 2));
	measurement(1) = acos(r_ns_B(2)/measurement(0)); // theta
	measurement(2) = atan2(r_ns_B(1), r_ns_B(0)); // phi

	std::normal_distribution<double> n_D(0.0, 0.02);
	std::normal_distribution<double> n_theta(0.0, 0.035);
	std::normal_distribution<double> n_phi(0.0, 0.035);
	measurement(0) += n_D(generator);
	measurement(1) += n_theta(generator);
	measurement(2) += n_phi(generator);		
		
	return measurement;
}
Eigen::Vector3d GT_measurement::positionMeasure(MAV_eigen GT_eigen, std::default_random_engine generator)
{
	Eigen::Vector3d measurement = GT_eigen.r;

	std::normal_distribution<double> n_x(0.0, 0.05);
	std::normal_distribution<double> n_y(0.0, 0.05);
	std::normal_distribution<double> n_z(0.0, 0.05);
	measurement(0) += n_x(generator);
	measurement(1) += n_y(generator);
	measurement(2) += n_z(generator);

	return measurement;
}

std::vector<Eigen::Vector4d> GT_measurement::getLidarMeasurements(){return lidarMeasurements;}
Eigen::Vector3d GT_measurement::getlidar4target(){return  lidar4target;}
Eigen::Vector3d GT_measurement::getPositionMeasurement(){return positionMeasurement;}
/*=================================================================================================================================
    Camera model
=================================================================================================================================*/

std::vector<Eigen::Vector4d> GT_measurement::Camera4Neighbor(std::vector<MAV_eigen> formation_GT,std::default_random_engine generator)
{	
	double fx = 1029.477219320806;
    double fy = 1029.477219320806;
	double cx = 960.5;
	double cy = 540.5;
	double X,Y,Z;
	Eigen::Vector4d measurement;
	std::vector<Eigen::Vector4d> measurements;

	Eigen::Matrix3d R_b2c ;
	 R_b2c << 0, 1, 0,
			0, 0, 1,
			1, 0, 0;
	Eigen::Matrix3d R_w2c = R_b2c*formation_GT[self_index].R_w2b; ///////////////// rotation problem
	Eigen::Vector3d r_qc_c;
	for(int i=0; i<formation_num; i++)
	{
		if(i != self_index)
		{
			r_qc_c= R_w2c*(formation_GT[i].r - formation_GT[self_index].r); 

			X = r_qc_c(0)/r_qc_c(2);
			Y = r_qc_c(1)/r_qc_c(2);
			Z = r_qc_c(2);

			measurement(0) = fx*X + cx;
			measurement(1) = fy*Y + cy ;
			measurement(2) = Z;
			measurement(3) = i+1; // ID

			measurements.push_back(measurement);
		}
	}
	return measurements;

}
Eigen::Vector3d GT_measurement::CameraMeasure4target_1(std::vector<MAV_eigen> formation_GT,MAV_eigen target_eigen, std::default_random_engine generator)
{	

	Eigen::Vector3d measurement;
	Eigen::Matrix3d R_b2c ;
	R_b2c = cam.R_B2C();
	Eigen::Matrix3d R_w2c = R_b2c*formation_GT[self_index].R_w2b; 				// rotation problem
	Eigen::Vector3d r_qc_c = R_w2c*(target_eigen.r - formation_GT[self_index].r - cam.t_B2C()); 
	// printf("target_1 pose in camera: %f, %f, %f\n", target_eigen.r(0), target_eigen.r(1), target_eigen.r(2));

	double X = r_qc_c(0)/r_qc_c(2);
	double Y = r_qc_c(1)/r_qc_c(2);
	double Z = r_qc_c(2);

	measurement(0) = cam.fx()*X + cam.cx();
	measurement(1) = cam.fy()*Y + cam.cy() ;
	measurement(2) = Z;
	// Now measurement is z_{Tij} = [u, v, z]

	return measurement;
}
Eigen::Vector3d GT_measurement::CameraMeasure4target_2(std::vector<MAV_eigen> formation_GT,MAV_eigen target_eigen, std::default_random_engine generator)
{	

	Eigen::Vector3d measurement;
	Eigen::Matrix3d R_b2c ;
	R_b2c = cam.R_B2C();
	Eigen::Matrix3d R_w2c = R_b2c*formation_GT[self_index].R_w2b; 				// rotation problem
	Eigen::Vector3d r_qc_c = R_w2c*(target_eigen.r - formation_GT[self_index].r - cam.t_B2C()); 
	// printf("target_2 pose in camera: %f, %f, %f\n", target_eigen.r(0), target_eigen.r(1), target_eigen.r(2));

	double X = r_qc_c(0)/r_qc_c(2);
	double Y = r_qc_c(1)/r_qc_c(2);
	double Z = r_qc_c(2);

	measurement(0) = cam.fx()*X + cam.cx();
	measurement(1) = cam.fy()*Y + cam.cy() ;
	measurement(2) = Z;
	// Now measurement is z_{Tij} = [u, v, z]

	return measurement;
}
void GT_measurement::setCamera(Camera camera)
{
	cam = camera;
}
std::vector<Eigen::Vector4d>GT_measurement::getCameraNeighbor(){return CameraModel;}
Eigen::Vector3d GT_measurement::getCamera4target_1(){return  CameraModel4target_1;}
Eigen::Vector3d GT_measurement::getCamera4target_2(){return  CameraModel4target_2;}
/*=================================================================================================================================
    Camera boundingBox
=================================================================================================================================*/
void GT_measurement::bboxes_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    bboxes_raw = msg->data;
	
	std::vector<Eigen::Vector3d> bboxes;
	if(bboxes_raw.size() > 3)
	{
		double min_dist = 99999;
		for(size_t i=0; i<bboxes_raw.size(); i+=3)
		{
			Eigen::Vector3d bbox(bboxes_raw[i], bboxes_raw[i+1], bboxes_raw[i+2]);
			bboxes.push_back(bbox);
		}
		for(auto& bbox : bboxes)
		{
			double dist = sqrt(pow(bbox(0) - bbox_eigen_past(0), 2) + pow(bbox(1) - bbox_eigen_past(1), 2));
			if(dist < min_dist)
			{
				min_dist = dist;
				bbox_eigen = bbox;
			}
		}
	}
	else if(bboxes_raw.size() == 3)
			bbox_eigen << bboxes_raw[0], bboxes_raw[1], bboxes_raw[2];
	no_bbox_count = 0;
}

bool GT_measurement::ifCameraMeasure(){return gotBbox;}
void GT_measurement::bbox_check()
{
	if(!gotBbox)
	{
		no_bbox_count = 0;
		checkCount++;
		if(checkCount == rosRate)
		{
			checkCount = 0;
			bbox_count = 0;
			gotBbox = false;
		}
		if(bbox_eigen != bbox_eigen_past)
		{
			bbox_count++;
			if(bbox_count == 10)
			{
				bbox_count = 0;
				checkCount = 0;
				gotBbox = true;
			}
		}
	}
	else
	{
		if(bbox_eigen == bbox_eigen_past)
		{
			no_bbox_count++;
			if(no_bbox_count == rosRate)
				gotBbox = false;
		}
	}
	bbox_eigen_past = bbox_eigen;
}

Eigen::Vector3d GT_measurement::getBboxEigen(){return bbox_eigen;}
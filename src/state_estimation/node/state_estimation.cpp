#include <string>
#include <sstream>
#include <vector>
#include <cctype>
#include <cmath>
#include <random>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>  
#include <ros/ros.h>
#include "ros/param.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <state_estimation/EIFpairStamped.h>
#include <state_estimation/Plot.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelStates.h>

#include "Mav.h"
#include "TEIF_Lidar.h"
#include "TEIF.h"
#include "HEIF_self.h"
#include "HEIF_target.h"
#include "SEIF_pose.h"
#include "SEIF_neighbors.h"
#include "SEIF_lidar_neighbors.h"
#include "GT_measurement_ros.h"
#include "EIFpairs_ros.h"
#include "Camera.h"
#include "MathLib.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh;

	ros::Publisher mavros_fusionPose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
	ros::Publisher mavros_fusionTwist_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/vision_pose/twist", 10);
	// // single target
	// ros::Publisher target_fusionPose_pub = nh.advertise<geometry_msgs::PoseStamped>("THEIF/pose", 10);
	// ros::Publisher target_fusionTwist_pub = nh.advertise<geometry_msgs::TwistStamped>("THEIF/twist", 10);
	// multiple targets
	// ros::Publisher* target_fusionPose_pubs;
	// ros::Publisher* target_fusionTwist_pubs;

	ros::Publisher isTargetEst_pub = nh.advertise<std_msgs::Bool>("THEIF/isTargetEst", 10);

    std::string vehicle;
    bool consensus = false;
	bool position_estimation = false;
	int mavNum = 3;
	int targetNum = 2;
    int rosRate = 50;
	int ID = 0;
	int state_size = 6;
	double targetTimeTol = 0.05;
	double last_t;
	double dt;
    ros::param::get("vehicle", vehicle);
	ros::param::get("ID", ID);
    ros::param::get("rate", rosRate);
	ros::param::get("consensus", consensus);
	ros::param::get("stateSize", state_size);
	ros::param::get("targetNum", targetNum);
	ros::param::get("targetTimeTolerance", targetTimeTol);
	ros::param::get("pos_est", position_estimation);
	
	ros::Rate rate(rosRate);

	geometry_msgs::PoseStamped self_fusedPoseMsg;
	geometry_msgs::TwistStamped self_fusedTwistMsg;
	// geometry_msgs::PoseStamped* target_fusedPoseMsgs;
	// geometry_msgs::TwistStamped* target_fusedTwistMsgs;

	// target_fusedPoseMsgs = new geometry_msgs::PoseStamped[targetNum];
	// target_fusedTwistMsgs = new geometry_msgs::TwistStamped[targetNum];
	geometry_msgs::PoseStamped target_fusedPoseMsg_1;
	geometry_msgs::TwistStamped target_fusedTwistMsg_1;
	geometry_msgs::PoseStamped target_fusedPoseMsg_2;
	geometry_msgs::TwistStamped target_fusedTwistMsg_2;
	ROS_INFO("target Msgs created.");

	MAV mav(nh);
	MAV mav_t1(nh);
	MAV mav_t2(nh);
	EIFpairs_ros eif_ros(nh, vehicle, ID, mavNum, targetNum);
	Camera cam(nh, false);
	GT_measurement gt_m(nh, ID, mavNum+targetNum);
	gt_m.setRosRate(rosRate);
	MAV_eigen mav_eigen;
	MAV_eigen mav_eigen_t1;
	MAV_eigen mav_eigen_t2;

	// target_fusionPose_pubs = new ros::Publisher[targetNum];
	// target_fusionTwist_pubs = new ros::Publisher[targetNum];

	// for (int i = 0; i < targetNum; i++) {
	// 	std::string pose_topic = "THEIF/target_" + std::to_string(i+1) + "/pose";
	// 	std::string twist_topic = "THEIF/target_" + std::to_string(i+1) + "/twist";
		
	// 	target_fusionPose_pubs[i] = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
	// 	target_fusionTwist_pubs[i] = nh.advertise<geometry_msgs::TwistStamped>(twist_topic, 10);
	// }
	// ROS_INFO("target pubs created.");
	ros::Publisher target1_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("THEIF/target_1/pose", 10);
	ros::Publisher target1_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("THEIF/target_1/twist", 10);
	ros::Publisher target2_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("THEIF/target_2/pose", 10);
	ros::Publisher target2_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("THEIF/target_2/twist", 10);

	while(ros::ok())
	{
		if(mav.imu_init)
				break;
		else
			printf("[%s_%i]: Waiting for Imu topic...\n", vehicle.c_str(), ID);
		rate.sleep();
		ros::spinOnce();
	}
	printf("\n[%s_%i EIF]: Topic checked\n", vehicle.c_str(), ID);
	for(int i=0; i< 20; i++)
	{
		rate.sleep();
		ros::spinOnce();
	}
	
	Self_pose_EIF SEIF_pose;
	Self_rel_EIF SEIF_neighbors;
	Self_lidar_EIF SEIF_lidar_neighbors;

	// create multiple target objects
	std::vector<target_EIF> teif_objects;
	for (int i = 0; i < targetNum; i++)
	{
		target_EIF teif(state_size);
		teif_objects.push_back(teif);
	}

	HEIF_self sheif(6);
	// HEIF_target theif(6);

	// std::vector<HEIF_target> theif_objects;
	//
	// for (int i = 0; i < targetNum; i++)
	// {
	// 	HEIF_target theif(state_size);
	// 	theif_objects.push_back(theif);
	// }
	
	std::vector<std::unique_ptr<HEIF_target>> theif_objects;

	for (int i = 0; i < targetNum; i++) {
		theif_objects.push_back(std::make_unique<HEIF_target>(state_size));
	}

	printf("\n[%s_%i EIF]: EIF constructed\n\n", vehicle.c_str(), ID);

	SEIF_pose.setCurrState(gt_m.getGTs_eigen()[ID]);
	if(position_estimation)
	{
		Eigen::MatrixXd Q(6, 6);
		Q.block(0, 0, 3, 3) = 1e-3*Eigen::MatrixXd::Identity(3, 3); // position
    	Q.block(3, 3, 3, 3) = 8e-2*Eigen::MatrixXd::Identity(3, 3); // velocity
		SEIF_pose.set_process_noise(Q);
	}
	
	dt = 0.001;
	last_t = ros::Time::now().toSec();

	std_msgs::Bool isTargetEst_msg;
	
    while(ros::ok())
    {
		mav.setOrientation(gt_m.getGTorientation(ID));   // set orientation of robot
		mav_eigen = mavMsg2Eigen(mav); 					 // convert mav message to eigen format
		mav_t1.setOrientation(gt_m.getGTorientation(0)); // set orientation of target_1
		mav_eigen_t1 = mavMsg2Eigen(mav_t1); 			 // convert mav message to eigen format
		mav_t2.setOrientation(gt_m.getGTorientation(4)); // set orientation of target_2
		mav_eigen_t2 = mavMsg2Eigen(mav_t2);  			 // convert mav message to eigen format
		/*=================================================================================================================================
			Prediction
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		SEIF_pose.setMavSelfData(mav_eigen);
		if(position_estimation)
			SEIF_pose.setMeasurement(gt_m.getPositionMeasurement());
		SEIF_pose.computePredPairs(dt);
		eif_ros.selfPredEIFpairs_pub.publish(eigen2EifMsg(SEIF_pose.getEIFData(), ID));
		
		SEIF_lidar_neighbors.setMavSelfData(mav_eigen);
		SEIF_lidar_neighbors.setEIFpredData(SEIF_pose.getEIFData());
		SEIF_lidar_neighbors.setLidarMeasurements(gt_m.getLidarMeasurements());
		SEIF_lidar_neighbors.setNeighborData(eif_ros.get_curr_fusing_data(eif_ros.neighborsEIFpairs, 0.05));
		// -------------------------------------Target-------------------------------------
		gt_m.setCamera(cam);
		for (int i = 0; i < targetNum; i++)
		{
			teif_objects[i].setCamera(cam); // camera parameters, transformation matrix
			teif_objects[i].setMavSelfData(mav_eigen); // self state (robot state, not target) // why? -> for observe the measurement correction so the value of prediction isn't important 

			// set gt as prediction.
			Eigen::Vector3d initialBbox;
			if (i == 0) {
				initialBbox << gt_m.getGTs_eigen()[0].r;
			} else {
				initialBbox << gt_m.getGTs_eigen()[4].r;
			}
			teif_objects[i].setInitialState(initialBbox);

			if (i == 0){
				teif_objects[i].setMavSelfData(mav_eigen_t1);
				teif_objects[i].setMeasurement(gt_m.getCamera4target_1()); // camera measurement(u, v, d) for target_1
			}
			else{
				teif_objects[i].setMavSelfData(mav_eigen_t2);
				teif_objects[i].setMeasurement(gt_m.getCamera4target_2()); // camera measurement(u, v, d) for target_2
			}
			teif_objects[i].setSEIFpredData(SEIF_pose.getEIFData());
			teif_objects[i].computePredPairs(dt);
		}
		// std::cout << "Robot" << ID << " set Pred TEIF:\n" << "target_1:\n" << teif_objects[0].getTgtData().X_hat << "\n\n" << "target_2:\n" << teif_objects[1].getTgtData().X_hat << "\n\n";

		// teif.setCamera(cam);
		// teif.setMavSelfData(mav_eigen); 
		// teif.setMeasurement(gt_m.getCamera4target_1());
		// teif.setSEIFpredData(SEIF_pose.getEIFData());
		// teif.computePredPairs(dt);

		/*=================================================================================================================================
			Correction
		=================================================================================================================================*/
		
		// -------------------------------------Self-------------------------------------
		SEIF_pose.computeCorrPairs();
		SEIF_lidar_neighbors.computeCorrPairs();

		// -------------------------------------Target-------------------------------------
		for (int i = 0; i < targetNum; i++)
		{
			teif_objects[i].computeCorrPairs(); // compute correction pairs for each target: self.s, self.y, T.s, T.y, T.P, T.X
			eif_ros.self2TgtEIFpairs_pubs[i].publish(eigen2EifMsg(teif_objects[i].getTgtData(), ID)); // eigen2EifMsg: convert eigen to EIFpairStamped // for single target
		}

		// teif.computeCorrPairs();
		// eif_ros.self2TgtEIFpairs_pub.publish(eigen2EifMsg(teif.getTgtData(), ID));

		/*=================================================================================================================================
			Fusion
		=================================================================================================================================*/
		// -------------------------------------Self-------------------------------------
		sheif.setSelfEstData(SEIF_pose.getEIFData());
		sheif.setNeighborEstData(SEIF_lidar_neighbors.getEIFData());
		sheif.process();
		SEIF_pose.setFusionPairs(sheif.getFusedCov(), sheif.getFusedState());
		
		std::cout << "SEIF" << ID << ":\n";
		eif_ros.selfState_Plot_pub.publish(compare(gt_m.getGTs_eigen()[ID], sheif.getFusedState() , sheif.getFusedCov(), gt_m.getGTorientation(ID)));
		
		// -------------------------------------Target-------------------------------------
		// std::vector<EIF_data> allTgtEIFData;
		std::vector<std::vector<EIF_data>> allTgtEIFDatas(targetNum); // for multiple targets from multiple robots
		for (int i = 0; i < targetNum; i++)
		{
			allTgtEIFDatas[i] = eif_ros.get_curr_fusing_data(eif_ros.rbs2Tgt_EIFPairs, 0.05);
			allTgtEIFDatas[i].push_back(teif_objects[i].getTgtData());
			theif_objects[i]->setTargetEstData(allTgtEIFDatas[i]);
			theif_objects[i]->process();
			teif_objects[i].setFusionPairs(theif_objects[i]->getFusedCov(), theif_objects[i]->getFusedState(), ros::Time::now().toSec());
			// Compute density gradient & publish to /$(vehicle)_$(id)/densityGradient. //For coverageCtrl
			Eigen::MatrixXd gradient_M(2, 240*240);
			gradient_M.setZero();
			gradient_M = teif_objects[i].getGradientDensityFnc(theif_objects[i]->getFusedCov(), theif_objects[i]->getWeightedS(), theif_objects[i]->getWeightedY(), theif_objects[i]->getWeightedXi_hat(), theif_objects[i]->getEta_ij());
			eif_ros.densityGradient_pubs[i].publish(eigen2densityGradient(gradient_M));		

			std::cout << ID << " TEIF_" << i+1 << ":\n";
			if (i == 0)
				eif_ros.tgtState_Plot_pubs[i].publish(compare(gt_m.getGTs_eigen()[0], theif_objects[i]->getFusedState() , theif_objects[i]->getFusedCov(), gt_m.getGTorientation(ID)));
			else
				eif_ros.tgtState_Plot_pubs[i].publish(compare(gt_m.getGTs_eigen()[4], theif_objects[i]->getFusedState() , theif_objects[i]->getFusedCov(), gt_m.getGTorientation(ID)));
		}
		// allTgtEIFData = eif_ros.get_curr_fusing_data(eif_ros.rbs2Tgt_EIFPairs, 0.05);
		// allTgtEIFData.push_back(teif.getTgtData());
		// theif.setTargetEstData(allTgtEIFData);
		// theif.process();
		// teif.setFusionPairs(theif.getFusedCov(), theif.getFusedState(), ros::Time::now().toSec());

		// // Compute density gradient & publish to /$(vehicle)_$(id)/densityGradient. //For coverageCtrl
		// Eigen::MatrixXd gradient_M(2, 240*240);
		// gradient_M.setZero();
		// gradient_M = teif.getGradientDensityFnc(theif.getFusedCov(), theif.getWeightedS(), theif.getWeightedY(), theif.getWeightedXi_hat(), theif.getEta_ij());
		// // std::cout << "ros_g:\n" << eigen2densityGradient(gradient_M) << "\n";
		// eif_ros.densityGradient_pub.publish(eigen2densityGradient(gradient_M));		

		// std::cout << "TEIF:\n";
		// eif_ros.tgtState_Plot_pub.publish(compare(gt_m.getGTs_eigen()[0], theif.getFusedState() , theif.getFusedCov(), gt_m.getGTorientation(ID)));
	
		/*=================================================================================================================================
			Publish to mavros for feedback
		=================================================================================================================================*/
		
		// -------------------------------------Position-------------------------------------
		self_fusedPoseMsg.header.frame_id = "/world";
		self_fusedPoseMsg.header.stamp = ros::Time::now();
		self_fusedPoseMsg.pose.position.x = sheif.getFusedState()(0);
		self_fusedPoseMsg.pose.position.y = sheif.getFusedState()(1);
		self_fusedPoseMsg.pose.position.z = sheif.getFusedState()(2);
		self_fusedPoseMsg.pose.orientation.w = mav_eigen.q.w();
		self_fusedPoseMsg.pose.orientation.x = mav_eigen.q.x();
		self_fusedPoseMsg.pose.orientation.y = mav_eigen.q.y();
		self_fusedPoseMsg.pose.orientation.z = mav_eigen.q.z();

		// target_fusedPoseMsg.header.frame_id = "/world";
		// target_fusedPoseMsg.header.stamp = ros::Time::now();
		// target_fusedPoseMsg.pose.position.x = theif.getFusedState()(0);
		// target_fusedPoseMsg.pose.position.y = theif.getFusedState()(1);
		// target_fusedPoseMsg.pose.position.z = theif.getFusedState()(2);
		// for (int i = 0; i < targetNum; i++)
		// {
		// 	target_fusedPoseMsgs[i].header.frame_id = "/world";
		// 	target_fusedPoseMsgs[i].header.stamp = ros::Time::now();
		// 	target_fusedPoseMsgs[i].pose.position.x = theif_objects[i]->getFusedState()(0);
		// 	target_fusedPoseMsgs[i].pose.position.y = theif_objects[i]->getFusedState()(1);
		// 	target_fusedPoseMsgs[i].pose.position.z = theif_objects[i]->getFusedState()(2);
		// }
		target_fusedPoseMsg_1.header.frame_id = "/world";
		target_fusedPoseMsg_1.header.stamp = ros::Time::now();
		target_fusedPoseMsg_1.pose.position.x = theif_objects[0]->getFusedState()(0);
		target_fusedPoseMsg_1.pose.position.y = theif_objects[0]->getFusedState()(1);
		target_fusedPoseMsg_1.pose.position.z = theif_objects[0]->getFusedState()(2);

		target_fusedPoseMsg_2.header.frame_id = "/world";
		target_fusedPoseMsg_2.header.stamp = ros::Time::now();
		target_fusedPoseMsg_2.pose.position.x = theif_objects[1]->getFusedState()(0);
		target_fusedPoseMsg_2.pose.position.y = theif_objects[1]->getFusedState()(1);
		target_fusedPoseMsg_2.pose.position.z = theif_objects[1]->getFusedState()(2);

		// -------------------------------------Velocity-------------------------------------
		self_fusedTwistMsg.header.stamp = ros::Time::now();
		self_fusedTwistMsg.twist.linear.x = sheif.getFusedState()(3);
		self_fusedTwistMsg.twist.linear.y = sheif.getFusedState()(4);
		self_fusedTwistMsg.twist.linear.z = sheif.getFusedState()(5);
		self_fusedTwistMsg.twist.angular.x = mav_eigen.omega_c(0);
		self_fusedTwistMsg.twist.angular.y = mav_eigen.omega_c(1);
		self_fusedTwistMsg.twist.angular.z = mav_eigen.omega_c(2);

		// target_fusedTwistMsg.header.stamp = ros::Time::now();
		// target_fusedTwistMsg.twist.linear.x = theif.getFusedState()(3);
		// target_fusedTwistMsg.twist.linear.y = theif.getFusedState()(4);
		// target_fusedTwistMsg.twist.linear.z = theif.getFusedState()(5);
		// for (int i = 0; i < targetNum; i++)
		// {
		// 	target_fusedTwistMsgs[i].header.stamp = ros::Time::now();
		// 	target_fusedTwistMsgs[i].twist.linear.x = theif_objects[i]->getFusedState()(3);
		// 	target_fusedTwistMsgs[i].twist.linear.y = theif_objects[i]->getFusedState()(4);
		// 	target_fusedTwistMsgs[i].twist.linear.z = theif_objects[i]->getFusedState()(5);
		// }
		target_fusedTwistMsg_1.header.stamp = ros::Time::now();
		target_fusedTwistMsg_1.twist.linear.x = theif_objects[0]->getFusedState()(3);
		target_fusedTwistMsg_1.twist.linear.y = theif_objects[0]->getFusedState()(4);
		target_fusedTwistMsg_1.twist.linear.z = theif_objects[0]->getFusedState()(5);

		target_fusedTwistMsg_2.header.stamp = ros::Time::now();
		target_fusedTwistMsg_2.twist.linear.x = theif_objects[1]->getFusedState()(3);
		target_fusedTwistMsg_2.twist.linear.y = theif_objects[1]->getFusedState()(4);
		target_fusedTwistMsg_2.twist.linear.z = theif_objects[1]->getFusedState()(5);
		// -------------------------------------Camera detect?-------------------------------------
		isTargetEst_msg.data = gt_m.ifCameraMeasure();
		// -------------------------------------debug-----------------------------------------
		// -------------------------------------Publish-------------------------------------
		mavros_fusionPose_pub.publish(self_fusedPoseMsg);
		mavros_fusionTwist_pub.publish(self_fusedTwistMsg);
		// // single target
		// target_fusionPose_pub.publish(target_fusedPoseMsg);
		// target_fusionTwist_pub.publish(target_fusedTwistMsg);
		// multiple targets
		// for (int i = 0; i < targetNum; i++)
		// {
		// 	target_fusionPose_pubs[i].publish(target_fusedPoseMsg_1);
		// 	target_fusionTwist_pubs[i].publish(target_fusedTwistMsgs_1);
		// }

		target1_pose_pub.publish(target_fusedPoseMsg_1);
		target1_twist_pub.publish(target_fusedTwistMsg_1);
		target2_pose_pub.publish(target_fusedPoseMsg_2);
		target2_twist_pub.publish(target_fusedTwistMsg_2);

		isTargetEst_pub.publish(isTargetEst_msg);

		/*=================================================================================================================================
			Descrete time
		=================================================================================================================================*/
		dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();
		
		rate.sleep();
    	ros::spinOnce();
    }

	return 0;
}

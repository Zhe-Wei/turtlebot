#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
//#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace std;
using namespace Eigen;

const float DEG_2_RAD = M_PI / 180.0;

geometry_msgs::PoseWithCovarianceStamped Calibration_d1;
geometry_msgs::PoseWithCovarianceStamped Calibration_f1;
geometry_msgs::PoseWithCovarianceStamped Calibration_d2;
geometry_msgs::PoseWithCovarianceStamped Calibration_f2;
geometry_msgs::PoseWithCovarianceStamped Calibration_d3;
geometry_msgs::PoseWithCovarianceStamped Calibration_f3;

vector<std_msgs::Header> header_d;
vector<std_msgs::Header> header_f;

float odomX, odomY, odomZ;
vector<vector<float>> forward_tags(3, vector<float>(7));  // (x, y, z, w, i, j, k)
vector<vector<float>> downward_tags(3, vector<float>(7));

// centre ids of tag_bundles and each position
map<int, vector<float>> downward_id = {
	{0, {0, 0, 0}},
	{10, {0, 2, 0}},
	{20, {0, -2, 0}}
};

map<int, vector<float>> forward_id = {
	{5, {4.01, 0, 2}},
	{15, {4.01, 2, 2}},
	{25, {4.01, -2, 2}}
};

//Matrix<float, 3, 3> forwardTag2world;

void OdeCallback(const geometry_msgs::Pose::ConstPtr &msg)
{

	odomX = msg->position.x;
	odomY = msg->position.y;
    odomZ = msg->position.z;

}

void IrisPositionDownword(vector<vector<float>> &q_d, vector<int> &detected_id)
{
	vector<float> temp_pos;
	vector<vector<float>> downward_pos;
	vector<vector<float>> downward_pos_error;

	for(unsigned int num = 0; num < q_d.size(); num++) {
		q_d[num][2] += (0.01 + 0.05);  // calibration to tag frame
		temp_pos = { q_d[num][0], q_d[num][1], q_d[num][2] };
		downward_pos.push_back(temp_pos);
	}

	for(unsigned int num = 0; num < q_d.size(); num++) {
		switch(num) {
			case 0:    // first downward pose sensor
				// Message of downward camera
				Calibration_d1.header = header_d[num];
				Calibration_d1.header.frame_id = "iris1/odometry_sensor1";

				Calibration_d1.pose.pose.position.x = downward_pos[num][0] + downward_id[detected_id[num]].at(0);
				Calibration_d1.pose.pose.position.y = downward_pos[num][1] + downward_id[detected_id[num]].at(1);
				Calibration_d1.pose.pose.position.z = downward_pos[num][2] + downward_id[detected_id[num]].at(2);

				downward_pos_error.push_back( { abs(odomX-downward_pos[num][0]), abs(odomY-downward_pos[num][1]), abs(odomZ-downward_pos[num][2]) } );

				break;
			case 1:   // second downward pose sensor
				// Message of downward camera
				Calibration_d2.header = header_d[num];
				Calibration_d2.header.frame_id = "iris1/odometry_sensor1";

				Calibration_d2.pose.pose.position.x = downward_pos[num][0] + downward_id[detected_id[num]].at(0);
				Calibration_d2.pose.pose.position.y = downward_pos[num][1] + downward_id[detected_id[num]].at(1);
				Calibration_d2.pose.pose.position.z = downward_pos[num][2] + downward_id[detected_id[num]].at(2);

				downward_pos_error.push_back( { abs(odomX-downward_pos[num][0]), abs(odomY-downward_pos[num][1]), abs(odomZ-downward_pos[num][2]) } );

				break;
			case 2:   // third downward pose sensor
				// Message of downward camera
				Calibration_d3.header = header_d[num];
				Calibration_d3.header.frame_id = "iris1/odometry_sensor1";

				Calibration_d3.pose.pose.position.x = downward_pos[num][0] + downward_id[detected_id[num]].at(0);
				Calibration_d3.pose.pose.position.y = downward_pos[num][1] + downward_id[detected_id[num]].at(1);
				Calibration_d3.pose.pose.position.z = downward_pos[num][2] + downward_id[detected_id[num]].at(2);

				downward_pos_error.push_back( { abs(odomX-downward_pos[num][0]), abs(odomY-downward_pos[num][1]), abs(odomZ-downward_pos[num][2]) } );

				break;
			default:
				break;
		}
	}
	
}

void IrisPositionForword(vector<vector<float>> &q_f, vector<int> &detected_id) 	// q_f = queue forward
{
	vector<float> temp_pos; 
	//Matrix<float, 3, 1> rota_pos; // = Matrix3f::Zero();
	//Matrix<float, 1, 3> temp_pos;
	vector<vector<float>> forward_pos_error; //放誤差
	vector<vector<float>> forward_pos; 

	//forwardTag2world << 0, 0, 1, 0, 1, 0, -1, 0, 0;

	for(unsigned int num = 0; num < q_f.size(); num++) {
		q_f[num][2] += (0.01 + 0.1);  // calibration to tag frame           將z-axis位移0.11
		temp_pos = {q_f[num][0], q_f[num][1], q_f[num][2]};
		//rota_pos = forwardTag2world * temp_pos.transpose();

		forward_pos.push_back(temp_pos);
		//forward_pos.push_back( {rota_pos.coeff(0, 0), rota_pos.coeff(1, 0), rota_pos.coeff(2, 0)} );
	}

	// q_f         = 原本的 detection position x, y, z
	// forward_pos = 修改後 detection position x, y, z+0.11
	cout << "Finished to transfer.    q_f size: " << q_f.size() << endl; 

	for(unsigned int num = 0; num < q_f.size(); num++) {
		cout << "num: " << num << endl;
		switch(num) {
			case 0:    // first forward pose sensor
				// Message of forward camera
				Calibration_f1.header = header_f[num];
				Calibration_f1.header.frame_id = "iris1/odometry_sensor1";

				Calibration_f1.pose.pose.position.x = - forward_pos[num][2] + forward_id[detected_id[num]].at(0);
				Calibration_f1.pose.pose.position.y = forward_pos[num][1] + forward_id[detected_id[num]].at(1);
				Calibration_f1.pose.pose.position.z = forward_pos[num][0] + forward_id[detected_id[num]].at(2);

				forward_pos_error.push_back( { abs(odomX-forward_pos[num][0]), 
												abs(odomY-forward_pos[num][1]), 
												abs(odomZ-forward_pos[num][2]) } );

				break;
			case 1:   // second forward pose sensor
				// Message of forward camera
				Calibration_f2.header = header_f[num];
				Calibration_f2.header.frame_id = "iris1/odometry_sensor1";

				Calibration_f2.pose.pose.position.x = - forward_pos[num][2] + forward_id[detected_id[num]].at(0);
				Calibration_f2.pose.pose.position.y = forward_pos[num][1] + forward_id[detected_id[num]].at(1);
				Calibration_f2.pose.pose.position.z = forward_pos[num][0] + forward_id[detected_id[num]].at(2);

				forward_pos_error.push_back( { abs(odomX-forward_pos[num][0]), abs(odomY-forward_pos[num][1]), abs(odomZ-forward_pos[num][2]) } );

				break;
			case 2:   // third forward pose sensor
				// Message of forward camera
				Calibration_f3.header = header_f[num];
				Calibration_f3.header.frame_id = "iris1/odometry_sensor1";

				Calibration_f3.pose.pose.position.x = - forward_pos[num][2] + forward_id[detected_id[num]].at(0);
				Calibration_f3.pose.pose.position.y = forward_pos[num][1] + forward_id[detected_id[num]].at(1);
				Calibration_f3.pose.pose.position.z = forward_pos[num][0] + forward_id[detected_id[num]].at(2);

				forward_pos_error.push_back( { abs(odomX-forward_pos[num][0]), abs(odomY-forward_pos[num][1]), abs(odomZ-forward_pos[num][2]) } );

				break;
			default:
				break;
		}
	}
}
/*
Eigen::Quaterniond Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z)
{
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;
	Eigen::Quaterniond q3;

	q1.x() = i; q1.y() = j; q1.z() = k; q1.w() = w;
	q2.x() = x; q2.y() = y; q2.z() = z; q2.w() = 0;

	q3 = q1.inverse() * q2 * q1;

	return q3;
}
*/
void downCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{

	vector<float> temp_q;
	vector<int> detected_id;
	vector<vector<float>> q_d;
	
	if(msg->detections.size() > 0) {

		for(unsigned int count = 0; count < msg->detections.size(); count++) {
			detected_id.push_back(msg->detections[count].id[0]);

			header_d.push_back( msg->detections[count].pose.header );

			downward_tags[count][1] = msg->detections[count].pose.pose.pose.position.x;
			downward_tags[count][0] = msg->detections[count].pose.pose.pose.position.y;
			downward_tags[count][2] = msg->detections[count].pose.pose.pose.position.z;
			downward_tags[count][3] = msg->detections[count].pose.pose.pose.orientation.w;
			downward_tags[count][4] = msg->detections[count].pose.pose.pose.orientation.x;
			downward_tags[count][5] = msg->detections[count].pose.pose.pose.orientation.y;
			downward_tags[count][6] = msg->detections[count].pose.pose.pose.orientation.z;

			temp_q = {downward_tags[count][0], downward_tags[count][1], downward_tags[count][2]};
			q_d.push_back(temp_q);
		}
		
		IrisPositionDownword(q_d, detected_id);
		header_d.clear();
	}
}

void forCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	vector<float> temp_q;
	vector<int> detected_id; //所有tag ids
	vector<vector<float>> q_f; //所有(position)資料 只放position資料沒有放orientation資料

	cout << "Forward camera: " << msg->detections.size() << " poses detected" << endl;
	
	if(msg->detections.size() > 0) {

		for(unsigned int count = 0; count < msg->detections.size(); count++) {
			detected_id.push_back(msg->detections[count].id[0]);

			header_f.push_back( msg->detections[count].pose.header );

			forward_tags[count][1] = msg->detections[count].pose.pose.pose.position.x;
			forward_tags[count][0] = msg->detections[count].pose.pose.pose.position.y;
			forward_tags[count][2] = msg->detections[count].pose.pose.pose.position.z;
			forward_tags[count][3] = msg->detections[count].pose.pose.pose.orientation.w;
			forward_tags[count][4] = msg->detections[count].pose.pose.pose.orientation.x;
			forward_tags[count][5] = msg->detections[count].pose.pose.pose.orientation.y;
			forward_tags[count][6] = msg->detections[count].pose.pose.pose.orientation.z;

			temp_q = {forward_tags[count][0], forward_tags[count][1], forward_tags[count][2]}; // tmp = y x z 只放position資料沒有放orientation資料
			q_f.push_back(temp_q);
		}
		
		IrisPositionForword(q_f, detected_id);
		header_f.clear();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "apriltag_coordinate_transformation");
	ros::NodeHandle nh;

	ROS_INFO("Start tranfering");

	// Node setting
	ros::Publisher Pose_detect_forward1;
	ros::Publisher Pose_detect_downward1;
	ros::Publisher Pose_detect_forward2;
	ros::Publisher Pose_detect_downward2;
	ros::Publisher Pose_detect_forward3;
	ros::Publisher Pose_detect_downward3;
	ros::Subscriber ode_sub;
	ros::Subscriber downward_tag;
	ros::Subscriber forward_tag;

	ros::Rate r(10000); // 100us

	// Variable setting
	Pose_detect_downward1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d1", 100);
	Pose_detect_forward1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f1", 100);
	Pose_detect_downward2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d2", 100);
	Pose_detect_forward2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f2", 100);
	Pose_detect_downward3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d3", 100);
	Pose_detect_forward3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f3", 100);

	forward_tag = nh.subscribe("/forward/tag_detections", 100, forCallback);
	downward_tag = nh.subscribe("/downward/tag_detections", 100, downCallback);
	ode_sub = nh.subscribe("/iris1/odometry_sensor1/pose", 100, OdeCallback);

	while (ros::ok())
	{
		
		Pose_detect_downward1.publish(Calibration_d1);
		Pose_detect_forward1.publish(Calibration_f1);
		Pose_detect_downward2.publish(Calibration_d2);
		Pose_detect_forward2.publish(Calibration_f2);
		Pose_detect_downward3.publish(Calibration_d3);
		Pose_detect_forward3.publish(Calibration_f3);
	
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}

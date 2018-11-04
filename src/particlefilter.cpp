#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iterator>
#include <random>
#include <stdlib.h>
#include <cstdlib>
#include <math.h> 
#include <vector> 


//For Gaussian noise: https://stackoverflow.com/questions/32889309/adding-gaussian-noise
//#include <iostream>



#define M 1000 //Choose number of particles (will be used in fraction)
// topic /grid_map   one long vector type int8[]
// ros MapMetaData
// use lidar measages straight away



// ->  means getting the attribute specified after the arrow

class Particle{

	public:
		double x;
		double y;
		double theta;
		double m;

	Particle(){
		double r1 = ((double) rand() / (RAND_MAX));
		double r2 = ((double) rand() / (RAND_MAX));
		double r3 = ((double) rand() / (RAND_MAX));
		
		x = r1*0.5;  //r1*mapWidth*mapResolution/10.0; //10 since we know initial pose close to origin!
		y = r2*0.5;  //r2*mapLength*mapResolution/10.0;
		theta = r3*2*M_PI; //Or does theta go from -pi to pi?
		m = 1.0/M;
	}

	~Particle(){
	}
}; //end of class particle


class PfPoseNode{

	private:
		double mapWidth;
		double mapLength;
		double mapResolution;
		double range_min;
		double range_max;
		double angle_min;
		double angle_increment;
		std::vector<float> measurements;
		float linear_x;
		float angular_z;

		nav_msgs::OccupancyGrid current_map;

	public:
		ros::NodeHandle n;

		ros::Subscriber measurement_sub;
		ros::Subscriber gridmap_sub;
		ros::Subscriber velocity_sub;
	
		tf::TransformBroadcaster pf_pose_broadcaster;

	PfPoseNode(){
		n = ros::NodeHandle("~");

		measurement_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1000,&PfPoseNode::readScan,this);
		gridmap_sub = n.subscribe<nav_msgs::OccupancyGrid>("/grid_map",1,&PfPoseNode::readMap,this);
		velocity_sub = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",1,&PfPoseNode::velocityCallback,this);
	}


///////////////////// POSITIONING LOOP (PARTICLE FILTER) ////////////////////////////////////////
	void positioningLoop(std::vector<Particle> particles){ //Resamples and broadcasts new center of mass as best guess of pose

		ros::Time msg_time = ros::Time::now();
		          

						//Assign weights (importance factor) to particles	
		double CDF[M]; //Comulative distribution function for the weights
		double cumsum = 0;

		for(int i=0; i< M; i++){
			particles[i].m = get_weight(particles[i]);  //updates weight
			cumsum += particles[i].m;   //use this weight for the CDF
			CDF[i] = cumsum;  //Not normalized, therefor use final cumsum to compute r below
		}

			//Resample M new particles randomly according to weight
		Particle new_particles[M];
		int ind = 0;

			//And move particles based on control input (speeds) with added noise
		double dt = 1; //How to get time between particle updates???
		float v = linear_x;
		float omega = angular_z;

		double dx = v/omega * sin(omega*dt);
		double dy = v/omega *( 1 - cos(omega*dt) );

			//Define Gaussian noise   HELP: fix this!!!
//		const double mean = 0.0;
//		const double stddev = 0.01;
//		std::default_random_engine generator;
//		std::normal_distribution<double> dist(mean, stddev);
//    		std::random_device rd{};
//    		std::mt19937 gen{rd()};

//		std::normal_distribution<double> d{mean, stddev};

		for(int i=0; i< M; i++){
			double r = ((double) rand() * cumsum / (RAND_MAX));
			while(CDF[ind] < r){
				ind ++;
			}
			new_particles[i].x = particles[ind].x + dx;//+ d(gen); //Gaussian noise
			new_particles[i].y = particles[ind].y + dy; //+ d(gen);
			new_particles[i].theta = particles[ind].theta; //+ d(gen);
		}


			//Check degeneracy: 

		//if almost all weight extremely small:
			//resample

//		particles = new_particles; Doesnt work, do the following instead://HELP: Pointers more efficient?

		for(int i=0; i< M; i++){
			particles[i] = new_particles[i];
		}

	//The center of mass of the particles will give the best estimate of the position:

			//WHAT TO DO IF PARTICLES SPRED OUT IN DIFFERENT BUNCHES?
		double m_sum = 0;
		double xm_sum = 0;
		double ym_sum = 0;
		double thetam_sum = 0;

		for(int i=0; i < M; i++){
			double x = particles[i].x;
			double y = particles[i].y;
			double theta = particles[i].theta;
			double m = particles[i].m;	

			m_sum += m;

			xm_sum += x*m;
			ym_sum += y*m;
			thetam_sum += theta*m;
			
		}

		double x_cof = xm_sum/m_sum;
	 	double y_cof = ym_sum/m_sum;
	 	double theta_cof = thetam_sum/m_sum;
		//Publish this as the estimated pose, new topic pf_pose
	
		

		 
	      //Publish the pf_pose over TF    AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!

 		geometry_msgs::Quaternion theta_quat = tf::createQuaternionMsgFromYaw(theta_cof);
		geometry_msgs::TransformStamped pf_trans;
		pf_trans.header.stamp = msg_time;
		pf_trans.header.frame_id = "pf_pose";
		pf_trans.child_frame_id = "base_link";
		pf_trans.transform.translation.x = x_cof;
		pf_trans.transform.translation.y = y_cof;
		pf_trans.transform.translation.z = 0.0;
		pf_trans.transform.rotation = theta_quat;

		pf_pose_broadcaster.sendTransform(pf_trans);
		//ROS_INFO("Position (x,y,phi) = (%f,%f,%f)", x,y,phi);
		
		}
			


/////////// CALLBACK FUNCTIONS ///////////////////////

	void readMap(const nav_msgs::OccupancyGrid map_msg)
	{
		current_map = map_msg;
		mapWidth = map_msg.info.width; // number of rows
		mapLength = map_msg.info.height; // number of columns
		mapResolution = map_msg.info.resolution;
	}

	void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg){
		  //ROS_INFO("velocityCallback velocity linear: %f, velocity angular: %f", msg->linear.x,msg->angular.z);

		  linear_x = (float) msg->linear.x;
		  angular_z = (float) msg->angular.z;
	//I use both to move particles
	}

	void readScan(const sensor_msgs::LaserScan scan_msg){
		measurements = scan_msg.ranges;
		range_min = scan_msg.range_min;
		range_max = scan_msg.range_max;
		angle_min = scan_msg.angle_min;
		angle_increment = scan_msg.angle_increment;
	}

////////////////////

	int get_weight(Particle particle){  // p(z|x)

		//Convert all scan measurements coordinates to occupancy grid
		tf::TransformListener tf_listener;
		tf::StampedTransform laser_tf;

		geometry_msgs::PointStamped laser_point; // represents point in laser frame
		geometry_msgs::PointStamped map_point; // represents point in map frame

		//get transform from laser to map frame
		int valueSum = 0;

		try
		{
			//tf_listener.waitForTransform("map","laser",TODO TIME ,ros::Duration(2));
			//tf_listener.lookupTransform("map",TODO TIME, laser_tf);
			
			for (int i = 0; i < measurements.size(); i++)
			{
				laser_point.point.x = particle.x + measurements[i]*cos(particle.theta + angle_min+i*angle_increment); //IS THIS RIGHT???
				laser_point.point.y = particle.y + measurements[i]*sin(particle.theta + angle_min+i*angle_increment);
				laser_point.point.z = 0.0;

				// transform to map frame
				tf_listener.transformPoint("/map",laser_point,map_point);

				// map matrix indices
				int map_index_x = (int) round(map_point.point.x/mapResolution);
				int map_index_y = (int) round(map_point.point.y/mapResolution);

				int map_index = map_index_x*mapWidth+map_index_y; // map array cell index

				int probValue = current_map.data[map_index]; // grid map cell value
			
				// Do something with value
				if(probValue > 20){
					valueSum += probValue;
				}

			}
		}catch(tf::TransformException ex){
			ROS_ERROR("Transform exception in particle node.");
		}
		
		return valueSum;
	}

/*  Should we consider at all how near the particle is to the best guess of position?
	void get_weight(const x_robot, y_robot, x, y){  // p(z|x)
		double r = 50;
		    
	// [x,y] = fit_inside(size(frame,2),size(frame,1),x,y,1);
		      
		distance = sqrt( pow(x_robot -x, 2) + pow(y_robot -y, 2));
		if(distance > r){
		  weight = 0;
		}
		else if(distance == 0){
		  weight = 2; //Better than second best = 1
		}
		else{
		  weight = 1/distance;
		}

	} */


}; //end of Node class


int main(int argc, char **argv)
{
  	ros::spin();
	
	//Initialize the particles. The constructor does this UNIFORMLY!
	std::vector <Particle> particles(M);
	ros::init(argc, argv,"pf_pose_est");
	PfPoseNode pf_pose_est;

	ros::Rate loop_rate(1);

	while(ros::ok())
	{
		pf_pose_est.positioningLoop(particles);
		ros::spinOnce();
		loop_rate.sleep();
	}

  	return 0;
}


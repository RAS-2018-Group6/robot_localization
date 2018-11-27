#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>
//#include <sensor_msgs/PointCloud.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iterator>
//#include <random>
#include <stdlib.h>
#include <cstdlib>
#include <cmath>



//For Gaussian noise: https://stackoverflow.com/questions/32889309/adding-gaussian-noise
//#include <iostream>



#define M 200 //Choose number of particles (will be used in fraction)
#define loopRate 10 //Choose how often to resample particles
#define x_start 2.2
#define y_start 0.2
#define theta_start M_PI/2

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

	//SOMETHING IS WIERD HERE! STARTING WITH x_start INLUENCES THE FILTER EVEN AFTER INITIALIZATION!!!
        x = r1*0.2-0.1 + 2.2;// x_start; //r1*2.45; //r1*0.2-0.1 + x_start; //r1*mapWidth*mapResolution;
        y = r2*0.2-0.1 + 0.2;//y_start;// r2*2.45;// r2*0.2-0.1 + y_start;//r2*0.3+0.1;  //r2*mapHeight*mapResolution;
        theta = r3*0.2 -0.1 + M_PI/2;//theta_start; //r3*theta_start;//M_PI/2 -0.2 + r3*0.4; //r3*2*M_PI; //Or does theta go from -pi to pi?
        m = 1.0;
    }

    ~Particle(){
    }
}; //end of class particle


class PfPoseNode{

private:
    double mapWidth;
    double mapHeight;
    double mapResolution;
    double range_min;
    double range_max;
    double angle_min;
    double angle_increment;
		int max_it; //numbers of iterations of the global localization before you restart

    bool map_received;
    bool laser_scan_received;

    std::vector<float> measurements;
    float linear_x;
    float angular_z;
    nav_msgs::Odometry position_msg;
    nav_msgs::OccupancyGrid current_map;
    //sensor_msgs::PointCloud pointcloud;
    tf::TransformListener tf_listener;
    ros::Time current_stamp;
    std::vector <Particle> particles;
		geometry_msgs::PoseArray particle_msg;


public:
    ros::NodeHandle n;

    ros::Subscriber measurement_sub;
    ros::Publisher measurement_pub;
    ros::Subscriber gridmap_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher position_pub;
    ros::Publisher particles_pub;

    tf::TransformBroadcaster pose_broadcaster;

    PfPoseNode(){
        n = ros::NodeHandle("~");
        initializeParticles();

        map_received = false;
        laser_scan_received = false;

				particle_msg.poses.resize(M);
				particle_msg.header.frame_id = "/map";

        position_pub = n.advertise<nav_msgs::Odometry>("/particle_position",1);
        particles_pub = n.advertise<geometry_msgs::PoseArray>("/particles_poses",1);
        measurement_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1,&PfPoseNode::readScan,this);
        measurement_pub= n.advertise<sensor_msgs::LaserScan>("/scan_modified",1); // test laserscan
        gridmap_sub = n.subscribe<nav_msgs::OccupancyGrid>("/smooth_map",1,&PfPoseNode::readMap,this);
        velocity_sub = n.subscribe<nav_msgs::Odometry>("/odom",1,&PfPoseNode::velocityCallback,this); //Previously /motor_controller/twist

        geometry_msgs::Quaternion theta_quat = tf::createQuaternionMsgFromYaw(theta_start);
        geometry_msgs::TransformStamped pose_trans;
        pose_trans.header.stamp = ros::Time::now(); // ????
        pose_trans.header.frame_id = "odom";
        pose_trans.child_frame_id = "base_link";
        pose_trans.transform.translation.x = x_start;
        pose_trans.transform.translation.y = y_start;
        pose_trans.transform.translation.z = 0.0;
        pose_trans.transform.rotation = theta_quat;
        pose_broadcaster.sendTransform(pose_trans);
    }

    void initializeParticles(){
      for (int i = 0; i< M;i++){
        Particle part;
        //ROS_INFO("x: %f, y:%f, theta:%f,m:%f",part.x,part.y,part.theta, part.m);
        particles.push_back(part);
      }
			max_it=30;
    }

    void reinitializeParticles(){
      for (int i = 0; i< M;i++){
        Particle part;
        //ROS_INFO("x: %f, y:%f, theta:%f,m:%f",part.x,part.y,part.theta, part.m);
        particles[i] = part;
      }
			max_it=30;
    }


    ///////////////////// POSITIONING LOOP (PARTICLE FILTER) ////////////////////////////////////////
    void positioningLoop(){   //Resamples and broadcasts new center of mass as best guess of pose
      current_stamp = ros::Time::now();

      if(map_received){
        if(laser_scan_received){
          //ROS_INFO("Running positioning loop.");
          //ros::Time msg_time = ros::Time::now();

					max_it -= 1; //counter for running global localization 10 times before restaring
					ROS_INFO("max_it %i",max_it);

          //Assign weights (importance factor) to particles
          double CDF[M]; //Comulative distribution function for the weights
          double cumsum = 0;
					double maxweight = 0;
					double most_prob_theta = 0;
          //ROS_INFO("Get Weights of particles");
          int i = 0;
          for(std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); ++it)
          {
							particle_msg.poses[i].position.x = it->x;
							particle_msg.poses[i].position.y = it->y;
							particle_msg.poses[i].orientation = tf::createQuaternionMsgFromYaw(it->theta);
              it->m = get_weight(*it);  //updates weight
							if( (it->m) > maxweight){
								maxweight = it->m;
								most_prob_theta = it->theta;
							}

              cumsum += (double) it->m;   //use the weight for the CDF, double as to compare with r below
              CDF[i] = cumsum;  //Not normalized, therefor use final cumsum to compute r below
              i += 1;
          }

					particles_pub.publish(particle_msg);

          for(std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); ++it){
            //ROS_INFO("x: %f, y: %f, theta: %f, weight: %f",it->x,it->y,it->theta,it->m);
          }
          //Resample M new particles randomly according to weight
  		    std::vector <Particle> new_particles(M);
          int ind = 0;

          //And move particles based on control input (speeds) with added noise
          double dt = 1.0/loopRate;
          float v = linear_x;
          float omega = angular_z;
          double dx, dy, dtheta;
          if(omega == 0.0){
            dx = v*dt; //_robotframe
            dy = 0.0;
						dtheta = 0.0;
		//change to mapframe
	//	dx = dx_robotframe*cos(theta_cof) - dy_robotframe*cos(M_PI/2 - theta_cof)
          }
					else if(v == 0.0){
						dx = 0.0;
						dy = 0.0;
						dtheta = omega*dt;
					}
          else{
            dx = v/omega * sin(omega*dt);
            dy = v/omega *( 1 - cos(omega*dt) );
						dtheta = omega*dt;
		//Change to mapframe, now taken care i
	//	dx = dx_robotframe*cos(theta_cof) - dy_robotframe*cos(M_PI/2 - theta_cof);
	//	dy = dx_robotframe*sin(theta_cof) + dy_robotframe*sin(M_PI/2 - theta_cof);
          }
//          ROS_INFO("Resample");
          //Define Gaussian noise   HELP: fix this!!!
          const double mean = 0.0;
          double stddev;
          if(v == 0.0 && omega == 0.0){
            stddev = 0.00;
          }
          else{
            stddev = 0.03;    ///0 when debugging!!!!!!!!!
          }
          std::default_random_engine generator;
          std::normal_distribution<double> dist(mean, stddev);
          std::random_device rd{};
          std::mt19937 gen{rd()};

          std::normal_distribution<double> d{mean, stddev};
          int idx = 0;
          for(std::vector<Particle>::iterator it = new_particles.begin(); it != new_particles.end(); ++it)
          {
              idx = 0;
              double r = ((double) rand() * cumsum/ (RAND_MAX));
              while(CDF[idx] < r)
              {
                  idx ++;
              }
              //ROS_INFO("Index: %i, Random: %f", idx,r);
							double w = particles[idx].theta;
              double x_toCheck = particles[idx].x + dx*cos(w) + dy*sin(M_PI/2-w) + d(gen); //Gaussian noise
              double y_toCheck = particles[idx].y + dx*sin(w) - dy*cos(M_PI/2-w) + d(gen);
              double theta_toCheck = particles[idx].theta + dtheta + 3*d(gen); //maybe have more noise in radians?

							//First check that the new particles will be inside of the map, otherwise just keep the old particle's pose
              if(x_toCheck < 0.0 || y_toCheck < 0.0 || x_toCheck > mapHeight*mapResolution || y_toCheck > mapWidth*mapResolution){
                it->x = particles[idx].x;
                it->y = particles[idx].y;
              }
              else{
                it->x = x_toCheck;
                it->y = y_toCheck;
              }
							if(theta_toCheck > 2*M_PI){
								it->theta = theta_toCheck - 2*M_PI;
							}
							else if(theta_toCheck < 0){
								it->theta = 2*M_PI-theta_toCheck;
							}
              else{
								it->theta = theta_toCheck;
							}
          }

          /* for(std::vector<Particle>::iterator it = new_particles.begin(); it != new_particles.end(); ++it){
            ROS_INFO("x: %f, y: %f, theta: %f, weight: %f",it->x,it->y,it->theta,it->m);
          } */


          //Check degeneracy:

          //if almost all weight extremely small:
          //resample

          particles = new_particles; //Seems to work for vectors?//HELP: Pointers more efficient?

  //        for(int i=0; i< M; i++){    //If above doesnt work
   //           particles[i] = new_particles[i];
    //      }

          //The center of mass of the particles will give the best estimate of the position:

          //WHAT TO DO IF PARTICLES SPRED OUT IN DIFFERENT BUNCHES?
          double m_sum = 0;
          double xm_sum = 0;
          double ym_sum = 0;
          //double thetam_sum = 0;
					double thetasin_sum = 0;
					double thetacos_sum = 0;
          //ROS_INFO("Get Center of Mass");
          for(std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); ++it){
              double x = it->x;
              double y = it->y;
              double theta = it->theta;
              double m = it->m;
              //ROS_INFO("x: %f, y: %f, theta:%f",x,y,theta);
              m_sum += m;

              xm_sum += x*m;
              ym_sum += y*m;
              //ROS_INFO("THETA %f", theta);
/*              if(theta > M_PI){      //Convert to interval -pi to pi for summing
                //ROS_INFO("Turned %f into %f", theta, theta - 2*M_PI);
                theta -= 2*M_PI;
              }     */
              ROS_INFO("theta = %f sintheta = %f", theta, sin(theta));
              thetasin_sum += sin(theta)*m; //*m;
							thetacos_sum += cos(theta)*m;
          }
          //ROS_INFO("xm: %f, ym: %f, theta: %f", xm_sum,ym_sum,thetam_sum);
          double x_com, y_com, theta_com, thetasin_com, thetacos_com;
          if(m_sum != 0){
            x_com = xm_sum/m_sum;
            y_com = ym_sum/m_sum;
            thetasin_com = thetasin_sum/m_sum;
            thetacos_com = thetacos_sum/m_sum;
            if(thetacos_sum == 0.0){
              if(thetasin_sum > 0.0){
                theta_com = M_PI/2;
              }
              else{
                theta_com = -M_PI/2;
              }
            }
            theta_com =  atan2(thetasin_com,thetacos_com);//most_prob_theta;   //thetam_sum/M; ///m_sum;
						if(theta_com < 0.0){
							ROS_INFO("Turned %f into %f", theta_com, theta_com+2*M_PI);
							theta_com += 2*M_PI;
						}
          /*  if(theta_com < 0){     //Convert back to intervall 0 to 2pi
              theta_com += 2*M_PI;
            } */

          }else{
            x_com = 10000000; //xm_sum/m_sum;
            y_com = 10000000; //ym_sum/m_sum;
            theta_com = 10000000; //thetam_sum/m_sum;
            ROS_INFO("M SUM IS ZERO!!!! THIS IS BAD.");
          }
//FOR GLOBAL LOCALIZATION
	double x_error = 0.0;
	double y_error = 0.0;
	double theta_error = 0.0;
  double x,y,theta; //m?
	for(std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); ++it){
	  x = it->x;
    y = it->y;
    theta = it->theta;
    // m = it->m;
		x_error += pow(x-x_com, 2);
		y_error += pow(y-y_com, 2);
		theta_error += pow(theta-theta_com, 2);
	}


	double x_var = x_error/M;
	double y_var = y_error/M;
	double theta_var = theta_error/M;
	if((x_var > 0.05 || y_var > 0.05 || theta_var > 0.1) && max_it < 0){
		ROS_INFO("Uncertain pose! x_var: %f , y_var: %f and theta_var: %f",x_var,y_var,theta_var);
		//reinitializeParticles();      //Turn
  	}
		//We have now localized the lidar, but the robots center is 8 cm ahead!!! (base link)
	double x_robot = x_com + 0.08*cos(theta_com);
	double y_robot = y_com + 0.08*sin(theta_com);
	double theta_robot = theta_com;

          //Broadcast this as the estimated pose




          //Publish the pf_pose over TF    AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!

          //ROS_INFO("Publish values x: %f , y: %f and theta: %f",x_robot,y_robot,theta_robot);
          geometry_msgs::Quaternion theta_quat = tf::createQuaternionMsgFromYaw(theta_robot);


          geometry_msgs::TransformStamped pose_trans;
          pose_trans.header.stamp = current_stamp; // ????
          pose_trans.header.frame_id = "odom";
          pose_trans.child_frame_id = "base_link";
          pose_trans.transform.translation.x = x_robot;
          pose_trans.transform.translation.y = y_robot;
          pose_trans.transform.translation.z = 0.0;
          pose_trans.transform.rotation = theta_quat;
          pose_broadcaster.sendTransform(pose_trans);


          position_msg.header.frame_id = "/map";
          position_msg.pose.pose.position.x = x_robot;
          position_msg.pose.pose.position.y = y_robot;
          position_msg.pose.pose.position.z = 0;
          position_msg.pose.pose.orientation = theta_quat;
          //position_msg.twist.twist.linear.x = linear_x; //example
          //position_msg.twist.twist.angular.z = angular_z; //example
          position_pub.publish(position_msg);
          //ROS_INFO("Publishing finished");

        }
      }


    }







    /////////// CALLBACK FUNCTIONS ///////////////////////

    void readMap(const nav_msgs::OccupancyGrid map_msg)
    {
        //ROS_INFO("Read Map");
        map_received = true;
        current_map = map_msg;
        mapWidth = map_msg.info.width; // number of columns (y)
        mapHeight = map_msg.info.height; // number of rows (x)
        mapResolution = map_msg.info.resolution;
        //current_stamp = map_msg.info.stamp;
    }

    void velocityCallback(const nav_msgs::Odometry::ConstPtr& msg){
        //ROS_INFO("velocityCallback velocity linear: %f, velocity angular: %f", msg->linear.x, msg->angular.z);
        //ROS_INFO("Velocity callback");
        linear_x = (float) msg-> twist.twist.linear.x;   //linear.x;
        angular_z = (float) msg-> twist.twist.angular.z;   //angular.z;
				ROS_INFO("linear_speed %f and angular_speed %f", linear_x, angular_z);
        //I use both to move particles
    }

    void readScan(const sensor_msgs::LaserScan scan_msg){

				laser_scan_received = true;
        measurements = scan_msg.ranges;
        range_min = scan_msg.range_min;
        range_max = scan_msg.range_max;
        angle_min = scan_msg.angle_min;
        angle_increment = scan_msg.angle_increment; // not needed anymore

        ROS_INFO("Transform Laser scan");

        // Test laser
        sensor_msgs::LaserScan new_scan;
        new_scan.ranges = scan_msg.ranges;
        new_scan.range_min = scan_msg.range_min;
        new_scan.range_max = scan_msg.range_max;
        new_scan.header.stamp = scan_msg.header.stamp;
        new_scan.header.frame_id = scan_msg.header.frame_id;
        new_scan.angle_max = scan_msg.angle_max;
        new_scan.angle_min = scan_msg.angle_min;
        new_scan.angle_increment = scan_msg.angle_increment;

        for (int i = 0; i < new_scan.ranges.size(); i++){
          new_scan.ranges[i] = new_scan.ranges[i]*(1-0.03*measurements[i]);
          measurements[i] = measurements[i]*(1-0.03*measurements[i]);
        }

        measurement_pub.publish(new_scan);

        /*
        laser_scan_received = true;
        laser_geometry::LaserProjection projector;

        try
        {
            tf_listener.waitForTransform("/map","/laser",scan_msg.header.stamp+ros::Duration().fromSec(scan_msg.ranges.size()*scan_msg.time_increment),ros::Duration(2.0));
            //ROS_INFO("Finished Listening");
            projector.transformLaserScanToPointCloud("/map",scan_msg,pointcloud,tf_listener);
        }catch(tf::TransformException ex)
        {
            ROS_ERROR("Transform error in map node: %s", ex.what());
            return;
        }
        */
    }

    ////////////////////

    int get_weight(Particle particle){  // p(z|x)

        float x,y;
        int valueSum, probValue, map_index;

        valueSum = 0;

        for(int i = 0; i < measurements.size(); i++)
        {
            if(measurements[i] > range_min && measurements[i] < range_max && !( std::isnan(measurements[i])) ){
              double mapFrameAngle = particle.theta + i*angle_increment;
              x = particle.x + measurements[i]*cos(mapFrameAngle);
              y = particle.y + measurements[i]*sin(mapFrameAngle);

							//ROS_INFO("mapHeight %f and mapWidth %f ", mapHeight*mapResolution, mapWidth*mapResolution);
              if(x < 0.0 || y < 0.0 || x > mapHeight*mapResolution || y > mapWidth*mapResolution){
                probValue = 0;
              }

              map_index = round(y/mapResolution)*mapWidth+round(x/mapResolution); // map array cell index

  			      if(map_index >= current_map.data.size() )  //SHOULDNT BE NEEDED?
              {
                probValue = 0;
              }
              else{
                probValue = current_map.data[map_index]; // / (1.0 + 10*measurements[i]); //Deviding to prioritize measurements nearby (more accurate)
              }


              if(probValue > 0){
                  valueSum += probValue;
              }
            }
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


};


int main(int argc, char **argv)
{
    //Initialize the particles. The constructor does this UNIFORMLY!

    ros::init(argc, argv,"pf_pose_est");
    PfPoseNode pf_pose_est;
    ros::Rate loop_rate(loopRate);

    while(ros::ok())
    {

        //ROS_INFO("Test");
        ros::spinOnce();
				pf_pose_est.positioningLoop();
        loop_rate.sleep();
        //ROS_INFO("The main loop");
    }

    return 0;
}

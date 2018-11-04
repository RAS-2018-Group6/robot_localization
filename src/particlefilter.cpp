#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iterator>
//#include <random>
#include <stdlib.h>
#include <cstdlib>
#include <math.h> 


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
    nav_msgs::Odometry position_msg;
    nav_msgs::OccupancyGrid current_map;
    sensor_msgs::PointCloud pointcloud;
    tf::TransformListener *tf_listener;

public:
    ros::NodeHandle n;

    ros::Subscriber measurement_sub;
    ros::Subscriber gridmap_sub;
    ros::Subscriber velocity_sub;
    ros::Publisher position_pub;

    tf::TransformBroadcaster pf_pose_broadcaster;

    PfPoseNode(){
        n = ros::NodeHandle("~");

        position_pub = n.advertise<nav_msgs::Odometry>("particle_position",1);
        measurement_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",10,&PfPoseNode::readScan,this);
        gridmap_sub = n.subscribe<nav_msgs::OccupancyGrid>("/grid_map",1,&PfPoseNode::readMap,this);
        velocity_sub = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",1,&PfPoseNode::velocityCallback,this);
    }


    ///////////////////// POSITIONING LOOP (PARTICLE FILTER) ////////////////////////////////////////
    void positioningLoop(){   //Resamples and broadcasts new center of mass as best guess of pose
        ROS_INFO("Running positioning loop.");
        //ros::Time msg_time = ros::Time::now();

        //		if(particles == Null){   HELP: How to have an initial set of particles and then loop by updating each time?

        Particle particles[M];

        //Assign weights (importance factor) to particles
        double CDF[M]; //Comulative distribution function for the weights
        double cumsum = 0;

        for(int i=0; i< M; i++)
        {
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

        for(int i=0; i< M; i++)
        {
            double r = ((double) rand() * cumsum / (RAND_MAX));
            while(CDF[ind] < r)
            {
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
        /*
                geometry_msgs::TransformStamped pf_trans;
                pf_trans.header.stamp = msg_time;
                pf_trans.header.frame_id = "pf_pose";
                pf_trans.child_frame_id = "base_link";
                pf_trans.transform.translation.x = x_cof;
                pf_trans.transform.translation.y = y_cof;
                pf_trans.transform.translation.z = 0.0;

                pf_trans.transform.rotation = theta_quat;

                //pf_pose_broadcaster.sendTransform(pf_trans);
                //ROS_INFO("Position (x,y,phi) = (%f,%f,%f)", x,y,phi);
                */

        position_msg.pose.pose.position.x = x_cof;
        position_msg.pose.pose.position.y = y_cof;
        position_msg.pose.pose.position.z = 0;
        position_msg.pose.pose.orientation = theta_quat;
        position_msg.twist.twist.linear.x = linear_x; //example
        position_msg.twist.twist.angular.z = angular_z; //example
        position_pub.publish(position_msg);

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

        /*
        measurements = scan_msg.ranges;
        range_min = scan_msg.range_min;
        range_max = scan_msg.range_max;
        angle_min = scan_msg.angle_min;
        angle_increment = scan_msg.angle_increment; // not needed anymore
        */

        laser_geometry::LaserProjection projector;

        try
        {
            tf_listener->waitForTransform("/map","/laser",scan_msg.header.stamp+ros::Duration().fromSec(scan_msg.ranges.size()*scan_msg.time_increment),ros::Duration(2.0));
            projector.transformLaserScanToPointCloud("/map",scan_msg,pointcloud,*tf_listener);
        }catch(tf::TransformException ex)
        {
            ROS_ERROR("Transform error in map node: %s", ex.what());
            return;
        }
    }

    ////////////////////

    int get_weight(Particle particle){  // p(z|x)

        float x,y;
        int valueSum, probValue, map_index;

        valueSum = 0;

        for(int i = 0; i < pointcloud.points.size(); i++)
        {
            x = particle.x + pointcloud.points[i].x; //IS THIS RIGHT???
            y = particle.y + pointcloud.points[i].y;

            map_index = round(x/mapResolution)*mapWidth+round(y/mapResolution); // map array cell index

            //probValue = current_map.data[map_index]; // going to get index out of bounds all the time
            probValue = 0;

            if(probValue > 20){
                valueSum += probValue;
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
    //ros::spin();

    //Initialize the particles. The constructor does this UNIFORMLY!
    //	Particle particles[M];
    ros::init(argc, argv,"pf_pose_est");
    PfPoseNode pf_pose_est;

    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        pf_pose_est.positioningLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


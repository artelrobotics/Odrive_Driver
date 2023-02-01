#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "odrive_driver/Channel_values.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
class Differential_Odometry
{
    public:
        Differential_Odometry();
        void spin();
        
    private:
        ros::NodeHandle nh = ros::NodeHandle("~");
        //tf::TransformBroadcaster odom_broadcaster;
        ros::Subscriber encoders_sub;
        ros::Publisher odom_pub;
        std::string encoder_topic;
        std::string base_frame;
        std::string odom_frame;
        bool publish_tf;
        void encoder_callback(const odrive_driver::Channel_values& msg);
        int left = 0;
        int right = 0;
        int enc_left = 0;
        int enc_right = 0;
        int ppr;
        double radius;
        double wheelbase;
        double rate;
        double dx;
        double dr;
        double ticks_meter;
        double x_final = 0, y_final = 0, theta_final = 0;
        bool init = false;
        ros::Time current_time, last_time;
        void init_variables();
        void update();   
};
Differential_Odometry::Differential_Odometry(){
    init_variables();
    ROS_INFO("Started Odometry Computing Cpp Node");
    encoders_sub = nh.subscribe(encoder_topic, 1000, &Differential_Odometry::encoder_callback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
}

void Differential_Odometry::init_variables()
{
    nh.param<std::string>("encoder_topic_name", encoder_topic, "/leo_bot/shadow_counts");
    nh.param<bool>("publish_tf", publish_tf, false);
    nh.param<std::string>("odom_frame", odom_frame, "odom");
    nh.param<std::string>("base_frame", base_frame, "base_link");
    nh.param<double>("wheelbase", wheelbase, 0.365);
    nh.param<double>("wheel_radius", radius, 0.085);
    nh.param<int>("ppr", ppr, 1024);
    nh.param("rate", rate, 100.0);
    
    ticks_meter = (ppr * 4) / (2 * M_PI * radius);
    current_time = ros::Time::now();
  	last_time = ros::Time::now();
    enc_left = left;
	enc_right =right;
    
}


void Differential_Odometry::update()
{
    ros::Time current_time = ros::Time::now();
    double elapsed;
    double d_left, d_right, d=0, th=0, x, y;
    
    if(init)
    {
        elapsed = current_time.toSec() - last_time.toSec();
        
        d_left = (left - enc_left) / ticks_meter;
        d_right = (right - enc_right) / ticks_meter;
        enc_left = left;
        enc_right = right;
        
        d = (d_left + d_right) / 2.0;
        th = (d_right - d_left) / wheelbase;
        
        dx = d / elapsed;
        dr = th / elapsed;

        x = cos(th) * d;
        y = sin(th) * d;

        x_final = x_final + ( cos(theta_final) * x - sin(theta_final) * y);
        y_final = y_final + ( sin(theta_final) * x + cos(theta_final) * y);    
        
        
        
        theta_final = theta_final + th;
        
        geometry_msgs::Quaternion odom_quad;
        odom_quad.x = 0.0;
        odom_quad.x = 0.0;
        odom_quad.z = sin(theta_final / 2);
        odom_quad.w = cos(theta_final / 2);

        if(publish_tf)
        {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = odom_frame;
            odom_trans.child_frame_id = base_frame;

            odom_trans.transform.translation.x = x_final;
            odom_trans.transform.translation.x = x_final;
            odom_trans.transform.translation.y = y_final;
            odom_trans.transform.translation.z = 0.0;  // 2D Z axis is 0
            odom_trans.transform.rotation = odom_quad;

            //send the transform
            //odom_broadcaster.sendTransform(odom_trans);
        }

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_frame;
        //set position
        odom.pose.pose.position.x = x_final;
        odom.pose.pose.position.y = y_final;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quad;

        //set velocity
        odom.twist.twist.linear.x = dx;
        odom.twist.twist.angular.z = dr;

        odom_pub.publish(odom);
    }
    

    last_time = current_time;     
    ros::spinOnce();
    }

//Spin function
void Differential_Odometry::spin(){
     ros::Rate loop_rate(rate);
     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}

void Differential_Odometry::encoder_callback(const odrive_driver::Channel_values& msg)
{
    left = msg.left;
    right = msg.right;
    if(init == false){
        init = true;
        enc_left = left;
        enc_right = right;
    }
    //ROS_INFO("left %d: ", left);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Differential_Odometry_CPP");
    Differential_Odometry obj;
    obj.spin();
    return 0;
}


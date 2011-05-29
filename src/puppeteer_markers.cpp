// estimator_node.cpp
// Jake Ware and Jarvis Schultz
// Spring 2011

//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------
/*

 */


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <puppeteer_msgs/State.h>
#include <visualization_msgs/Marker.h>

#include <math.h>


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class PuppeteerMarkers {

private:
  ros::NodeHandle n_;
  ros::Subscriber state_sub;
  ros::Publisher marker_pub;
  ros::Time t_now, t_last;
  visualization_msgs::Marker mass_marker;
  visualization_msgs::Marker cart_marker;
  float dt;

public:
  PuppeteerMarkers() {
    state_sub = n_.subscribe("/system_state", 1, &PuppeteerMarkers::statecb, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("puppeteer_markers", 1);

    t_now = ros::Time::now();

    // set marker properties for mass_marker
    mass_marker.header.frame_id = "/openni_rgb_optical_frame";
    mass_marker.ns = "estimator_mass";
    mass_marker.id = 0;
    mass_marker.type = visualization_msgs::Marker::SPHERE;
    mass_marker.scale.x = 0.05;
    mass_marker.scale.y = 0.05;
    mass_marker.scale.z = 0.05;
    mass_marker.color.r = 1.0f;
    mass_marker.color.g = 0.0f;
    mass_marker.color.b = 0.0f;
    mass_marker.color.a = 1.0f;
    mass_marker.lifetime = ros::Duration();

    // set market properties for cart_marker
    cart_marker.header.frame_id = "/openni_rgb_optical_frame";
    cart_marker.ns = "estimator_cart";
    cart_marker.id = 0;
    cart_marker.type = visualization_msgs::Marker::CUBE;
    cart_marker.scale.x = 0.15;
    cart_marker.scale.y = 0.15;
    cart_marker.scale.z = 0.15;
    cart_marker.color.r = 0.0f;
    cart_marker.color.g = 1.0f;
    cart_marker.color.b = 0.0f;
    cart_marker.color.a = 1.0f;
    cart_marker.lifetime = ros::Duration();

    ROS_INFO("Starting Puppeteer Markers...\n");
  }

  void statecb(const puppeteer_msgs::State &state) {
	ROS_DEBUG("Entered state callback");
    
	// check if the operating_condition parameter exists and set its value
	int operating_condition = 0;
	if(ros::param::has("operating_condition")) {
	  ros::param::get("/operating_condition", operating_condition);
	}
	else {
	    ROS_WARN("Cannot Find Parameter: operating_condition");
	    ROS_INFO("Setting operating_condition to IDLE");
	    ros::param::set("/operating_condition", 0);
      
	    return;
	}

	// check to see if we are in calibrate or run state
	if(operating_condition == 1 || operating_condition == 2) {
	  // get current time and dt to last call
	  t_last = t_now;
	  t_now = ros::Time::now();
	  dt = (t_now.toSec()-t_last.toSec());
	  ROS_DEBUG("dt: %f", dt);

	  // set mass_marker details
	  mass_marker.pose.position.x = state.xm;
	  mass_marker.pose.position.y = state.ym;
	  mass_marker.pose.position.z = 2;  // hard coding this for now
	  mass_marker.pose.orientation.x = 0.0;
	  mass_marker.pose.orientation.y = 0.0;
	  mass_marker.pose.orientation.z = 0.0;
	  mass_marker.pose.orientation.w = 1.0;
	  mass_marker.header.stamp = t_now;
	 	
	  // set cart_marker details
	  cart_marker.pose.position.x = state.xc;
	  cart_marker.pose.position.y = 0;
	  cart_marker.pose.position.z = 2;
	  cart_marker.pose.orientation.x = 0.0;
	  cart_marker.pose.orientation.y = 0.0;
	  cart_marker.pose.orientation.z = 0.0;
	  cart_marker.pose.orientation.w = 1.0;
	  cart_marker.header.stamp = t_now;

	  // publish markers
	  marker_pub.publish(mass_marker);
	  marker_pub.publish(cart_marker);
    	}
    
	// are we in idle or stop condition?
	else if(operating_condition == 0 || operating_condition == 3) {
	    ROS_DEBUG("Estimator node is idle due to operating condition");
	} 
    
	// are we in emergency stop condition?
	else if(operating_condition == 4) {
	    // did we get an emergency stop request?
	    static bool emergency_flag = false;
	    if(operating_condition == 4 && emergency_flag == false) {
		ROS_WARN("Emergency Stop Requested");
		emergency_flag = true;
	    }
	}
    
	// otherwise something terrible has happened
	else {
	    ROS_ERROR("Invalid value for operating_condition");
	}

	ROS_DEBUG("Leaving tracker callback");
    }
};


//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "puppeteer_markers");
  ros::NodeHandle n;

  PuppeteerMarkers markers;

  ros::spin();

  return 0;
}

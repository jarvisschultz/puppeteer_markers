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

class MarkerNode {

private:
    ros::NodeHandle n_;
    ros::Subscriber tracker_sub;
    ros::Publisher marker_pub;

public:

  PuppeteerMarkers() {
    tracker_sub = n_.subscribe("/object1_position", 1, &PuppeteerMarkers::statecb, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("puppeteer_markers", 1);

    ROS_INFO("Starting Puppeteer Markers...\n");
  }

    void trackercb(const puppeteer_msgs::PointPlus &point) {
	ROS_DEBUG("Entered tracker callback");
    
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

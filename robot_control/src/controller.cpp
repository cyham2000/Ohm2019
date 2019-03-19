#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <isc_joy/xinput.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ohm_igvc_msgs/Target.h>
#include <ohm_igvc_srvs/waypoint.h>
#include <serial/serial.h>

class robotController 
{
	public:

		enum state
		{
			AUTOMATIC, 
			LOW_VOLTAGE, 
			KILL, 
			PAUSE, 
			MANUAL
		};

		void robotController();
		void update_state();
		void manualCallback(const geometry_msgs::Twist::ConstPtr &msg);
		void autoCallback(const geometry_msgs::Twist::ConstPtr &msg);	

	private:
		bool automatic;	//sets up booleans to be input into switching case
		bool low_voltage;
		bool kill;
		bool pause;
		bool manual;

		STATE robostate; 
		STATE prev_state;
}

void robotController::manualCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	if (robostate == MANUAL)
	{
		pub.publish(msg);
	}		
}

void robotController::autoCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	if (robostate == AUTOMATIC)
	{
		pub.publish(msg);
	}
}

void robotController::update_state()
{
	
	switch(robostate)
	{
		case AUTOMATIC:
			if (manual)
			{
				robostate = MANUAL;
			}
			
			else if (kill)
			{
				robostate = KILL;
			}
			
			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
			
			else if (pause)
			{
				robostate = PAUSE;
				prev_state = AUTOMATIC;
			}
		break;
		
		case LOW_VOLTAGE:
			if (!low_voltage)
			{
				robostate = MANUAL;
			}
		break;

		case KILL:
			if (!kill && pause)
			{
				robostate = PAUSE;
				prev_state = MANUAL;
			}
			
			else if (!kill)
			{
				robostate = MANUAL;
			}
			
			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
		break;

		case PAUSE:
			if (!pause)
			{
				robostate = prev_state;
			}

			else if (kill)
			{
				robostate = KILL;
			}

			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
		break;
		
		case MANUAL:
			if (automatic)
			{
				robostate = AUTOMATIC;
			}
			else if (kill)
			{
				robostate = KILL;
			}
			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
			else if (pause)
			{
				robostate = PAUSE;
				prev_state = MANUAL;
			}
		break;
		
		default:
			robostate = MANUAL;
	}

	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	
	ros::Rate rate(2); //determine a rate that works for this ask matt!!!

	ros::Subscriber manualSub = nh.subscribe("manual_signal", 1, &robotController::manualCallback, robotController);
	ros::Subscriber autoSub = nh.subscribe("auto_signal", 1, &robotController::autoCallback, robotController);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("Twistmsg", 1)


	/*while(ros::ok())
	{
		robotController::robotController(state);

		rate.sleep();
	}*/
}

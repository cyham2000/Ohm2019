#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <vn/sensors/sensors.h>

#include <vn300/Pose.h>
#include <vn300/Velocities.h>
#include <vn300/Status.h>

// | 0  |  1  |  2  |  3  |  4 |  5  |  6  |  7  |  8  |  9  |  10  |  11  |  12  |  13  |  14  |  15  |
// | INS MODE |  ^  |     SENSOR ERROR     | N/A |  ^  |  ^  |                   N/A                   |       
//            | GPS FIX |                        |  ^  | GPS COMPASS |
//                                               | GPS HEADING INS |

// Bit masks to help extract the status info

#define INS_MODE_MASK 0x3
#define GPS_FIX_MASK 0x4
#define IMU_ERROR_MASK 0x10
#define MAGPRES_ERROR_MASK 0x20
#define GPS_ERROR_MASK 0x40
#define GPS_HEADING_INS_MASK 0x100
#define GPS_COMPASS_MASK 0x200

// class vn300_node
// Description: interfaces with vectornav sensor via vnproglib and outputs ros messages as they come in

class vn300_node {
	private:
		// vnproglib makes it impossible to use a class member as a packet handler easily.
		// so all these functions are friends to have access to the private members of a vn300_node but remain outside class
		// may use boost::bind at some point to get around that
		friend void vn300_packet_handler(void *userdata, vn::protocol::uart::Packet &p, size_t index);
		friend void vn300_error_handler(void *userdata, vn::protocol::uart::Packet &e, size_t index);
		friend std::string cookSensorError(vn::protocol::uart::SensorError e);

		vn::sensors::VnSensor sensor;

		ros::Publisher pose;
		ros::Publisher velocity;
		ros::Publisher status;

		ros::NodeHandle node;

		std::string device;
		int rate;

		void setup(int pose_rate, int vel_rate, int status_rate);

	public:
		vn300_node();
		~vn300_node() {
			sensor.disconnect();
		};

		bool ok() { return sensor.isConnected(); };

		// publishing wrappers so the handler can still publish

		void publish_pose(vn300::Pose msg) { pose.publish(msg); };
		void publish_velocity(vn300::Velocities msg) { velocity.publish(msg); };
		void publish_status(vn300::Status msg) { status.publish(msg); };
};

/*
	cookSensorError
	parameters:
		e - integer representing the sensor error
	return: error message string
*/

std::string cookSensorError(vn::protocol::uart::SensorError e) {
	using namespace vn::protocol::uart;	
	switch(e) {
		case ERR_HARD_FAULT:
			return "HARDWARE FAULT";
		break;
		case ERR_SERIAL_BUFFER_OVERFLOW:
			return "SERIAL BUFFER OVERFLOW";
		break;
		case ERR_INVALID_CHECKSUM:
			return "INVALID CHECKSUM";
		break;          
		case ERR_INVALID_COMMAND:
			return "INVALID COMMAND";
		break;          
		case ERR_NOT_ENOUGH_PARAMETERS:
			return "NOT ENOUGH PARAMETERS";
		break;
		case ERR_TOO_MANY_PARAMETERS:
			return "TOO MANY PARAMETERS";
		break;       
		case ERR_INVALID_PARAMETER:
			return "INVALID PARAMETER";
		break;    
		case ERR_INVALID_REGISTER:
			return "INVALID REGISTER";
		break; 
		case ERR_UNAUTHORIZED_ACCESS:
			return "UNAUTHORIZED ACCESS";
		break; 
		case ERR_WATCHDOG_RESET:
			return "WATCHDOG RESET";
		break;   
		case ERR_OUTPUT_BUFFER_OVERFLOW:
			return "OUTPUT BUFFER OVERFLOW";
		break;   
		case ERR_INSUFFICIENT_BAUD_RATE:
			return "INSUFFICIENT BAUD RATE";
		break;
		case ERR_ERROR_BUFFER_OVERFLOW:
			return "ERROR BUFFER OVERFLOW";
		break;
		default:
			return "What error?";
	}
}

/* 
	vn300_packet_handler
	parameters:
		userdata - (stores pointer to vn300_node object)
		p - the packet received from the sensor
		index - packet count so far
	description:
		receives binary packets from sensor, extracts data, and publishes it.
*/

void vn300_packet_handler(void *userdata, vn::protocol::uart::Packet &p, size_t index) {
	using namespace vn::sensors;
	using namespace vn::math;
	using namespace vn::protocol::uart;

	vn300_node *obj = (vn300_node *)userdata; // nasty, but it's literally the only way

	if(p.type() == Packet::TYPE_BINARY) {
		//ROS_INFO_THROTTLE(2, "Binary packet recevied");
		if(p.isCompatible(COMMONGROUP_POSITION, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_FIX, ATTITUDEGROUP_NONE, INSGROUP_NONE)) {	
			// p is a pose packet
			sensor_msgs::NavSatFix msg;

			msg.header.stamp = ros::Time::now();
			
			vec3d pos_lla = p.extractVec3d();
			uint8_t gps_fix = p.extractUint8();

			if(gps_fix < 2) 
			{
				msg.status.status = msg.status.STATUS_NO_FIX;
			}
			else
			{
				msg.status.status = gps_fix - 2;
			}
			
			msg.status.service = 5;

			msg.latitude = pos_lla[0];
			msg.longitude = pos_lla[1];
			msg.altitude = pos_lla[2];

			msg.position_covariance_type = 0;

			obj->publish_pose(msg);

		} else if(p.isCompatible(COMMONGROUP_QTN | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE, INSGROUP_NONE)) {			
			// p is a velocities packet
			sensor_msgs::Imu msg;
				
			msg.header.stamp = ros::Time::now();
		
			vec4f qtn = p.extractVec4f();
			vec3f ang_rate = p.extractVec3f();
			vec3f lin_accel = p.extractVec3f();
			
			msg.orientation.x = qtn[0];
			msg.orientation.y = qtn[1];
			msg.orientation.z = qtn[2];
			msg.orientation.w = qtn[3];

			msg.angular_velocity.x = ang_rate[0];
			msg.angular_velocity.y = ang_rate[1];
			msg.angular_velocity.z = ang_rate[2];

			msg.linear_acceleration.x = lin_accel[0];
			msg.linear_acceleration.y = lin_accel[1];
			msg.linear_acceleration.z = lin_accel[2];

			obj->publish_velocity(msg);

		} else if(p.isCompatible(COMMONGROUP_INSSTATUS, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NUMSATS | GPSGROUP_FIX, ATTITUDEGROUP_NONE, INSGROUP_NONE)) {			
			// p is a status packet
			vn300::Status msg;

			msg.header.stamp = ros::Time::now();
		
			uint16_t ins_stat = p.extractUint16();
			uint8_t gps_num_sats = p.extractUint8();
			uint8_t gps_fix = p.extractUint8();

			// ins status bit field can be found at top of program

			msg.ins_mode = (ins_stat & INS_MODE_MASK);
			msg.usingGPSHeading = (bool)(ins_stat & GPS_HEADING_INS_MASK);
			msg.gpsCompassActive = (bool)(ins_stat & GPS_COMPASS_MASK);
			msg.imu_error = (bool)(ins_stat & IMU_ERROR_MASK);
			msg.magpres_error = (bool)(ins_stat & MAGPRES_ERROR_MASK);
			msg.num_sats = gps_num_sats;
			msg.fix = gps_fix;
			msg.gps_error = (bool)(ins_stat & GPS_ERROR_MASK);

			obj->publish_status(msg);
			
		} else {
			//ROS_INFO("Unknown packet found");
		}
	} else {
		//ROS_DEBUG("Ascii packet received");
	}
}

/*
	vn300_error_handler
	parameters:	
		read vn300_packet_handler
*/

void vn300_error_handler(void *userdata, vn::protocol::uart::Packet &e, size_t index) {
	ROS_INFO("%s", cookSensorError(e.parseError()).c_str());
}

/*
	setup
	description: 
		sets up the binary registers to send out packets at at 10, 8, and 20 hz respectively. also registers the packet and error handlers with the library.
*/

void vn300_node::setup(int pose_rate, int vel_rate, int status_rate) {
	using namespace vn::sensors;
	using namespace vn::protocol::uart;

	BinaryOutputRegister pose_bor(
		ASYNCMODE_PORT1,
		20,
		COMMONGROUP_POSITION,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_FIX,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE			
	);

	BinaryOutputRegister velocities_bor(
		ASYNCMODE_PORT1,
		50,
		COMMONGROUP_QTN | COMMONGROUP_ANGULARRATE | COMMONGROUP_VELOCITY,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE
	);

	BinaryOutputRegister status_bor(
		ASYNCMODE_PORT1,
		20,
		COMMONGROUP_INSSTATUS,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NUMSATS | GPSGROUP_FIX,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE
	);

	sensor.writeAsyncDataOutputType(VNOFF);

	ROS_INFO("Connecting binary output 1 . . . ");
	sensor.writeBinaryOutput1(pose_bor);

	ROS_INFO("Connecting binary output 2 . . . ");
	sensor.writeBinaryOutput2(velocities_bor);

	ROS_INFO("Connecting binary output 3 . . . ");
	sensor.writeBinaryOutput3(status_bor);

	sensor.registerAsyncPacketReceivedHandler(this, vn300_packet_handler);
	sensor.registerErrorPacketReceivedHandler(NULL, vn300_error_handler);
}

/* 
	constructor
	description:
		attempts to connect to sensor. if sensor is not connected, throws vn::not_found. also advertises heading, position, and status.
*/

vn300_node::vn300_node() :
	device("/dev/ttyUSB0"),
	rate(115200)
{
	using namespace vn::sensors;
	using namespace vn::protocol::uart;

	int pose_hz = 10;
	int vel_hz = 8;
	int status_hz = 20;

	// params
	ros::NodeHandle nh_private("~");
	nh_private.param("device", device, device); // for roslaunch files
	nh_private.param("serial_rate", rate, rate);
	nh_private.param("pose_refresh_rate", pose_hz, pose_hz);
	nh_private.param("velocity_refresh_rate", vel_hz, vel_hz);
	nh_private.param("status_refresh_rate", status_hz, status_hz);
	
	// setup
	try {
		sensor.connect(device, rate); // tried to prevent it from aborting, but couldn't find a workaround.
	} catch(vn::not_found &e) {}

	ROS_INFO("Connected to %s at %d baud", device.c_str(), rate);

	pose = node.advertise< sensor_msgs::NavSatFix >("pose", true);
	velocity = node.advertise< sensor_msgs::Imu >("velocities", true);
	status = node.advertise< vn300::Status >("status", true);

	setup(pose_hz, vel_hz, status_hz);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "vn300_node");
	vn300_node gps;
	ros::Rate r(10);

	while(ros::ok() && gps.ok()) { 
		//ROS_INFO("Publishing jobs remaining: %d", gps.published()); // debug
		r.sleep(); 
	}

	return 0;	
}

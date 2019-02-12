#include <ros/ros.h>
#include <serial/serial.h>
#include <string>

class ArduinoStateComm
{
    public:
        // Description: Constructor for the ArduinoStateComm object
        // Parameters: A ROS node handle
        ArduinoStateComm(ros::NodeHandle& nodeHandle);

        // Description: Function disconnects the Arduino.
        void arduinoDisconnect();
        // Description: Function connects the Arduino.
        void arduinoConnect();
        // Description: Function sends a command to the Arduino.
        // Parameters: A string with the command to send
        void arduinoSendCommand(string command);
        // Description: Callback function for the robot state topic
        void robotStateReceived_callback(const ohm_igvc_msgs::RobotState& message);
        // Description: Callback function for the GPS status topic
        void gpsStatusReceived_callback(const vn300::Status& message);
        // Description: Function reads the from the Arduino pause, E-top, and kill
        void readEmergencyStates();

        // Public members for the Arduino port name and serial port object
        std::string arduinoPortName;
        serial::Serial *arduinoSerialPort;

    private:
        // Private members for the subscribers for the robot state and GPS
        ros::Subscriber robotStateSub;
        ros::Subscriber gpsStatusSub;

        // Private members for the emergency states read from the Arduino
        bool estopped;
        bool killed;
        bool paused;
}; // END of class Arduino state communication

ArduinoStateComm::ArduinoStateComm(ros::NodeHandle& nodeHandle)
{
    // Subscribe to topic. 1 = topic name, 2 = queue size, 3 = callback function, 4 = object to call function on
    robotStateSub = nodeHandle.subscribe("robotState", 1, &ArduinoStateComm::robotStateReceived_callback, this);
    gpsStatusSub = nodeHandle.subscribe("gpsStatus", 1, &ArduinoStateComm::gpsStatusReceived_callback, this);
} // END of ArduinoStateComm constructor

void ArduinoStateComm::arduinoDisconnect()
{
    if(arduinoSerialPort != NULL)
    {
	    delete arduinoSerialPort;
	    arduinoSerialPort = NULL;
	} // END of if the serial port is not null
} // END of arduinoDisconnect() function
	
void ArduinoStateComm::arduinoConnect()
{
    if(arduinoPortName.empty())
    {
        ROS_ERROR("Arduino serial port name is empty.");
        return;
	} // END of if the port name is empty
	
	arduinoDisconnect();
    
    // Create and configure new serial port
	arduinoSerialPort = new Serial();
	arduinoSerialPort->setPort(arduinoPortName);
	arduinoSerialPort->setBaudrate(9600);
	arduinoSerialPort->setBytesize(serial::eightbits);
	arduinoSerialPort->setParity(serial::parity_even);
	serial::Timeout to = serial::Timeout::simpleTimeout(10);
	arduinoSerialPort->setTimeout(to);
	
	arduinoSerialPort->open();
	ROS_INFO("Connected to Arduino.");
} // END of arduinoConnect() function
	
void ArduinoStateComm::arduinoSendCommand(string command)
{
	ROS_INFO("Sending Arduino commend: %s", command.c_str());
	arduinoSerialPort->write(command+"\r");
} // END of arduinoSendCommand() function

void ArduinoStateComm::robotStateReceived_callback(const ohm_igvc_msgs::RobotState& message)
{
    if(message.state == "A" || message.state == "M")
    {
        // Send robot state to Arduino
        arduinoSendCommand(message.state);
    } // END of if robot state is A or M
} // END of robotStateReceived_callback() function

void ArduinoStateComm::gpsStatusReceived_callback(const vn300::Status& message)
{
    // Send GPS status to Arduino
    arduinoSendCommand(message.state);
} // END of gpsStatusReceived_callback() function

void ArduinoStateComm::readEmergencyStates()
{
    std::bitset<8> emergencyStatesByte;
} // END of readEmergencyStates() function

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduino_state_comm");
    ros::NodeHandle nh;

    // Create object used to communicate states with the Ardunio.
    ArduinoStateComm arduinoObj(nh);

    // Get the serial port name from parameters or use default
    nh.param("arduino_serial_port", arduinoObj.arduinoPortName, std::string("/dev/ttyACM0"));

} // END of main() function
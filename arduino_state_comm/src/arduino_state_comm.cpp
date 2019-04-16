#include <ros/ros.h>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>
#include <string>
#include <sstream>

#include <ohm_igvc_msgs/RobotState.h>
#include <ohm_igvc_msgs/ArduinoInfo.h>
#include <vn300/Status.h>

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
        // Description: Function reads the from the Arduino
        void readArduino();
        // Description: Function publishes values read from the Arduino
        void publishArduinoInfo(int values[]);

        // Public members for the Arduino port name, serial port object, and listener
        std::string arduinoPortName;
        serial::Serial *arduinoSerialPort;
        serial::utils::SerialListener serialListener;

    private:
        // Private members for the subscribers for the robot state and GPS
        ros::Subscriber robotStateSub;
        ros::Subscriber gpsStatusSub;

        // Private member for the publisher for estop and kill states
        ros::Publisher arduinoKillStatePub;
        // Private member for the publisher for the batteries
        ros::Publisher arduinoBatteryPub;  
}; // END of class Arduino state communication

ArduinoStateComm::ArduinoStateComm(ros::NodeHandle& nodeHandle)
{
    // Subscribe to topic. 1 = topic name, 2 = queue size, 3 = callback function, 4 = object to call function on
    robotStateSub = nodeHandle.subscribe("robotState", 1, &ArduinoStateComm::robotStateReceived_callback, this);
    gpsStatusSub = nodeHandle.subscribe("gpsStatus", 1, &ArduinoStateComm::gpsStatusReceived_callback, this);

    // Publish to topic.arduinoInfoPub
    arduinoKillStatePub = nodeHandle.advertise<ohm_igvc_msgs::ArduinoKillState>("arduino_kill_state", 1);
    arduinoBatteryPub = nodeHandle.advertise<ohm_igvc_msgs::ArduinoBattery>("arduino_batteries", 1);
} // END of ArduinoStateComm constructor

void ArduinoStateComm::arduinoDisconnect()
{
    if(serialListener.isListening())
    {
        serialListener.stopListening();
    } // END of if the serial listener is listening
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

    serialListener.setChunkSize(2);
    serialListener.setartListening(*arduinoSerialPort);
	ROS_INFO("Connected to Arduino.");
} // END of arduinoConnect() function

void ArduinoStateComm::arduinoSendCommand(string command)
{
	ROS_INFO("Sending Arduino commend: %s", command.c_str());
	arduinoSerialPort->write(command+"\r\n");
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
    switch(message.fix)
    {
        case 0:
            arduinoSendCommand("O");
            break;
        case 1:
            arduinoSendCommand("S");
            break;
            case 2:
        case 3:
            arduinoSendCommand("F");
            break;
        default:
            break;
    } // END of switch
} // END of gpsStatusReceived_callback() function

void ArduinoStateComm::readArduino()
{
    // Create buffer filter pointer with delimiter of \r\n
    // As of 2-26-19: Each token is a string of numbers separated by commas
    //      1st = kill, 2nd = pause, 3-8 = cell voltage, 9 = temp
    BufferedFilterPtr bufferFilter = serialListener.createBufferedFilter(SerialListener::delimeter_tokenizer("\r\n"));

    string valueStr;
    double convertedValues[9];

    string line = bufferFilter->wait(50);
    if(!line.empty())
    {
        stringstream ss(line);

        for(int valueIndex = 0; valueIndex < 9; valueIndex++)
        {
            // Get the value using a commas a delimiter
            getline(ss, valueStr, ',');

            // Convert the string to an int
            stringstream convertToDouble(valueStr);
            convertToDouble >> convertedValues[valueIndex];
        } // END of for loop going through all the values

        publishArduinoInfo(convertedValues);
    } // END of if the line isn't empty
} // END of readArduino() function

void ArduinoStateComm::publishArduinoInfo(double values[])
{
    ohm_igvc_msgs::ArduinoKillState killStateMsg;
    killStateMsg.kill = (values[0] == 0) ? false : true;
    killStateMsg.pause = (values[1] == 0) ? false : true;

    ohm_igvc_msgs::ArduinoBattery batteryMsg;
    batteryMsg.cell1 = values[2];
    batteryMsg.cell2 = values[3];
    batteryMsg.cell3 = values[4];
    batteryMsg.cell4 = values[5];
    batteryMsg.cell5 = values[6];
    batteryMsg.cell6 = values[7];
    batteryMsg.temp = values[8];

    arduinoKillStatePub.publish(killStateMsg);
    arduinoBatteryPub.publish(batteryMsg);
} // END of publishArduinoInfo() function

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduino_state_comm");
    ros::NodeHandle nh;

    // Create object used to communicate states with the Arduino.
    ArduinoStateComm arduinoObj(nh);

    // Get the serial port name from parameters or use default
    nh.param("arduino_serial_port", arduinoObj.arduinoPortName, std::string("/dev/ttyACM0"));

    arduinoObj.arduinoConnect();
    arduinoObj.arduinoSendCommand("Start");

    ros::Rate rate(2);
    while(ros::ok())
    {
        arduinoObj.readArduino();
        ros::spin();
    } // END of while ros ok
} // END of main() function

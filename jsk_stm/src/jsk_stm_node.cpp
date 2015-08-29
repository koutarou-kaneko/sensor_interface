#include <ros/ros.h>
#include <jsk_stm/serial_comm.h>


int
main(int argc, char** argv)
{
    ros::init(argc, argv, "jsk_stm");
    ros::NodeHandle nh("jsk_stm_serial_comm");
    ros::NodeHandle nhp("~");

    std::string port_str;
    nhp.param("serial_port", port_str, std::string("/dev/ttyUSB0"));

    int baudrate;
    //nhp.param("baudrate", baudrate, 1382400); => bad
    nhp.param("baudrate", baudrate, 921600);

    std::string frame_id;
    nhp.param("frame_id", frame_id, std::string("/world"));

    SerialComm* comm;
    comm = new SerialComm(nh, nhp, frame_id);
    if (!comm->open(port_str, baudrate)) { return -1; }


    ros::spin();

    delete comm;

    return 0;
}


#include "SerialCom.h"

SerialCom::SerialCom()
{
	// Open File Descriptor 
	const char device[] = "/dev/ttyUSB1";

	m_dev = open(device, O_RDWR| O_NONBLOCK);

	while (m_dev < 0)
	{
		
		ROS_INFO("Error %d opening %s: %s. Could not connect to MCU, retrying...", 
				 errno, device, strerror(errno));

		usleep(2000*1000); 

		m_dev = open(device, O_RDWR| O_NONBLOCK);
		
	}

	// Configure port
	struct termios tty;
	memset (&tty, 0, sizeof(tty));

	if (tcgetattr (m_dev, &tty) != 0 )
	{
		std::cout << "Error " << errno 
		     	  << " from tcgetattr: " 
		     	  << strerror(errno) << std::endl;
	}

	// Set Baud Rate 
	cfsetospeed (&tty, B115200);
	cfsetispeed (&tty, B115200);

	// Set port settings
	tty.c_cflag     &=  ~CSTOPB;	//
	tty.c_cflag     &=  ~CRTSCTS;   // no flow control
	tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
	tty.c_oflag     =   0;      	// no remapping, no delays
	tty.c_cc[VMIN]  =   0;      	// read doesn't block
	tty.c_cc[VTIME] =   5;      	// 0.5 seconds read timeout

	tty.c_cflag     |=  CREAD | CLOCAL; // turn on READ & ignore ctrl lines
	tty.c_iflag     &=  ~IXOFF;			// turn off s/w flow ctrl

	// set terminal to raw mod
	cfmakeraw(&tty);
	// Flush Port, then applies attributes 
	tcflush(m_dev, TCIFLUSH );

	if (tcsetattr (m_dev, TCSANOW, &tty) != 0)
	{
		std::cout << "Error " 
				  << errno 
				  << " from tcsetattr" 
				  << std::endl;
	}

	// Allocate memory for read and write buffer
	memset(&m_read_buffer, '\0', sizeof(m_read_buffer));

	m_thruster_cmd[MAGIC_START_BYTE_INDEX] = MAGIC_START_BYTE;
	m_thruster_cmd[MAGIC_STOP_BYTE_INDEX] = MAGIC_STOP_BYTE;
	m_thruster_cmd[MSG_TYPE_INDEX] = MSG_TYPE_THRUSTER;

	m_heartbeat_cmd[MAGIC_START_BYTE_INDEX] = MAGIC_START_BYTE;
	m_heartbeat_cmd[MSG_HEARTBEAT_SIZE-1] = MAGIC_STOP_BYTE;
	m_heartbeat_cmd[MSG_TYPE_INDEX] = MSG_TYPE_HEARTBEAT;
}


int SerialCom::serial_write(char* cmd, int cmd_size)
{
	return write(m_dev, cmd, cmd_size);
}


int SerialCom::serial_read()
{
	return read(m_dev, &m_read_buffer, sizeof(m_read_buffer));
}


void SerialCom::thruster_pwm_callback(const vortex_msgs::Pwm& msg)
{
	int i;
	ROS_INFO("I heard: ");

	for(i = 0; i < 8; i++) ROS_INFO("%d ", msg.positive_width_us[i]);

	std::cout << "Command: "
			  << m_thruster_cmd[MAGIC_START_BYTE_INDEX] 
		 	  << m_thruster_cmd[MSG_TYPE_INDEX];


	for (i = 0; i < 8; i++)
	{
		m_thruster_cmd[MSG_PAYLOAD_START_INDEX + i*2] = (uint8_t)(msg.positive_width_us[i]  >> 8);
		m_thruster_cmd[MSG_PAYLOAD_START_INDEX + i*2 + 1] = (uint8_t)(msg.positive_width_us[i]  & 0xFF);
		printf(" %d ", m_thruster_cmd[i]);
		printf(" %d ", m_thruster_cmd[i+1]);
	}

	int16_t checksum = crc_checksum(&m_thruster_cmd[MSG_PAYLOAD_START_INDEX], 
									(MSG_PAYLOAD_STOP_INDEX - MSG_PAYLOAD_START_INDEX + 1));

	m_thruster_cmd[MSG_CRC_BYTE_INDEX] 	  = (uint8_t)(checksum  >> 8);
	m_thruster_cmd[MSG_CRC_BYTE_INDEX + 1] = (uint8_t)(checksum  & 0xFF);

	printf("%d %d", 
			m_thruster_cmd[MSG_CRC_BYTE_INDEX], 
			m_thruster_cmd[MSG_CRC_BYTE_INDEX+1]);

	std::cout << m_thruster_cmd[MAGIC_STOP_BYTE_INDEX] << std::endl;

	std::cout << "uint16_ t CRC_CHECKSUM = : " << checksum << std::endl;

	// Error Handling 
	if (serial_write(&m_thruster_cmd[0], MAX_MSG_SIZE) < 0)
	{
	    std::cout << "Error writing: " 
	              << strerror(errno) 
	              << std::endl;
	}

	if (serial_read() < 0)
	{
	    std::cout << "Error reading: " 
	              << strerror(errno) 
	              << std::endl;
	}
	else
	{
		std::cout << "Received over UART: " << m_read_buffer << std::endl;
		clear_read_buffer();
	}

}


void SerialCom::heartbeat_callback(const std_msgs::String::ConstPtr& msg)
{

	ROS_INFO("I heard: [%s]", msg->data.c_str());

	if(msg->data.c_str() == "F")
		m_heartbeat_cmd[MSG_TYPE_INDEX] = MSG_TYPE_ARM;
	else
		m_heartbeat_cmd[MSG_TYPE_INDEX] = MSG_TYPE_HEARTBEAT; 

	std::cout << "Command: "
		  << m_heartbeat_cmd[MAGIC_START_BYTE_INDEX] 
	 	  << m_heartbeat_cmd[MSG_TYPE_INDEX]
		  << m_heartbeat_cmd[MSG_HEARTBEAT_SIZE-1] << std::endl;


	// Error Handling 
	if (serial_write(&m_heartbeat_cmd[0], MSG_HEARTBEAT_SIZE) < 0)
	{
	    std::cout << "Error writing: " 
	              << strerror(errno) 
	              << std::endl;
	}
}

uint16_t SerialCom::crc_checksum(char *input, uint8_t num)
{
	return crc_16((const unsigned char*)input, (unsigned long)num);
}

void SerialCom::clear_read_buffer()
{
	for (int i = 0; i < sizeof(m_read_buffer); i++)
	{
		m_read_buffer[i] = 0;
	}
}

SerialCom::~SerialCom()
{
	close(m_dev);
}
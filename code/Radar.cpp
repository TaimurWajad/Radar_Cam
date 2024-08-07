#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <csignal>

#include "decoding_awr1843.h"

using namespace std;
#define SENDCFGSIZE 29
#define MAGIC_WORD_SIZE sizeof(short int) * 4
#define SLEEP_TIME_100MS 1000 * 100
#define SLEEP_TIME_50MS 1000 * 50
#define READ_BUF_SIZE 64000
std::vector<std::string> sendCfg;
std::string configFileName = "radar/profile.cfg";

extern "C" {
    int init_radar();
    int close_radar();
    int read_radar_data(float *radar_data, int *data_size);
}

int serial_port_cfg, serial_port_data;

void closePorts() {
    cout << "closing serial ports " << endl;
    string stopMessage = "sensorStop\n";
    write(serial_port_cfg, stopMessage.c_str(), stopMessage.size());
    close(serial_port_cfg);
    close(serial_port_data);
}

void signal_callback_handler(int signum) {
    closePorts();
    exit(signum);
}

int close_radar()
{
    closePorts();
    cout << "bye"<<endl;
    return 0;
}

int init_radar()
{
    std::string port_cfg = "/dev/ttyACM0";
    std::string port_data = "/dev/ttyACM1";
    struct termios tty;

    // Open first serial port for configuration
    serial_port_cfg = open(port_cfg.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (-1 == serial_port_cfg) {
        cout <<"serial_port_cfg open error!!! "<<errno <<endl;
    }

    // Get the current terminal settings
    if(tcgetattr(serial_port_cfg, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    // Set the baud rate for the terminal
    if(cfsetspeed(&tty, B115200) != 0) { // Set baud rate for /dev/ttyACM0
        printf("Error %i from cfsetspeed: %s\n", errno, strerror(errno));
    }
    // Apply the settings to the terminal
    if(tcsetattr(serial_port_cfg, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    // Open second serial port for data
    serial_port_data = open(port_data.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (-1 == serial_port_data) {
        cout <<"serial_port_data open error!!! "<<errno <<endl;
    }

    if(tcgetattr(serial_port_cfg, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    // Get the current terminal settings
    if(tcgetattr(serial_port_data, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    // Set the baud rate for the terminal
    if(cfsetspeed(&tty, B921600) != 0) { // Set different baud rate for /dev/ttyACM1
        printf("Error %i from cfsetspeed: %s\n", errno, strerror(errno));
    }
    // Apply the settings to the terminal
    if(tcsetattr(serial_port_data, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    cfsetspeed(&tty, B115200);

    if (tcsetattr(serial_port_cfg, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    signal(SIGINT, signal_callback_handler);
    std::ifstream configFile(configFileName);
    if (!configFile) {
        std::cerr << "Unable to open file " << configFileName;
        return 1; // or handle the error in another appropriate way
    }

    std::string line;
    while (std::getline(configFile, line)) {
        sendCfg.push_back(line);
    }

    configFile.close();

    // Write configuration to the first serial port
    for(const auto& mes : sendCfg)
    {
        std::string message = mes + "\n";
        int n = write(serial_port_cfg, message.c_str(), message.size());
        if (n < 0) {
            perror("Error writing to serial port");
            closePorts();
            return 1;
        }
        usleep(SLEEP_TIME_100MS);
    }
    cout<< "configuration is OK" << endl;

    return 0;
}


int read_radar_data(float *radar_data, int *data_size)
{
    std::vector<char> data;
    char read_buf [READ_BUF_SIZE];
    int count=0;

    //usleep(SLEEP_TIME_50MS);

    // Before reading from the serial port
    cout << " Reading from serial port, read_buf size: " << sizeof(read_buf) << endl;


    // Read from first serial port
    int n = read(serial_port_data, read_buf, sizeof(read_buf));
    // After reading from the serial port
    cout << " After reading from serial port, read count: " << n << endl;
    if (-1==n) {
        perror("Error reading from serial port");
        closePorts();
        return 1;
    }
    else if (n>0)
    {
        // Before inserting into tje vector
        cout << "Inserting into vector, read_buf size: " << n << " , data size: " << data.size() << endl;


        data.insert(data.end(), read_buf, read_buf + n);


        if (data.size()<350) {
            return 1;
        }

        uint32_t cur_pos = 0;
        std::string data_str(data.begin(), data.end());
        const unsigned int hdr_pos = data_str.find("\x02\x01\x04\x03\x06\x05\x08\x07");

        if ( string::npos == hdr_pos )
        {
            return 1;
        }

        cur_pos = hdr_pos;
        MmwDemo_output_message_header omh;
        cur_pos += sizeof(short int) * 4;  // this is for magicWord

        memcpy(&omh.version, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        memcpy(&omh.totalPacketLen, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        memcpy(&omh.platform, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        memcpy(&omh.frameNumber, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        memcpy(&omh.timeCpuCycles, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        memcpy(&omh.numDetectedObj, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        memcpy(&omh.numTLVs, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        memcpy(&omh.subFrameNumber, &data[cur_pos], sizeof(uint32_t));
        cur_pos += sizeof(uint32_t);

        cur_pos = hdr_pos + omh.mysize;

        vector<DPIF_PointCloudCartesian> pcc_array;

        unsigned int idx = 0;
        for (idx=0; idx < omh.numTLVs; idx++)
        {
            MmwDemo_output_message_tl tl;
            if (data.size() < (cur_pos+tl.mysize))
            {
                break;
            }

            memcpy(&tl.msg_type, &data[cur_pos], sizeof(uint32_t));
            memcpy(&tl.msg_length, &data[cur_pos + sizeof(uint32_t)], sizeof(uint32_t));

            cur_pos = cur_pos + tl.mysize;

            if (tl.msg_type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS)
            {
                for (unsigned int i = 0; i < omh.numDetectedObj; i++)
                {
                    DPIF_PointCloudCartesian pcc;
                    memcpy(&pcc, &data[cur_pos], sizeof(DPIF_PointCloudCartesian));
                    cur_pos += sizeof(DPIF_PointCloudCartesian);
                    pcc_array.push_back(pcc);
                }
            }
        }

        for (DPIF_PointCloudCartesian pcc: pcc_array) {
            *data_size +=3;
            radar_data[count]   = pcc.x;
            radar_data[count+1] = pcc.y;
            radar_data[count+2] = pcc.z;
            count+=3;


            float distance = sqrt(pcc.x * pcc.x + pcc.y * pcc.y + pcc.z * pcc.z);
            cout << "x: " << pcc.x << ", y: " << pcc.y << ", z: " << pcc.z << ", distance: " << distance << endl;
        }
        // Before erasing from the vector
        cout << "Erasing from vector, cur_pos: " << cur_pos << ", data size:" << data.size() << endl;
        data.erase(data.begin(), data.begin() + ((cur_pos< data.size())? cur_pos:data.size()));
    }

    return 0;
}


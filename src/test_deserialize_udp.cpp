#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <arpa/inet.h> // for ntohl/htonl
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define PORT 65000
#define BUFFER_SIZE 24 // 6*4 data*byte

static void serialize_float(unsigned char **, const float);
static float unserialize_float(unsigned char **);
static uint32_t pack754_32(float);
static float unpack754_32(uint32_t);
void bufferization_sample(void);

/* Buffer serialization functions */
static void serialize_float(unsigned char **buffer, const float data)
{
    const uint32_t packed = pack754_32(data);
    const uint32_t netend = htonl(packed);
    memcpy(*buffer, &netend, 4);
    *buffer += 4;
}

static float unserialize_float(unsigned char **buffer)
{
    uint32_t netend;
    memcpy(&netend, *buffer, 4);
    *buffer += 4;
    const uint32_t hostend = ntohl(netend);
    return unpack754_32(hostend);
}

/* Beej's IEEE 754 floating-point arithmetic serialization functions */
static uint32_t pack754_32(float f)
{
    const unsigned bits = 32, expbits = 8;
    float fnorm;
    int shift;
    uint32_t sign, exp, significand;
    unsigned significandbits = bits - expbits - 1;

    if (fabs(f) < 0.00001f)
        return 0;
    if (f < 0)
    {
        sign = 1;
        fnorm = -f;
    }
    else
    {
        sign = 0;
        fnorm = f;
    }

    shift = 0;
    while (fnorm >= 2.0f)
    {
        fnorm /= 2.0f;
        shift++;
    }
    while (fnorm < 1.0f)
    {
        fnorm *= 2.0f;
        shift--;
    }
    fnorm -= 1.0f;

    significand = (uint32_t)(fnorm * ((1U << significandbits) + 0.5f));
    exp = shift + ((1 << (expbits - 1)) - 1);
    return (sign << (bits - 1)) | (exp << (bits - expbits - 1)) | significand;
}

static float unpack754_32(uint32_t i)
{
    const unsigned bits = 32, expbits = 8;
    unsigned bias = (1 << (expbits - 1)) - 1;
    unsigned significandbits = bits - expbits - 1;
    if (i == 0)
        return 0.0f;

    float result = (float)(i & ((1U << significandbits) - 1));
    result /= (1U << significandbits);
    result += 1.0f;

    int shift = ((i >> significandbits) & ((1U << expbits) - 1)) - bias;
    while (shift > 0)
    {
        result *= 2.0f;
        shift--;
    }
    while (shift < 0)
    {
        result /= 2.0f;
        shift++;
    }

    return result * (((i >> (bits - 1)) & 1) ? -1.0f : 1.0f);
}

void bufferization_sample()
{
    unsigned char tx_args_ser[6 * 4];
    unsigned char *p = tx_args_ser;
    float vals[6] = {800.0f, 0.0f, 270.0f, 0.0f, 180.0f, 0.0f};
    for (int i = 0; i < 6; ++i)
        serialize_float(&p, vals[i]);
}

void unbufferization_sample(const unsigned char rx_ret_ser[6 * 4], float out[6])
{
    unsigned char *mvptr = (unsigned char *)rx_ret_ser;
    for (int i = 0; i < 6; ++i)
    {
        out[i] = unserialize_float(&mvptr);
    }
}

void unbufferization_sample()
{
    unsigned char rx_ret_ser[6 * 4];
    unsigned char *rx_ret_ser_mvptr = &rx_ret_ser[0];

    // Presume rx_ret_ser has valid serialized data

    // Unserializing values
    float deser_x = unserialize_float(&rx_ret_ser_mvptr);
    float deser_y = unserialize_float(&rx_ret_ser_mvptr);
    float deser_z = unserialize_float(&rx_ret_ser_mvptr);
    float deser_a = unserialize_float(&rx_ret_ser_mvptr);
    float deser_e = unserialize_float(&rx_ret_ser_mvptr);
    float deser_r = unserialize_float(&rx_ret_ser_mvptr);

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_pose_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian/arm2_8/reference", 10);
    ros::Rate loop_rate(500);  // Adjust to your expected frequency

    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    unsigned char buffer[BUFFER_SIZE];
    unsigned char* buffer_ptr;
    socklen_t len = sizeof(cliaddr);
    struct timeval timeout = {2, 0};

    int stream_active = 0;  // Stream status indicator

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return EXIT_FAILURE;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        return EXIT_FAILURE;
    }

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

    time_t last_receive_time = 0;

    while (ros::ok()) {

        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL,
                         (struct sockaddr*)&cliaddr, &len);
        time_t current_time = time(NULL);

        if (n == -1) {
            perror("No stream active");

            if (stream_active) {
                std::cout << "Stream is not active anymore" << std::endl;
                stream_active = 0;
            }

            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        double interval = difftime(current_time, last_receive_time);

        // Stream is active or it's the first reception
        if (last_receive_time == 0 || interval < 2.0) {
            stream_active = 1;
            std::cout << "Stream is active!" << std::endl;

            // Deserialize
            buffer_ptr = buffer;
            float deser[6];
            for (int i = 0; i < 6; ++i)
                deser[i] = unserialize_float(&buffer_ptr);

            // mm → m 
            float x = deser[0] / 1000.0f;
            float y = deser[1] / 1000.0f;
            float z = deser[2] / 1000.0f;

            // A E R notation for the angles + deg → rad
            float azimuth  = deser[3] * M_PI / 180.0f;
            float elevation = deser[4] * M_PI / 180.0f;
            float roll   = deser[5] * M_PI / 180.0f;
            
            std::cout << "azimuth: " << azimuth << "elevation: " << elevation << "range or roll?: " << roll << std::endl;

            tf::Quaternion q_a = tf::createQuaternionFromRPY(0, 0, azimuth);
            tf::Quaternion q_e = tf::createQuaternionFromRPY(0, elevation, 0);
            tf::Quaternion q_r = tf::createQuaternionFromRPY(roll, 0, 0);

            // Quaternion conversion
            tf::Quaternion q = q_a * q_e * q_r;
            q.normalize();

            // Prepare PoseStamped msg for RT CARTESIO
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "base_link";  // or other frame
            pose_msg.pose.position.x = x;
            pose_msg.pose.position.y = y;
            pose_msg.pose.position.z = z;
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pub.publish(pose_msg);
        }

        last_receive_time = current_time;
        ros::spinOnce();
        loop_rate.sleep();
    }

    close(sockfd);
    return 0;
    
}
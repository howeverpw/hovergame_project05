/*
*   HoverGames Challenge 1: Fight Fire with Flyers
*
*   Project: Air Strategist Companion
*   Author: Raul Alvarez-Torrico (raul@tecbolivia.com)
*   Date: 02/02/2020
*
*   @file mlx90614_sensor_node.cpp
*   @brief Reads ground temperature with a MLX90614 heta sensor
*   
*   The MLX90614 interfacing part of this code is based on code from this tutorial:
*   https://olegkutkov.me/2017/08/10/mlx90614-raspberry/
*/

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <linux/i2c-dev.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/console.h>

#include <iostream>
#include <sensor_msgs/Temperature.h>
#include <mavros_msgs/PositionTarget.h>

#include <tf/transform_datatypes.h>

#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    std::cout << "[MLX90614 NODE] Starting... " << std::endl;

    ros::NodeHandle nh;

    // Publishes the ground temperature taken from the air
    ros::Publisher mlx90614_temp_pub = nh.advertise<sensor_msgs::Temperature>
     ("/mlx90614/temp", 10);

    system("sudo chmod 777 /sys/module/i2c_bcm2708/parameters/combined");
    system("sudo echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined");

    int fdev = open("/dev/i2c-1", O_RDWR); // open i2c bus

    if (fdev < 0) {
        // fprintf(stderr, "Failed to open I2C interface %s Error: %s\n", dev_path, strerror(errno));
        fprintf(stderr, "Failed to open I2C interface Error: %s\n", strerror(errno));
        return -1;
    }

    unsigned char i2c_addr = 0x5A;

    // set slave device address, default MLX is 0x5A
    if (ioctl(fdev, I2C_SLAVE, i2c_addr) < 0) {
        fprintf(stderr, "Failed to select I2C slave device! Error: %s\n", strerror(errno));
        return -1;
    }

    // enable checksums control
    if (ioctl(fdev, I2C_PEC, 1) < 0) {
        fprintf(stderr, "Failed to enable SMBus packet error checking, error: %s\n", strerror(errno));
        return -1;
    }


    // trying to read something from the device unsing SMBus READ request

    // i2c_data data;
    i2c_smbus_data data;
    char command = 0x07; // command 0x06 is reading thermopile sensor, see datasheet for all commands

    // build request structure
    struct i2c_smbus_ioctl_data sdat = {
        .read_write = I2C_SMBUS_READ,
        .command = command,
        .size = I2C_SMBUS_WORD_DATA,
        .data = &data
    };

    sensor_msgs::Temperature temp_msg;

    ros::Rate rate(1.0); // Execution rate in Hz

    while(ros::ok()){

        // do actual request
        if (ioctl(fdev, I2C_SMBUS, &sdat) < 0) {
            fprintf(stderr, "Failed to perform I2C_SMBUS transaction, error: %s\n", strerror(errno));
            return -1;
        }

        // calculate temperature in Celsius by formula from datasheet
        double temp = (double) data.word;
        temp = (temp * 0.02)-0.01;
        temp = temp - 273.15;

        // print result
        // printf("Tamb = %04.2f\n", temp);
        temp_msg.temperature = temp;

        mlx90614_temp_pub.publish(temp_msg);

        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}
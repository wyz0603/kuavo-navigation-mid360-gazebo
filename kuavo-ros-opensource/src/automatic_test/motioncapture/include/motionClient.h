#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/unistd.h>
#include <dirent.h>
#include <iostream>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <chrono>

#include <vector>
#define MAX_PATH 256

#include "SeekerSDKTypes.h"
#include "SeekerSDKClient.h"
#include "Utility.h"

#include <thread>

#include "lcm_logger.h"
struct MotionData
{
    timespec time;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};
class MotionCaptureClient
{
public:
    MotionCaptureClient(char *serverIP, LogWriter *logger_ptr = nullptr);
    ~MotionCaptureClient()
    {
        stop();
    }
    void start();
    void stop();
    static void DataHandler(sFrameOfMocapData *data, void *pUserData); // receives data from the server
    void MessageHandler(int msgType, char *msg);                       // receives SeekerSDK error messages
    int CreateClient(char *szServerIP);
    void updateWithImu(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro);
    void initialTransform(const Eigen::Vector3d &offset);
    bool getData(MotionData &newdata);
    static bool running_flag;

private:
    char *szServerIPAddress;
    SeekerSDKClient *theClient = nullptr;
};

class TransformationManager
{
private:
    Eigen::Matrix4d initialTransformation; // Initial transformation matrix

public:
    TransformationManager()
    {
        // Initialize with an identity matrix as the default transformation
        initialTransformation = Eigen::Matrix4d::Identity();
    }

    // Set the initial transformation based on provided position and quaternion
    void setInitialTransformation(const Eigen::Vector3d &position, const Eigen::Quaterniond &quaternion, const Eigen::Vector3d &offset = Eigen::Vector3d(-0.023, 0, 0.781))
    {
        // Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();
        // // Eigen::Vector3d offset(-0.023, 0, 0.781); // 设置当前位置为里程计坐标系的offset位置
        // // Create a transformation matrix by combining rotation and translation
        // initialTransformation.block<3, 3>(0, 0) = rotationMatrix;
        // initialTransformation.block<3, 1>(0, 3) = position - rotationMatrix * offset;

        Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();

        // Extract yaw angle from the rotation matrix
        double yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

        // Recreate rotation matrix with modified yaw angle
        Eigen::Matrix3d modifiedRotationMatrix;
        modifiedRotationMatrix << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

        // Create a transformation matrix by combining rotation and translation for yaw and position only
        initialTransformation.block<3, 3>(0, 0) = modifiedRotationMatrix;
        initialTransformation.block<3, 1>(0, 3) = position - modifiedRotationMatrix * offset;
    }

    // Apply a transformation to a given pose (position and quaternion)
    void applyTransformation(Eigen::Vector3d &position, Eigen::Quaterniond &quaternion)
    {
        Eigen::Vector4d pose(position(0), position(1), position(2), 1.0);
        Eigen::Vector4d transformedPose = initialTransformation.inverse() * pose;
        Eigen::Vector3d transformedPosition = transformedPose.head<3>();
        position = transformedPosition;

        Eigen::Quaterniond initialQuaternion(initialTransformation.block<3, 3>(0, 0));
        quaternion = initialQuaternion * quaternion;
    }

    // Apply a transformation to a given linear velocity
    void applyVelocityTransformation(Eigen::Vector3d &velocity)
    {
        Eigen::Matrix3d rotationMatrix = initialTransformation.block<3, 3>(0, 0);
        velocity = rotationMatrix.inverse() * velocity;
    }
};

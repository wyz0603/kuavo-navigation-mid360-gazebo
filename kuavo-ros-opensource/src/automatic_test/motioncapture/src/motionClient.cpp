#include "motionClient.h"
#define TIME_DIFF(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9)) * 1000
#define TORSO_ID 1
#define LEFT_FOOT_ID 2
#define RIGHT_FOOT_ID 3
LogWriter *logger_ptr_ = nullptr;

TransformationManager transform_;
bool InitTransform_flag{true};
std::mutex data_mutx;
struct timespec updateTime;
static MotionData newestTorsoData;
std::vector<SlideFrameArray> BoneVelocityTrackerArray;
std::vector<SlideFrameArray> BoneAccelerationTrackerArray;
bool MotionCaptureClient::running_flag{false};
Eigen::Vector3d odom_offset = {-0.023, 0, 0.781};

MotionCaptureClient::MotionCaptureClient(char *serverIP, LogWriter *logger_ptr) : szServerIPAddress(serverIP)
{
    logger_ptr_ = logger_ptr;
}
void MotionCaptureClient::start()
{
    int iResult;
    iResult = CreateClient(szServerIPAddress);
    if (iResult != ErrorCode_OK)
    {
        std::cout << "Error initializing client. Exiting" << std::endl;
        return;
    }
    else
    {
        std::cout << "MotionCaptureClient initialized and ready." << std::endl;
    }
}
void MotionCaptureClient::stop()
{
    theClient->Uninitialize();
    running_flag = false;
}
// Establish a SeekerSDK Client connection
int MotionCaptureClient::CreateClient(char *szServerIP)
{
    // release previous server
    if (theClient != nullptr)
    {
        theClient->Uninitialize();
        delete theClient;
    }
    running_flag = false;

    // create SeekerSDK client
    theClient = new SeekerSDKClient();

    // print version info
    unsigned char ver[4];
    theClient->SeekerSDKVersion(ver);
    printf("SeekerSDK Sample Client 2.4.0.4177(SeekerSDK ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // Set callback handlers
    int retCode = -1;
    retCode = theClient->Initialize(szServerIP); // szMyIPAddress：Client IP (local IP)；szServerIP：Server IP (computer IP running motion capture software)

    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.Error code: %d. Exiting\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // print server info
        sServerDescription ServerDescription;
        memset(&ServerDescription, 0, sizeof(ServerDescription));
        theClient->GetServerDescription(&ServerDescription);
        if (!ServerDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("Successfully connected to server\n");
    }

    {
        sDataDescriptions *ps = nullptr;
        theClient->GetDataDescriptions(&ps);
        if (ps)
        {
            for (int dsIndex = 0; dsIndex < ps->nDataDescriptions; ++dsIndex)
            {
                const auto &dataDescription = ps->arrDataDescriptions[dsIndex];

                switch (dataDescription.type)
                {
                case Descriptor_RigidBody:
                    printf("RigidBody:%s\n", dataDescription.Data.RigidBodyDescription->szName);
                    break;
                default:
                    break;
                }
            }

            theClient->FreeDataDescriptions(ps);
            ps = nullptr;
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &updateTime);
    theClient->SetDataCallback(DataHandler, theClient); // this function will receive data from the server
    while (!running_flag)
        usleep(10000);
    return ErrorCode_OK;
}
void MotionCaptureClient::DataHandler(sFrameOfMocapData *data, void *pUserData) // receives data from the server
{

    SeekerSDKClient *pClient = (SeekerSDKClient *)pUserData;

    int i = 0;

    // FrameOfMocapData params
    bool bIsRecording = data->params & 0x01;
    bool bTrackedModelsChanged = data->params & 0x02;
    if (bIsRecording)
        printf("RECORDING\n");
    if (bTrackedModelsChanged)
        printf("Models Changed.\n");

    // decode to friendly string
    // char szTimecode[128] = "";
    // pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
    // printf("FrameNO:%d\tTimeStamp:%lld\n", data->iFrame, data->iTimeStamp);
    // static long long callback_time = 0;
    // static struct timespec rt0,rt1;
    // clock_gettime(CLOCK_MONOTONIC, &rt1);
    // std::cout << "realFPS:" << 1/(TIME_DIFF(rt0,rt1)/1000) << std::endl;
    // clock_gettime(CLOCK_MONOTONIC, &rt0);
    // double dt = (data->iTimeStamp - callback_time) / 1000.0;
    // std::cout << "fps:" << 1 / dt << std::endl;
    // callback_time = data->iTimeStamp;

    BoneVelocityTrackerArray.resize(data->nRigidBodies);
    BoneAccelerationTrackerArray.resize(data->nRigidBodies);

    // Print RigidBodies
    // printf("Markerset.RigidBodies [Count=%d]\n", data->nRigidBodies); // Number of Markerset.Skeleton(skeleton)
    // printf("\tid\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
    for (i = 0; i < data->nRigidBodies; i++)
    {
        // printf("\tRigidBody%d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
        //        data->RigidBodies[i].ID,
        //        data->RigidBodies[i].x,
        //        data->RigidBodies[i].y,
        //        data->RigidBodies[i].z,
        //        data->RigidBodies[i].qx,
        //        data->RigidBodies[i].qy,
        //        data->RigidBodies[i].qz,
        //        data->RigidBodies[i].qw);

        std::string bodyname;

        if (data->RigidBodies[i].ID == TORSO_ID)
        {
            bodyname = "capture/torso";
        }
        else if (data->RigidBodies[i].ID == LEFT_FOOT_ID)
        {
            bodyname = "capture/lfoot";
        }
        else if (data->RigidBodies[i].ID == RIGHT_FOOT_ID)
        {
            bodyname = "capture/rfoot";
        }
        else
        {
            continue;
        }

        Eigen::VectorXd record_all(4 + 3 + 3 + 3);
        Eigen::Vector4d record_q;
        Eigen::Vector3d record_p;
        Eigen::Vector3d record_pd;
        Eigen::Vector3d record_pdd;
        record_p << data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z;
        record_q << data->RigidBodies[i].qw, data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz;

        // calculate the velocity and acceleration
        BoneVelocityTrackerArray[i].Cache(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z);
        BoneAccelerationTrackerArray[i].Cache(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z);

        // Caution: Actually, you cat get velocity of frame 2 after you get frame 3's position, But it just has a little difference
        static CalculateVelocity method1(FPS, FrameFactor);     // FPS:60 FrameFactor:3
        static CalculateAcceleration method2(FPS, FrameFactor); // FPS:60 FrameFactor:3

        Vel boneVelocity;
        BoneVelocityTrackerArray[i].tryToCalculate(boneVelocity, method1);

        Accel boneAccleration;
        BoneAccelerationTrackerArray[i].tryToCalculate(boneAccleration, method2);

        // TODO
        record_pd << boneVelocity.Vx, boneVelocity.Vy, boneVelocity.Vz;
        record_pdd << boneAccleration.Ax, boneAccleration.Ay, boneAccleration.Az;
        record_all.segment(0, 4) = record_q;
        record_all.segment(4, 3) = 0.001 * record_p;    // translate from mm to m.
        record_all.segment(7, 3) = 0.001 * record_pd;   // translate from mm/s to m/s.
        record_all.segment(10, 3) = 0.001 * record_pdd; // translate from mm/s^2 to m/s^2.

        auto filterAbnormal = [](Eigen::VectorXd &vec) -> bool
        {
            bool lost_ = false;
            for (int i = 0; i < vec.size(); i++)
            {
                if (std::abs(vec[i]) > 1e4)
                {
                    vec[i] = 0;
                    lost_ = true;
                }
            }
            return lost_;
        };
        bool lost_tracking = filterAbnormal(record_all);
        if (lost_tracking)
        {
            std::cout << "MotionCapturer lost tracking! " << bodyname << std::endl;
        }

        if (data->RigidBodies[i].ID == TORSO_ID && !lost_tracking)
        {
            struct timespec rec_t0;
            clock_gettime(CLOCK_MONOTONIC, &rec_t0);
            MotionData newTorsoData;
            newTorsoData.time = rec_t0;
            Eigen::Quaterniond quat(record_q[0], record_q[1], record_q[2], record_q[3]);
            newTorsoData.quaternion = quat;
            newTorsoData.position = record_all.segment(4, 3);
            newTorsoData.velocity = record_all.segment(7, 3);
            if (InitTransform_flag)
            {
                transform_.setInitialTransformation(newTorsoData.position, newTorsoData.quaternion, odom_offset);
                InitTransform_flag = false;
                std::cout << "setInitialTransformation\n";
            }
            transform_.applyTransformation(newTorsoData.position, newTorsoData.quaternion);
            transform_.applyVelocityTransformation(newTorsoData.velocity);
            data_mutx.lock();
            newestTorsoData = newTorsoData;
            updateTime = rec_t0;
            if (!running_flag)
                running_flag = true;
            // std::cout << "newTorsoData.position:" << newTorsoData.position.transpose() << std::endl;
            data_mutx.unlock();
            if (logger_ptr_ != nullptr)
            {
                Eigen::Vector4d quat(newTorsoData.quaternion.w(), newTorsoData.quaternion.x(), newTorsoData.quaternion.y(), newTorsoData.quaternion.z());
                auto euler = newTorsoData.quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
                logger_ptr_->logData(bodyname + "/odom/euler", euler);
                logger_ptr_->logData(bodyname + "/odom/position", newTorsoData.position);
                logger_ptr_->logData(bodyname + "/odom/velocity", newTorsoData.velocity);
                logger_ptr_->logData(bodyname + "/odom/quaternion", quat);
            }
        }

        if (logger_ptr_ != nullptr)
            logger_ptr_->logData(bodyname + "/", record_all);
    }
    // auto now = std::chrono::system_clock::now();
    // auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     now.time_since_epoch());
    // std::cout << "time:" << ms.count() << std::endl;
    // std::cout << "TimeStamp diff:" << std::fixed << data->iTimeStamp - ms.count() << std::endl;
    if (logger_ptr_ != nullptr)
        logger_ptr_->logData("cat_time", static_cast<double>(data->iTimeStamp));
}
void MotionCaptureClient::updateWithImu(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    data_mutx.lock();
    double dt = TIME_DIFF(newestTorsoData.time, now) * 1e-3;
    newestTorsoData.velocity += acc * dt;
    newestTorsoData.position += newestTorsoData.velocity * dt;
    Eigen::Quaterniond dq(1.0, gyro.x() * dt / 2, gyro.y() * dt / 2, gyro.z() * dt / 2);
    newestTorsoData.quaternion = newestTorsoData.quaternion * dq;

    newestTorsoData.time = now;
    data_mutx.unlock();
}
void MotionCaptureClient::initialTransform(const Eigen::Vector3d &offset = odom_offset)
{
    odom_offset = offset;
    InitTransform_flag = true;
}
bool MotionCaptureClient::getData(MotionData &newdata)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (TIME_DIFF(updateTime, now) > 32)
        return false;
    std::unique_lock<std::mutex> lock(data_mutx);
    newdata = newestTorsoData;
    return true;
}
void MotionCaptureClient::MessageHandler(int msgType, char *msg) // receives SeekerSDK error messages
{
    printf("\n%s\n", msg);
}

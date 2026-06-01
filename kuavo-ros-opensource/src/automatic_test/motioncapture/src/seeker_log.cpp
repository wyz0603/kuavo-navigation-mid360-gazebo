#include "motionClient.h"
char szServerIPAddress[128] = "10.1.1.198";

int main(int argc, char *argv[])
{

    // init logger
    LogWriter *logWriter = new LogWriter();
    logWriter->start("/tmp/log_dyn.csv");
    MotionCaptureClient *motion = new MotionCaptureClient(szServerIPAddress, logWriter);
    motion->start();

    while (motion->running_flag)
    {
        sleep(1);
    }

    motion->stop();

    return ErrorCode_OK;
}

import pyaudio

def listAudioDevice(p: pyaudio.PyAudio):
    info = p.get_host_api_info_by_index(0)
    numDevices = info.get('deviceCount')

    for i in range(0, numDevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("输入设备 ID ", i, " - ",
                p.get_device_info_by_host_api_device_index(0, i).get('name'), "inputchannel:", p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels'))

        if (p.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels')) > 0:
            print("输出设备 ID ", i, " - ",
                p.get_device_info_by_host_api_device_index(0, i).get('name'), "outputchannel:", p.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels'))
    

if __name__ == "__main__":
    p = pyaudio.PyAudio()
    listAudioDevice(p)
    p.terminate()
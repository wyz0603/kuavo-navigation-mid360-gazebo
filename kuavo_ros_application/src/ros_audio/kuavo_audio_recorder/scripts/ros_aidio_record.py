#!/usr/bin/env python3
import os, sys, datetime, rospy, pyaudio, wave
from kuavo_audio_recorder.srv import recordmusic, recordmusicResponse

FORMAT = pyaudio.paInt16
CHANNELS = rospy.get_param('~channels', 1)
RATE = rospy.get_param('~rate', 16000)
CHUNK = 1024

def list_input_devices():
    pa = pyaudio.PyAudio()
    devices = []
    for i in range(pa.get_device_count()):
        dev = pa.get_device_info_by_index(i)
        if dev.get('maxInputChannels', 0) > 0:
            devices.append((i, dev['name']))
            rospy.loginfo(f"Input Device {i}: {dev['name']} (max ch: {dev['maxInputChannels']})")
    pa.terminate()
    return devices

def record_audio_file(idx, timeout, output_path):
    pa = pyaudio.PyAudio()
    stream = pa.open(format=FORMAT, channels=CHANNELS, rate=RATE,
                     input=True, frames_per_buffer=CHUNK,
                     input_device_index=idx)
    rospy.loginfo(f"Recording from device index {idx} for {timeout}s → {output_path}")
    frames = []
    for _ in range(int(RATE / CHUNK * (timeout+1))):
        data = stream.read(CHUNK, exception_on_overflow=False)
        frames.append(data)
    stream.stop_stream()
    stream.close()
    pa.terminate()

    wf = wave.open(output_path, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    return True

def handle_record_music(req):
    idx = rospy.get_param('~record_input_index', None)
    if idx is None:
        rospy.loginfo("Listing input devices for selection:")
        devs = list_input_devices()
        if not devs:
            rospy.logerr("No input devices found.")
            return recordmusicResponse(success_flag=False)
        idx = devs[0][0]
        rospy.set_param('~record_input_index', idx)
        rospy.loginfo(f"Defaulting to device index {idx}")

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{req.music_number}_{timestamp}.wav"
    filepath = os.path.join(os.getcwd(), filename)

    try:
        ok = record_audio_file(idx, req.time_out, filepath)
        return recordmusicResponse(success_flag=ok)
    except Exception as e:
        rospy.logerr(f"Recording failed: {e}")
        return recordmusicResponse(success_flag=False)

def record_music_server():
    rospy.init_node('record_music_node')
    rospy.Service('record_music', recordmusic, handle_record_music)
    rospy.loginfo("Recording service '/record_music' ready")
    rospy.spin()

if __name__ == "__main__":
    record_music_server()

import os
import shutil
from voice_synthesis import voiceSynthesis
from audio_tool import pcm2wav

soundList: dict = {
    "hello": ["你好", "sound/hello"],
    "speak": ["请说", "sound/speak"],
    "bye": ["再见", "sound/bye"],
}

def buildSoundList():
    import configparser

    configFile = configparser.ConfigParser()
    configFile.read("config.txt",encoding='utf-8')
    config = dict(configFile.items("config"))

    for name in soundList:
        text = soundList[name][0]
        filePath = soundList[name][1]
        print("[Text] {}".format(text))

        audioFile = voiceSynthesis(
            text=soundList[name][0],
            appid=config["synthesis_app_id"],
            apiKey=config["synthesis_api_key"],
            apiSecret=config["synthesis_api_secret"],
        )

        pcmPath = filePath + ".pcm"
        wavPath = filePath + ".wav"

        shutil.copy(audioFile, pcmPath)
        print("Saved file {}.".format(pcmPath))

        pcm2wav(
            pcm_path=pcmPath,
            out_path=wavPath,
            channel=1,
            sample_rate=16000,
        )
        print("Saved file {}.".format(wavPath))


if __name__ == "__main__":
    buildSoundList()

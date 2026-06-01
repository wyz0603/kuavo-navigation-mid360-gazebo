import os
import pygame
from audio_tool import pcm2wav
from sound_list import soundList


class AudioPlayer(object):
    def __init__(self, sampleRate: int = 16000, channel=1) -> None:
        self.sampleRate = sampleRate
        self.channel = channel

        pygame.mixer.pre_init(frequency=sampleRate, channels=channel)
        pygame.init()

        self.audio = pygame.mixer.Channel(0)

    def playWavFile(self, path: str):
        # 获取当前脚本的目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # 构建绝对路径
        abs_path = os.path.join(script_dir, path)
        sound = pygame.mixer.Sound(abs_path)
        self.audio.play(sound)

    def playPcmFile(self, path: str):
        wavPath = os.path.splitext(path)[0] + ".wav"
        pcm2wav(
            pcm_path=path,
            out_path=wavPath,
            channel=self.channel,
            sample_rate=self.sampleRate,
        )
        self.playWavFile(wavPath)

    def playFile(self, path: str):
        suffix = os.path.splitext(path)[1]
        if suffix == ".wav":
            self.playWavFile(path)
        elif suffix == ".pcm":
            self.playPcmFile(path)

    def playSoundList(self, name: str):
        self.playWavFile(soundList[name][1] + ".wav")

    def isPlaying(self):
        return self.audio.get_busy()

    def waitForPlayer(self):
        while self.isPlaying():
            pass


if __name__ == "__main__":
    player = AudioPlayer()

    while True:
        print("[Sound List]")
        soundKeys = list(soundList.keys())
        for i, soundName in enumerate(soundKeys):
            print("{} - {}: {}".format(i, soundName, soundList[soundName][0]))

        i = int(input("Choose a sound to play: "))

        if i < len(soundKeys):
            player.playSoundList(soundKeys[i])
            player.waitForPlayer()
        else:
            break

    pygame.quit()

#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo.core.audio import KuavoRobotAudioCore   

class KuavoRobotAudio:
    """Audio system interface for controlling audio playback functionality of Kuavo humanoid robot.
    
    Provides functionality to play music files.
    """
    
    def __init__(self):
        """Initialize the audio system."""
        self.audio = KuavoRobotAudioCore()

    def play_audio(self, file_name: str, volume: int = 100, speed: float = 1.0) -> bool:
        """Play the specified audio file.
        
        Args:
            file_name (str): Name of the audio file to play
                
        Returns:
            bool: True if the play request was successfully sent, False otherwise
        """
        return self.audio.play_audio(file_name, volume, speed)
    
    def stop_music(self):
        """Stop the currently playing audio."""
        return self.audio.stop_music()
    
    def text_to_speech(self, text: str, volume: float = 0.5) -> bool:
        """Synthesize and play the specified text.
        
        Args:
            text (str): Text to be played
                
        Returns:
            bool: True if the play request was successfully sent, False otherwise
        """
        return self.audio.text_to_speech(text, volume)

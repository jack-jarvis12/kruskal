import simpleaudio as sa
import os
from ament_index_python.packages import get_package_share_directory


# Function to play beep-boop sounds
def beep_boop(blocking=True):
    """
    Plays the next sound in the sequence of beep-boops.
    """
    print("Beep Boop!")
    wave_obj = beep_boop.sounds[beep_boop.next_sound]
    play_obj = wave_obj.play()
    if blocking:
        play_obj.wait_done()
    beep_boop.next_sound = (beep_boop.next_sound + 1) % len(beep_boop.sounds)


# Initialise the beep-boop sounds and state

package_share_directory = get_package_share_directory('voice_commands')

beep_boop.sounds = [
    sa.WaveObject.from_wave_file(os.path.join(package_share_directory, 'sounds', 'beep_boop_1.wav')),
    sa.WaveObject.from_wave_file(os.path.join(package_share_directory, 'sounds', 'beep_boop_2.wav')),
    sa.WaveObject.from_wave_file(os.path.join(package_share_directory, 'sounds', 'beep_boop_3.wav')),
]
beep_boop.next_sound = 0
    
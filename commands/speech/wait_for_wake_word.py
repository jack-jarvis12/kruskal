import os
import platform

import pvporcupine
import sounddevice as sd
from dotenv import load_dotenv

load_dotenv()


def wait_for_wake_word():
    """
    Detects a wake word using Picovoice Porcupine.
    """

    porcupine = None

    keyword_paths = ["speech/Crustal_en_windows_v3_0_0.ppn", "speech/Cross-call_en_windows_v3_0_0.ppn"]
    if platform.system() == "Linux":
        if platform.machine().startswith("arm") or platform.machine() == "aarch64":
            keyword_paths = ["speech/Crustal_en_raspberry-pi_v3_0_0.ppn", "speech/Cross-call_en_raspberry-pi_v3_0_0.ppn"]
        else:
            keyword_paths = ["speech/Crustal_en_linux_v3_0_0.ppn", "speech/Cross-call_en_linux_v3_0_0.ppn"]


    try:
        # Initialise Porcupine
        porcupine = pvporcupine.create(
            access_key=os.getenv("PICOVOICE_ACCESS_KEY"),
            keyword_paths=keyword_paths,
        )
        print("Porcupine initialised successfully.")

        wake_word_detected = False

        # Open the audio stream using 'with' statement
        with sd.InputStream(
            samplerate=porcupine.sample_rate,
            channels=1,
            dtype="int16",
            blocksize=porcupine.frame_length,  # Ensure the block size matches Porcupine frame length
        ) as stream:
            print("Listening for the wake word... Press Ctrl+C to exit.")

            while not wake_word_detected:
                # Read audio data from the stream
                audio_frame, _ = stream.read(porcupine.frame_length)
                pcm = audio_frame[:, 0]  # Use the first channel

                # Process the audio frame
                result = porcupine.process(pcm)
                if result >= 0:
                    print("Wake word detected!")
                    wake_word_detected = True

    except KeyboardInterrupt:
        print("Stopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Porcupine cleanup (no need for explicit delete in 'with' block)
        if porcupine:
            porcupine.delete()
        print("Resources released.")

    return wake_word_detected

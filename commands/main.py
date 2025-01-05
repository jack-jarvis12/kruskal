import environment
environment.check_env_vars()

import interpreter
import sounds
import speech


if __name__ == "__main__":
    try:
        while True:
            # Hangs until wake word detected or KeyboardInterrupt
            detected = speech.wait_for_wake_word()

            if detected:
                sounds.beep_boop(blocking=True)
                text = speech.transcribe_request()
                if text:
                    print("Recognized speech:", text)
                    command = interpreter.get_command(text)
                    print("Command:", command)
            else:
                print("Program terminated by user.")
                break

    except KeyboardInterrupt:
        print("Program terminated by user.")

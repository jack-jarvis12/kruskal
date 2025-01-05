import speech_recognition as sr

# Function to recognize speech
def transcribe_request():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for speech...")
        try:
            audio = recognizer.listen(source, timeout=5)
            try:
                return recognizer.recognize_google(audio)
            except sr.UnknownValueError:
                print("Could not understand the audio")
            except sr.RequestError as e:
                print(f"Could not request results; {e}")
        except sr.WaitTimeoutError:
            print("Listening timed out whilst waiting for speech")

        return ""
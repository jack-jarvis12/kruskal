import cv2
import numpy
import datetime
import simpleaudio as sa


# Function to play beep-boop sounds
def beep_boop(blocking=True):
    """
    Plays the next sound in the sequence of beep-boops.
    """
    if beep_boop.last_sound_time < datetime.datetime.now() - datetime.timedelta(minutes=1):
        print("Beep Boop!")
        wave_obj = beep_boop.sounds[beep_boop.next_sound]
        play_obj = wave_obj.play()
        if blocking:
            play_obj.wait_done()
        beep_boop.next_sound = (beep_boop.next_sound + 1) % len(beep_boop.sounds)
        beep_boop.last_sound_time = datetime.datetime.now()


# Initialise the beep-boop sounds and state
beep_boop.sounds = [
    sa.WaveObject.from_wave_file("beep_boop_1.wav"),
    sa.WaveObject.from_wave_file("beep_boop_2.wav"),
    sa.WaveObject.from_wave_file("beep_boop_3.wav"),
]
beep_boop.next_sound = 0
beep_boop.last_sound_time = datetime.datetime.now() - datetime.timedelta(seconds=56)

model_file = "res10_300x300_ssd_iter_140000.caffemodel" # Contains the trained weights of the model
config_file = "deploy.prototxt" # Defines the model architecture
net = cv2.dnn.readNetFromCaffe(config_file, model_file) # Loads the model into the Deep Neural Network module

video_capture = cv2.VideoCapture(0) # Tries USB camera first
if not video_capture.isOpened():
    video_capture = cv2.VideoCapture(0) # Defaults to integrated camera
if not video_capture.isOpened():
    raise Exception("Could not open video device")



while True:
    ret, frame = video_capture.read() # Read frame from webcam
    if not ret: # Kill if webcam disconnects
        break

    # Prepare the frame for the DNN model
    blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104.0, 177.0, 123.0), swapRB=False, crop=False)

    # Pass the frame through the network
    net.setInput(blob)
    detections = net.forward()
    number_of_detections = detections.shape[2]

    # Filter out the detections with low confidence
    confident_detections = []
    for i in range(number_of_detections):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            confident_detections.append(detections[0, 0, i])

    # Render the detection boxes
    for detection in confident_detections:
        box = detection[3:7] * numpy.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
        (x, y, x1, y1) = box.astype("int")
        cv2.rectangle(frame, (x, y), (x1, y1), (0, 255, 0), 2)


    # Display the frame
    cv2.imshow("Video", frame)

    # Make a sound if there is a person, and we haven't made a sound in the last minute
    if len(confident_detections) > 0:
        beep_boop(blocking=False)

    # Break the loop if 'q' is pressed or the window is closed
    if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty("Video", cv2.WND_PROP_VISIBLE) < 1:
        break

# Cleanup
video_capture.release()
cv2.destroyAllWindows()

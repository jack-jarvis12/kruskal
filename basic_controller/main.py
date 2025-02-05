import serial
import sys
import time
from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
# from gpiozero import PWMLED, Button
from signal import pause



class SerialDriver:
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, baudrate=57600, timeout=1)
        # pass

    def setMotorSpeedRPS(self, rps1, rps2):
        response = ""
        attempts = 0
        message = "m "+str(round(rps1*91.3))+" "+str(round(rps2*91.3))+"\r"
        while response == "" or attempts > 20:
            # print(message)
            self.ser.reset_input_buffer()
            self.ser.write(message.encode())
            response = self.ser.readline().decode()

        print("Response:", response)
        # print(rps1, rps2)
        

    def getMotorPosition(self):
        response = ""
        attempts = 0
        message = "e\r"
        while response == "" or attempts > 20:
            # print(message)
            self.ser.reset_input_buffer()
            self.ser.write(message.encode())
            response = self.ser.readline().decode()

        print("Position:", response)
        return response
        # return "1 2"

    def close(self):
        self.ser.close()




app = Flask(__name__, static_folder='static',)
app.config['SECRET_KEY'] = "KRUSKAL"
socketio = SocketIO(app)
driver = None

@app.route('/speed', methods=['GET'])
def speed():
    global driver
    try:
        # Get the query parameters
        rps1 = request.args.get('rps1', type=float)
        rps2 = request.args.get('rps2', type=float)

        # Validate inputs
        if rps1 is None or rps2 is None:
            return jsonify({"error": "Both rps1 and rps2 are required"}), 400

        driver.setMotorSpeedRPS(rps1, rps2)
        positions = driver.getMotorPosition().strip().split(" ")
        

        return jsonify(positions), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/position', methods=['GET'])
def position():
    global driver
    try:
        positions = driver.getMotorPosition().strip().split(" ")
        return jsonify(positions), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    


@socketio.on("speed")
def handle_message(msg):
    driver.setMotorSpeedRPS(float(msg["rps1"]), float(msg["rps2"]))

    positions = driver.getMotorPosition().strip().split(" ")
    emit("position", positions)

# Define the PWMLED and Button pins
# led = PWMLED(17)  # GPIO pin 17 for PWMLED
# button = Button(3)  # GPIO pin 3 for Button (updated)

# State tracking for the LED mode
# pulsing = True

# def toggle_led():
# 	global pulsing
# 	pulsing = not pulsing
# 	if not pulsing:
# 		led.value = 0.1
# 	else:
# 		led.pulse(fade_in_time=0.4, fade_out_time=0.6)

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <serial_port>")
        sys.exit(1)
    
    # button.when_pressed = toggle_led

    # toggle_led()

    global driver
    driver = SerialDriver(sys.argv[1])

    # app.run(host='0.0.0.0', port=5000)
    socketio.run(app, debug=True, host="0.0.0.0", port=5000)

    driver.close()

if __name__ == "__main__":
    main()

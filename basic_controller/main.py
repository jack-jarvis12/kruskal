import serial
import sys
import time
from flask import Flask, request, jsonify

class SerialDriver:
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, baudrate=57600, timeout=1)

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

    def close(self):
        self.ser.close()

class DiffDrive(SerialDriver):        
    def drive(self, speed):
        self.setMotorSpeedRPS(speed, speed)

    def turn(self, speed):
        self.setMotorSpeedRPS(speed, -speed)

    def stop(self):
        self.setMotorSpeedRPS(0, 0)
    


app = Flask(__name__, static_folder='static',)
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
        

        response =  jsonify(positions)
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response, 200
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
    

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <serial_port>")
        sys.exit(1)
    
    global driver
    driver = SerialDriver(sys.argv[1])

    app.run(host='0.0.0.0', port=5000)

    driver.close()

if __name__ == "__main__":
    main()
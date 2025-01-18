import serial
import sys
import time
from pynput import keyboard

class SerialDriver:
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, baudrate=57600, timeout=1)

    def setMotorSpeedRPS(self, rps1, rps2):
        response = ""
        attempts = 0
        message = "m "+str(rps1*91.3)+" "+str(rps2*91.3)+"\r"
        while response == "" or attempts > 20:
            self.ser.reset_input_buffer()
            self.ser.write(message.encode())
            response = self.ser.readline().decode()

        print("Response:", response)

    def getMotorPosition(self):
        response = ""
        attempts = 0
        message = "e"
        while response == "" or attempts > 20:
            self.ser.reset_input_buffer()
            self.ser.write(message.encode())
            response = self.ser.readline().decode()

        print("Response:", response)
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
    

    
def on_press(key, diffDrive):
    # print(f"Key {key} is Pressed.")
    try:
        if key.char == 'w':
            diffDrive.drive(1)
        elif key.char == "s":
            diffDrive.drive(-1)
        elif key.char == "d":
            diffDrive.turn(1)
        elif key.char == "a":
            diffDrive.turn(-1)
    except AttributeError:
        pass

def on_release(key, diffDrive):
    # print(f"Key {key} is released.")
    diffDrive.stop()
    diffDrive.getMotorPosition()

    if key == keyboard.Key.esc:
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <serial_port>")
        sys.exit(1)
    

    driver = DiffDrive(sys.argv[1])


    with keyboard.Listener(on_press=lambda key: on_press(key, driver), 
                       on_release=lambda key: on_release(key, driver)) as listener:
        listener.join()


    driver.close()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import environment
environment.check_env_vars()

import interpreter
import sounds
import speech

class VoiceCommandPublisher(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.get_logger().info('VoiceCommandPublisher node has been started.')

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandPublisher()
    try:
        while True:
            # Hangs until wake word detected or KeyboardInterrupt
            detected = speech.wait_for_wake_word()

            if detected:
                node.publish_command("<listen>")
                sounds.beep_boop(blocking=True)
                text = speech.transcribe_request()
                if text:
                    print("Recognized speech:", text)
                    command = interpreter.get_command(text)
                    print("Command:", command)
                    node.publish_command(command)
            else:
                print("Program terminated by user.")
                break
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

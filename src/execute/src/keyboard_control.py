#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
from key_mapping import Key_mapping
import time

EXIT_CODE = 8

class KeyBoardControl():
    def __init__(self):
        #super().__init__('keyboard_node')
        #rospy.init_node('keyboard_node')
        self.key_publisher = rospy.Publisher('keyboard_control',String, queue_size = 10)
        self.mapping_key = {
            # 'w',
            # 's',
            # 'a',
            # 'd',
            # ' ',
            # ',',
            # '.'
            Key_mapping.FORWARD,
            Key_mapping.BACK,
            Key_mapping.LEFT,
            Key_mapping.RIGHT,
            Key_mapping.ROTATE_LEFT,
            Key_mapping.ROTATE_RIGHT,
            Key_mapping.FAST,
            Key_mapping.SLOW,
            Key_mapping.TURN_45DEG_LEFT,
            Key_mapping.TURN_45DEG_RIGHT,
            Key_mapping.STOP,
            Key_mapping.COMMAND_SAVE_MAP
        }

    def press_handle(self, key):
        try:
            if key.char in self.mapping_key:
                self.public_direction(key.char)
        except:
            if key == key.esc:
                return False
                
        

    def release_handle(self, key):
        self.public_direction(Key_mapping.STOP)
            

    def key_board_handle(self):
        with Listener(on_press = self.press_handle) as listener:
            listener.join()
            
    def public_direction(self, command):
        msg = String()
        msg.data = str(command)
        self.key_publisher.publish(msg)
        rospy.loginfo("msg: {0}".format(msg))
            

def main():
    rospy.init_node('keyboard_control')

    keyboard_node = KeyBoardControl()
    keyboard_node.key_board_handle()
    if not keyboard_node :
        rospy.spin()
        

if __name__ == "__main__":
    main()

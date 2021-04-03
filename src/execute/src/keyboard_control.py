
import sys
import rospy
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
from key_mapping import Key_mapping

EXIT_CODE = 8

class KeyBoardControl():
    def __init__(self):
        #super().__init__('keyboard_node')
        #rospy.init_node('keyboard_node')
        self.key_publisher = rospy.Publisher('key_control',String, queue_size = 10)
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
            Key_mapping.TURN_45DEG_LEFT,
            Key_mapping.TURN_45DEG_RIGHT
        }

    def press_handle(self, key):
        try:
            if key.char in self.mapping_key:
                self.public_direction(key.char)    
        except:
            if key == key.ctrl or key == key.esc:
                return False
                
        

    def release_handle(self, key):
            self.public_direction(' ')
            

    def key_board_handle(self):
        with Listener(on_press = self.press_handle, on_release = self.release_handle) as listener:
            listener.join()
            
    def public_direction(self, command):
        msg = String()
        msg.data = str(command)
        self.key_publisher.publish(msg)
            

def main():
    rospy.init_node('keyboard_control')

    keyboard_node = KeyBoardControl()
    keyboard_node.key_board_handle()
    if  keyboard_node == 0:
        rospy.spin(keyboard_node)
        

if __name__ == "__main__":
    main()

#!/usr/bin/env python

from pynput.keyboard import Key, Listener, KeyCode
import curses
import rospy
from geometry_msgs.msg import Twist


class Main:
    def __init__(self):
        self.intro = """Keyboard Teleop...
                ---made by: SUYASH VERMA (Mr. Crazy Unknown)

        Controls:
        Classic WASD, but 'A' and 'D' are used for rotating rather than moving sideways,
        Also, use 'T' to increase and 'G' to decrease linear speed
        And, use 'Y' to increase and 'H' to decrease angular speed
        (Don't use Ctrl+C to exit. Use Esc. (If did ....close the current terminal tab and start a new one))
------------------------------------------------------------------------------------------------------------\n"""
        self.speed = 0.3
        self.rot = 3
        self.cmd_count = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.init_node('teleop', anonymous=True)
        self.flag = [0, 0, 0, 0]
        self.msg = Twist()
        self.screen = curses.initscr()
        self.screen_clear()
        self.screen.refresh()
        curses.noecho()
        curses.cbreak()

    def screen_clear(self):
        self.screen.clear()
        self.screen.refresh()
        if self.screen.getmaxyx()[0] > 10:
            self.screen.addstr(0, 0, self.intro)
            self.screen.refresh()
            self.cmd_count = 10
        else:
            self.screen.refresh()
            self.cmd_count = 0

    def change_speed(self, incr):
        if incr:
            self.speed += self.speed * 0.5
            text = "Speed increased by 50%"
        else:
            self.speed -= self.speed * 0.5
            text = "Speed decreased by 50%"
        self.screen.addstr(self.cmd_count, 0, text)
        self.screen.refresh()
        self.cmd_count += 1

    def change_rot(self, incr):
        if incr:
            self.rot += self.rot * 0.5
            text = "Rotation increased by 50%"
        else:
            self.rot -= self.rot * 0.5
            text = "Rotation decreased by 50%"
        self.screen.addstr(self.cmd_count, 0, text)
        self.screen.refresh()
        self.cmd_count += 1

    def on_press(self, key):
        try:
            if rospy.is_shutdown():
                exit(0)
            if key == KeyCode(char = 'w'):
                self.flag[0] = 1
            elif key == KeyCode(char = 'a'):
                self.flag[1] = 1
            elif key == KeyCode(char = 's'):
                self.flag[2] = 1
            elif key == KeyCode(char = 'd'):
                self.flag[3] = 1
            self.move()
        except rospy.ROSInterruptException as e:
            print(e)

    def on_release(self, key):
        try:
            if rospy.is_shutdown():
                exit(0)
            if key == KeyCode(char = 'w'):
                self.flag[0] = 0
            elif key == KeyCode(char = 'a'):
                self.flag[1] = 0
            elif key == KeyCode(char = 's'):
                self.flag[2] = 0
            elif key == KeyCode(char = 'd'):
                self.flag[3] = 0
            elif key == KeyCode(char = 't'):
                self.change_speed(incr=True)
            elif key == KeyCode(char = 'g'):
                self.change_speed(incr=False)
            elif key == KeyCode(char = 'y'):
                self.change_rot(incr=True)
            elif key == KeyCode(char = 'h'):
                self.change_rot(incr=False)
            if key == Key.esc:
                curses.nocbreak()
                self.screen.keypad(False)
                self.screen_clear()
                curses.flushinp()
                curses.echo()
                curses.endwin()
                return False
            self.move()
        except rospy.ROSInterruptException as e:
            print(e)

    def move(self):
        if(self.screen.getmaxyx()[0] <= self.cmd_count):
            self.screen_clear()
        self.msg.linear.x = (self.flag[0] - self.flag[2]) * self.speed
        self.msg.angular.z = (self.flag[1] - self.flag[3]) * self.rot
        self.pub.publish(self.msg)
        text = "linear x:" + str(round(self.msg.linear.x, 2))
        text += ", y:" + str(round(self.msg.linear.y, 2))
        text += ", z:" + str(round(self.msg.linear.z, 2))
        text += "  angular x:" + str(round(self.msg.angular.x, 2))
        text += ", y:" + str(round(self.msg.angular.y, 2))
        text += ", z:" + str((self.msg.angular.z, 2))
        self.screen.addstr(self.cmd_count, 0, text)
        self.screen.refresh()
        self.cmd_count += 1

    def main(self):
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()


if __name__ == "__main__":
    obj = Main()
    obj.main()

#!/usr/bin/env python2

import controller


def read_input():
    while True:
        a = input()
        if a == 1:
            controller.do_takeoff()
        elif a == 2:
            controller.do_land()
        elif a == 3:
            controller.rotate(float(input()), int(input()))
        elif a == 4:
            controller.go_forward(int(input()))
        elif a == 5:
            controller.go_backward(int(input()))
        elif a == 6:
            controller.go_left(int(input()))
        elif a == 7:
            controller.go_right(int(input()))
        elif a == 8:
            controller.go_up(int(input()))
        elif a == 9:
            controller.go_down(int(input()))
        elif a == 10:
            controller.look_up()
        elif a == 11:
            controller.look_down()
        elif a == 12:
            controller.look_right()
        elif a == 13:
            controller.look_left()
        elif a == 14:
            controller.reset_camera()
        elif a == 15:
            controller.publish_twist(float(input()), float(input()), float(input()), float(input()), int(input()))
        else:
            pass


if __name__ == '__main__':
    controller = controller.Controller("a")
    read_input()

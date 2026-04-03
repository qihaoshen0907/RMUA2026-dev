#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import termios
import tty
import select
import time
import threading

import rospy
from std_msgs.msg import Header
from airsim_ros.msg import VelCmd
from airsim_ros.srv import Takeoff, Land


class KeyboardTeleop(object):
    def __init__(self):
        rospy.init_node("keyboard_teleop")

        self.linear = rospy.get_param("~linear", 2.0)
        self.yaw_deg = rospy.get_param("~yaw_deg", 30.0)
        self.vertical = rospy.get_param("~vertical", 1.0)
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.idle_timeout = rospy.get_param("~idle_timeout", 0.25)
        self.auto_takeoff = rospy.get_param("~auto_takeoff", True)
        self.takeoff_settle = rospy.get_param("~takeoff_settle", 3.0)

        topic = rospy.get_param("~cmd_topic", "/airsim_node/drone_1/vel_body_cmd")
        takeoff_srv = rospy.get_param("~takeoff_service", "/airsim_node/drone_1/takeoff")
        land_srv = rospy.get_param("~land_service", "/airsim_node/drone_1/land")

        va = int(rospy.get_param("~va", 8))
        self._va = max(0, min(255, va))

        self._topic = topic
        self.pub = rospy.Publisher(topic, VelCmd, queue_size=1)

        self.lock = threading.Lock()
        self.vx = self.vy = self.vz = self.yaw = 0.0
        self.last_input = time.time()

        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)

        self._takeoff_proxy = rospy.ServiceProxy(takeoff_srv, Takeoff)
        self._land_proxy = rospy.ServiceProxy(land_srv, Land)

    def _do_takeoff(self):
        self._takeoff_proxy(waitOnLastTask=True)
        rospy.loginfo("takeoff sent")

    def _do_land(self):
        self._land_proxy(waitOnLastTask=True)
        rospy.loginfo("land sent")

    def _apply_key(self, ch):
        ch = ch.lower()

        if ch == "t":
            self._do_takeoff()
            self.last_input = time.time()
            return

        if ch == "g":
            with self.lock:
                self.vx = self.vy = self.vz = self.yaw = 0.0
            self._do_land()
            self.last_input = time.time()
            return

        with self.lock:
            if ch in ("k", " "):
                self.vx = self.vy = self.vz = self.yaw = 0.0
            elif ch == "w":
                self.vx = self.linear
            elif ch == "s":
                self.vx = -self.linear
            elif ch == "d": # ad 是反向的 
                self.vy = self.linear
            elif ch == "a":
                self.vy = -self.linear
            elif ch == "q":
                self.vz = self.vertical
            elif ch == "e":
                self.vz = -self.vertical
            elif ch == "j":
                self.yaw = self.yaw_deg
            elif ch == "l":
                self.yaw = -self.yaw_deg

        self.last_input = time.time()

    def _reader_loop(self):
        tty.setraw(self._fd)
        try:
            while not rospy.is_shutdown():
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    c = sys.stdin.read(1)
                    if c in ("\x03", "\x04"):
                        rospy.signal_shutdown("exit")
                        break
                    self._apply_key(c)
        finally:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def _maybe_idle_stop(self):
        if time.time() - self.last_input > self.idle_timeout:
            with self.lock:
                self.vx = self.vy = self.vz = self.yaw = 0.0

    def _publish(self, _evt):
        self._maybe_idle_stop()

        msg = VelCmd()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        with self.lock:
            msg.vx = self.vx
            msg.vy = self.vy
            msg.vz = self.vz
            msg.yawRate = self.yaw
            msg.va = self._va
            msg.stop = 0

        self.pub.publish(msg)

    def run(self):
        rospy.sleep(0.5)

        if self.auto_takeoff:
            self._do_takeoff()
            rospy.sleep(self.takeoff_settle)

        print("T takeoff | G land | WASD move | Q/E up/down | J/L yaw | K/SPACE stop | Ctrl+C exit")

        t = threading.Thread(target=self._reader_loop)
        t.daemon = True
        t.start()

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._publish)
        rospy.spin()


def main():
    try:
        KeyboardTeleop().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
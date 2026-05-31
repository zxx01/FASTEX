#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import threading

class VelocityPlotter(object):
    def __init__(self):
        # store time and velocity data
        self.time_data = []
        self.velocity_data = []
        self.max_data_len = 10000  # max number of historical data points
        self.start_time = None
        self.lock = threading.Lock()

        # initialize matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-', label="Linear Speed")
        self.avg_line = self.ax.axhline(y=0, color='b', linestyle='--', label="Average Speed")

        # configure chart title, labels, etc.
        self.ax.set_title("Real-time Velocity")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.legend()
        self.ax.grid(True)

        odometry_topic = rospy.get_param("~odometry_topic", "/quad_0/lidar_slam/odom")
        self.sub_odom = rospy.Subscriber(odometry_topic, Odometry, self.odom_callback)
        rospy.loginfo("OdomVisualizer subscribes to: %s", odometry_topic)

        # flag indicating whether data has been received
        self.data_received = False

    def odom_callback(self, msg):
        # get current time
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time

        # compute relative time
        relative_time = current_time - self.start_time

        # get magnitude of current robot linear velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        speed = np.sqrt(vx**2 + vy**2 + vz**2)

        with self.lock:
            # append new data to list
            self.time_data.append(relative_time)
            self.velocity_data.append(speed)

            # if data exceeds max_data_len, remove oldest entries
            if len(self.time_data) > self.max_data_len:
                self.time_data.pop(0)
                self.velocity_data.pop(0)

        # mark data as received
        self.data_received = True

    def update_plot(self):
        if not self.data_received:
            self.fig.canvas.flush_events()
            import time; time.sleep(0.1)
            return
        
        with self.lock:
            x_data = list(self.time_data)
            y_data = list(self.velocity_data)
        
        if len(y_data) > 0:
            avg_val = sum(y_data) / len(y_data)
            self.avg_line.set_ydata([avg_val, avg_val])

        self.line.set_data(x_data, y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main():
    rospy.init_node('velocity_plotter', anonymous=True)
    velocity_plotter = VelocityPlotter()

    while not rospy.is_shutdown():
        velocity_plotter.update_plot()

if __name__ == "__main__":
    main()

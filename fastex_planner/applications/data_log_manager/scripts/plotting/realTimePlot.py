#!/usr/bin/python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib as mpl
import rospy
from fastex_msgs.msg import ExploredVolumeTravedDistTime, IterationTime
import threading

mpl.rcParams['toolbar'] = 'None'


class RealTimePlot:
    def __init__(self):
        self.metrics = {
            'time_duration1': 0,
            'time_duration2': 0,
            'time_duration3': 0,
            'time_duration4': 0,
            'time_duration5': 0,
            'explored_volume': 0,
            'traveling_distance': 0,
            'total_run_time': 0,
            'preprocess_run_time': 0,
            'motion_run_time': 0,
            'max_explored_volume': 0,
            'max_traveling_distance': 0,
            'max_total_run_time': 0,
            'max_preprocess_run_time': 0,
            'max_motion_run_time': 0,
        }

        self.data_lists = {
            'time_list1': [0],
            'time_list2': [0],
            'time_list3': [],
            'time_list4': [],
            'time_list5': [],
            'explored_volume_list': [0],
            'traveling_distance_list': [0],
            'total_run_time_list': [],
            'preprocess_run_time_list': [],
            'motion_run_time_list': [],
        }

        plt.ion()
        self.fig, self.axes, self.lines = self._setup_plot()

        self.lock = threading.Lock()
        self.new_data = False

        rospy.init_node('realTimePlot')

        topic_exploration_data = rospy.get_param(
            "~exploration_data_topic",
            "/exploration_data_visualization/explored_volume_traved_dist_time")
        topic_total_run_time = rospy.get_param(
            "~total_run_time_topic",
            "/exploration_data_visualization/expl_total_run_time")
        topic_preprocess_run_time = rospy.get_param(
            "~preprocess_run_time_topic",
            "/exploration_data_visualization/expl_preprocess_run_time")
        topic_motion_run_time = rospy.get_param(
            "~motion_run_time_topic",
            "/exploration_data_visualization/expl_motion_run_time")
        rospy.Subscriber(topic_exploration_data,
                         ExploredVolumeTravedDistTime, self._exploration_data_callback, queue_size=2)
        rospy.Subscriber(topic_total_run_time, IterationTime,
                         self._iteration_total_time_callback, queue_size=2)
        rospy.Subscriber(topic_preprocess_run_time, IterationTime,
                         self._iteration__preprocess_time_callback2, queue_size=2)
        rospy.Subscriber(topic_motion_run_time, IterationTime,
                         self._iteration_motion_time_callback3, queue_size=2)
        rospy.loginfo("realTimePlot subscribes to: exploration=%s total=%s preprocess=%s motion=%s",
                      topic_exploration_data, topic_total_run_time, topic_preprocess_run_time,
                      topic_motion_run_time)

    def _exploration_data_callback(self, msg):
        with self.lock:
            self.metrics['time_duration1'] = msg.timeConsumed
            self.metrics['time_duration2'] = msg.timeConsumed
            self.metrics['explored_volume'] = msg.exploredVolume
            self.metrics['traveling_distance'] = msg.travelDist

            self.metrics['max_explored_volume'] = self.metrics['explored_volume']
            self.metrics['max_traveling_distance'] = self.metrics['traveling_distance']

            self.data_lists['time_list1'].append(
                self.metrics['time_duration1'])
            self.data_lists['explored_volume_list'].append(
                self.metrics['explored_volume'])
            self.data_lists['time_list2'].append(
                self.metrics['time_duration2'])
            self.data_lists['traveling_distance_list'].append(
                self.metrics['traveling_distance'])
            self.new_data = True

    def _iteration_total_time_callback(self, msg):
        with self.lock:
            self.metrics['total_run_time'] = msg.iterationTime
            self.metrics['time_duration3'] = msg.timeConsumed
            self.metrics['max_total_run_time'] = max(
                self.metrics['total_run_time'], self.metrics['max_total_run_time'])
            self.data_lists['time_list3'].append(
                self.metrics['time_duration3'])
            self.data_lists['total_run_time_list'].append(
                self.metrics['total_run_time'])
            self.new_data = True

    def _iteration__preprocess_time_callback2(self, msg):
        with self.lock:
            self.metrics['preprocess_run_time'] = msg.iterationTime
            self.metrics['time_duration4'] = msg.timeConsumed
            self.metrics['max_preprocess_run_time'] = max(
                self.metrics['preprocess_run_time'], self.metrics['max_preprocess_run_time'])
            self.data_lists['time_list4'].append(
                self.metrics['time_duration4'])
            self.data_lists['preprocess_run_time_list'].append(
                self.metrics['preprocess_run_time'])
            self.new_data = True

    def _iteration_motion_time_callback3(self, msg):
        with self.lock:
            self.metrics['motion_run_time'] = msg.iterationTime
            self.metrics['time_duration5'] = msg.timeConsumed
            self.metrics['max_motion_run_time'] = max(
                self.metrics['motion_run_time'], self.metrics['max_motion_run_time'])
            self.data_lists['time_list5'].append(
                self.metrics['time_duration5'])
            self.data_lists['motion_run_time_list'].append(
                self.metrics['motion_run_time'])
            self.new_data = True

    def _setup_plot(self):
        fig = plt.figure(figsize=(15, 8))
        plt.suptitle("Exploration Metrics\n", fontsize=18)
        plt.subplots_adjust(wspace=0.3, hspace=0.3)

        axes = []
        lines = []

        plot_configs = [
            {'title': 'Volume Time',
                'ylabel': "Explored\nVolume (m$^2$)", 'xlabel': "Time (s)"},
            {'title': 'Traveling Distance',
                'ylabel': "Traveling\nDistance (m)", 'xlabel': "Time (s)"},
            {'title': 'Volume Distance',
                'ylabel': "Explored\nVolume (m$^2$)", 'xlabel': "Traveling Distance (m)"},
            {'title': 'Algorithm Runtime',
                'ylabel': "Algorithm\nRuntime (s)", 'xlabel': "Time (s)"},
            {'title': 'Preprocess Runtime',
                'ylabel': "Preprocess\nRuntime (s)", 'xlabel': "Time (s)"},
            {'title': 'Motion Runtime',
                'ylabel': "Motion\nRuntime (s)", 'xlabel': "Time (s)"}
        ]

        colors = ['r', 'r', 'r', 'r', 'r', 'r']
        line_patterns = ['-', '-', '-', '', '', '']
        marker_patterns = ['', '', '', 'o', 'o', 'o']

        for i, config in enumerate(plot_configs):
            ax = fig.add_subplot(2, 3, i + 1)
            ax.set_title(config['title'])
            ax.set_ylabel(config['ylabel'], fontsize=10)
            ax.set_xlabel(config['xlabel'], fontsize=10)
            line, = ax.plot([], [], color=colors[i],
                            linestyle=line_patterns[i], marker=marker_patterns[i])
            axes.append(ax)
            lines.append(line)

        return fig, axes, lines

    def _update_subplot(self, idx, x_data, y_data, x_limit, y_limit, min_x_limit=0.1, min_y_limit=0.1):
        """Helper to update a single subplot"""
        self.lines[idx].set_data(x_data, y_data)
        self.axes[idx].set_xlim(0, max(x_limit * 1.1, min_x_limit))
        self.axes[idx].set_ylim(0, max(y_limit * 1.1, min_y_limit))

    def _update_plot(self):
        # if no new data or data already processed, just handle events
        if not self.new_data:
            self.fig.canvas.flush_events()
            import time; time.sleep(0.1)
            return

        # create a snapshot of data under lock protection
        with self.lock:
            data_snapshot = {k: list(v) for k, v in self.data_lists.items()}
            metrics_snapshot = self.metrics.copy()
            self.new_data = False  # reset flag

        try:
            # 0. Volume Time
            self._update_subplot(0, data_snapshot['time_list1'], data_snapshot['explored_volume_list'],
                                 metrics_snapshot['time_duration1'], metrics_snapshot['max_explored_volume'])

            # 1. Traveled Distance
            self._update_subplot(1, data_snapshot['time_list2'], data_snapshot['traveling_distance_list'],
                                 metrics_snapshot['time_duration2'], metrics_snapshot['max_traveling_distance'])

            # 2. Volume Distance
            # NOTE: ensure list lengths are consistent; slight differences may exist due to different data sources
            min_len_vol_dist = min(len(data_snapshot['traveling_distance_list']),
                                   len(data_snapshot['explored_volume_list']))
            self._update_subplot(2, data_snapshot['traveling_distance_list'][:min_len_vol_dist],
                                 data_snapshot['explored_volume_list'][:min_len_vol_dist],
                                 metrics_snapshot['max_traveling_distance'], metrics_snapshot['max_explored_volume'])

            # 3. Algorithm Runtime (min_y_limit=0.01)
            self._update_subplot(3, data_snapshot['time_list3'], data_snapshot['total_run_time_list'],
                                 metrics_snapshot['time_duration3'], metrics_snapshot['max_total_run_time'], min_y_limit=0.01)

            # 4. Preprocess Runtime (min_y_limit=0.01)
            self._update_subplot(4, data_snapshot['time_list4'], data_snapshot['preprocess_run_time_list'],
                                 metrics_snapshot['time_duration4'], metrics_snapshot['max_preprocess_run_time'], min_y_limit=0.01)
            # 5. Motion Runtime (min_y_limit=0.01)
            self._update_subplot(5, data_snapshot['time_list5'], data_snapshot['motion_run_time_list'],
                                 metrics_snapshot['time_duration5'], metrics_snapshot['max_motion_run_time'], min_y_limit=0.01)

        except Exception as e:
            rospy.logwarn("Plotting error: {}".format(e))

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self):
        while not rospy.is_shutdown():
            self._update_plot()


if __name__ == '__main__':
    plotter = RealTimePlot()
    plotter.run()

#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import Mavlink


class ImuRateTest(Node):

    def __init__(self):
        super().__init__('imu_rate_test')

        self.counts = {
            27: 0,
            116: 0,
            129: 0
        }

        self.first_time = {
            27: None,
            116: None,
            129: None
        }

        self.last_time = {
            27: None,
            116: None,
            129: None
        }

        self.intervals = {
            27: [],
            116: [],
            129: []
        }

        self.start_time = time.monotonic()
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Mavlink,
            '/uas1/mavlink_source',
            self.mavlink_callback,
            qos
        )

        self.timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info(
            'Monitoring MAVLink IDs 27, 116 and 129...'
        )

    def mavlink_callback(self, msg):

        msg_id = msg.msgid

        if msg_id not in self.counts:
            return

        now = time.monotonic()

        self.counts[msg_id] += 1

        if self.first_time[msg_id] is None:
            self.first_time[msg_id] = now
        else:
            interval = now - self.last_time[msg_id]
            self.intervals[msg_id].append(interval)

        self.last_time[msg_id] = now

    def print_status(self):

        elapsed = time.monotonic() - self.start_time

        print()
        print('==============================================')
        print(f'Elapsed time: {elapsed:.1f} s')

        names = {
            27: 'RAW_IMU',
            116: 'SCALED_IMU2',
            129: 'SCALED_IMU3'
        }

        for msg_id in [27, 116, 129]:

            count = self.counts[msg_id]

            if self.first_time[msg_id] is not None:
                duration = self.last_time[msg_id] - self.first_time[msg_id]

                if duration > 0:
                    rate = (count - 1) / duration
                else:
                    rate = 0.0
            else:
                rate = 0.0

            if self.intervals[msg_id]:
                avg_interval = (
                    sum(self.intervals[msg_id])
                    / len(self.intervals[msg_id])
                )

                min_interval = min(self.intervals[msg_id])
                max_interval = max(self.intervals[msg_id])
            else:
                avg_interval = 0.0
                min_interval = 0.0
                max_interval = 0.0

            print(
                f'{names[msg_id]:15s} '
                f'ID={msg_id:3d} '
                f'count={count:5d} '
                f'rate={rate:7.2f} Hz '
                f'avg_dt={avg_interval*1000:7.2f} ms '
                f'min={min_interval*1000:7.2f} ms '
                f'max={max_interval*1000:7.2f} ms'
            )

        print('==============================================')
        print()


def main(args=None):

    rclpy.init(args=args)

    node = ImuRateTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

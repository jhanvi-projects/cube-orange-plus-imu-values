#!/usr/bin/env python3

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Mavlink


class ImuSplitter(Node):

    def __init__(self):
        super().__init__('imu_splitter')
        
        mavlink_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Subscribe to raw MAVLink packets coming from MAVROS
        self.subscription = self.create_subscription(
            Mavlink,
            '/uas1/mavlink_source',
            self.mavlink_callback,
            mavlink_qos
        )

        # Three separate ROS IMU publishers
        self.raw_imu_pub = self.create_publisher(
            Imu,
            '/imu/raw_imu',
            10
        )

        self.imu2_pub = self.create_publisher(
            Imu,
            '/imu/scaled_imu2',
            10
        )

        self.imu3_pub = self.create_publisher(
            Imu,
            '/imu/scaled_imu3',
            10
        )

        self.get_logger().info('IMU splitter started')
        self.get_logger().info(
            'Listening to /uas1/mavlink_source'
        )

    def payload_to_bytes(self, payload64, length):
        """
        Convert MAVROS uint64[] payload into raw MAVLink payload bytes.
        MAVLink payload is little-endian.
        """

        data = b''

        for value in payload64:
            data += int(value).to_bytes(
                8,
                byteorder='little',
                signed=False
            )

        return data[:length]

    def create_imu_message(
        self,
        acceleration,
        gyro,
        frame_id
    ):
        """
        Create sensor_msgs/Imu message.
        """

        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        # Orientation is not provided by these raw IMU messages.
        msg.orientation_covariance[0] = -1.0

        # Convert acceleration from mg to m/s^2
        msg.linear_acceleration.x = acceleration[0] * 0.00980665
        msg.linear_acceleration.y = acceleration[1] * 0.00980665
        msg.linear_acceleration.z = acceleration[2] * 0.00980665

        # Convert gyro from mrad/s to rad/s
        msg.angular_velocity.x = gyro[0] * 0.001
        msg.angular_velocity.y = gyro[1] * 0.001
        msg.angular_velocity.z = gyro[2] * 0.001

        return msg

    def decode_raw_imu(self, data):
        """
        MAVLink RAW_IMU (ID 27)

        Payload layout:
        time_usec : uint64
        xacc      : int16
        yacc      : int16
        zacc      : int16
        xgyro     : int16
        ygyro     : int16
        zgyro     : int16
        xmag      : int16
        ymag      : int16
        zmag      : int16
        """

        if len(data) < 26:
            return None

        values = struct.unpack_from(
            '<Qhhhhhhhhh',
            data,
            0
        )

        xacc = values[1]
        yacc = values[2]
        zacc = values[3]

        xgyro = values[4]
        ygyro = values[5]
        zgyro = values[6]

        return self.create_imu_message(
            (xacc, yacc, zacc),
            (xgyro, ygyro, zgyro),
            'imu_raw'
        )

    def decode_scaled_imu(self, data, frame_id):
        """
        MAVLink SCALED_IMU2 (ID 116)
        MAVLink SCALED_IMU3 (ID 129)

        Payload layout:
        time_boot_ms : uint32
        xacc         : int16
        yacc         : int16
        zacc         : int16
        xgyro        : int16
        ygyro        : int16
        zgyro        : int16
        xmag         : int16
        ymag         : int16
        zmag         : int16
        """

        if len(data) < 22:
            return None

        values = struct.unpack_from(
            '<Ihhhhhhhhh',
            data,
            0
        )

        xacc = values[1]
        yacc = values[2]
        zacc = values[3]

        xgyro = values[4]
        ygyro = values[5]
        zgyro = values[6]

        return self.create_imu_message(
            (xacc, yacc, zacc),
            (xgyro, ygyro, zgyro),
            frame_id
        )

    def mavlink_callback(self, msg):

        # MAVLink message ID
        msgid = msg.msgid

        try:

            data = self.payload_to_bytes(
                msg.payload64,
                msg.len
            )

            # -------------------------------------------------
            # ID 27 = RAW_IMU
            # -------------------------------------------------

            if msgid == 27:

                imu_msg = self.decode_raw_imu(data)

                if imu_msg is not None:
                    self.raw_imu_pub.publish(imu_msg)

            # -------------------------------------------------
            # ID 116 = SCALED_IMU2
            # -------------------------------------------------

            elif msgid == 116:

                imu_msg = self.decode_scaled_imu(
                    data,
                    'imu_scaled_2'
                )

                if imu_msg is not None:
                    self.imu2_pub.publish(imu_msg)

            # -------------------------------------------------
            # ID 129 = SCALED_IMU3
            # -------------------------------------------------

            elif msgid == 129:

                imu_msg = self.decode_scaled_imu(
                    data,
                    'imu_scaled_3'
                )

                if imu_msg is not None:
                    self.imu3_pub.publish(imu_msg)


        except Exception as e:

            self.get_logger().error(
                f'Error decoding MAVLink IMU: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ImuSplitter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

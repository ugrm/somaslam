#!/usr/bin/env python3

import rosbag
import pandas as pd
import numpy as np
from tf.transformations import euler_from_quaternion

def extract_lidar_pose(bag_file, scan_topic, tf_topic, parent_frame, child_frame, output_csv):
    bag = rosbag.Bag(bag_file)
    poses = {}
    data = []

    # Extract poses from /tf
    for topic, msg, t in bag.read_messages(topics=[tf_topic]):
        for transform in msg.transforms:
            if (transform.header.frame_id == parent_frame and 
                transform.child_frame_id == child_frame):

                ts_sec = transform.header.stamp.to_sec()
                trans = transform.transform.translation
                rot = transform.transform.rotation
                (_, _, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

                poses[ts_sec] = (trans.x, trans.y, yaw)

    pose_times = np.array(list(poses.keys()))

    # Extract lidar scans and match nearest pose
    for topic, msg, t in bag.read_messages(topics=[scan_topic]):
        ts_sec = msg.header.stamp.to_sec()
        nearest_pose_ts = pose_times[np.argmin(np.abs(pose_times - ts_sec))]
        x, y, yaw = poses[nearest_pose_ts]

        ranges = msg.ranges

        entry = {
            'timestamp': ts_sec,
            'x': x,
            'y': y,
            'theta': yaw,
        }
        entry.update({f'range_{i}': r for i, r in enumerate(ranges)})

        data.append(entry)

    bag.close()

    # Save to CSV
    df = pd.DataFrame(data)
    df.to_csv(output_csv, index=False)
    print(f"Data clearly saved to {output_csv}")

if __name__ == "__main__":
    bag_file = "Team_Hector_MappingBox_Dagstuhl_Neubau.bag"
    output_csv = "lidar_pose.csv"

    scan_topic = "/scan"
    tf_topic = "/tf"
    parent_frame = "odom"
    child_frame = "base_link"

    extract_lidar_pose(bag_file, scan_topic, tf_topic, parent_frame, child_frame, output_csv)

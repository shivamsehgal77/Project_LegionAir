import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilterNode:
    def __init__(self):
        rospy.init_node("kalman_filter_node", anonymous=True)

        # Kalman filter for x, y, z
        self.kf = KalmanFilter(dim_x=6, dim_z=3)  # Assuming measurement frequency of 5 Hz

        dt = 1.0 / 30

        self.kf.F = np.array([
            [1, dt, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, dt, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, dt],
            [0, 0, 0, 0, 0, 1]
        ])

        self.kf.H = np.array([[1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0]])

        # Process noise covariance
        self.kf.Q = np.eye(6) * 0.01  # Assuming process noise is small

        # Measurement noise covariance
        self.kf.R = np.eye(3) * 0.1  # Assuming measurement noise is small

        # Publisher for predicted positions
        self.predicted_pos_pub = rospy.Publisher(
            "/predicted_positions", PointStamped, queue_size=10
        )

        # Subscriber to detections topic
        rospy.Subscriber("/detections", PointStamped, self.detections_callback)
        rospy.Subscriber("/object_available", Bool, self.bbox_callback)
        self.last_pcl_callback_time = rospy.get_time()
        self.object_available = False
        self.measurements = None
        self.det_counter = 0
        self.det_availabe = False
    def bbox_callback(self, msg):
        self.det_availabe = msg.data
    def detections_callback(self, msg):
        self.last_pcl_callback_time = rospy.get_time()
        if self.det_counter < 5:
            rospy.loginfo("I m intializing")
            init_meas = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.kf.update(init_meas)
            self.det_counter+=1
        else:
            self.measurements = np.array([
                msg.point.x,
                msg.point.y,
                msg.point.z,
            ])

    def run(self):
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            if self.det_counter >= 5 and self.det_availabe is True:
                # Check if new data has been received within the last second
                current_time = rospy.get_time()
                print("Time: ", current_time - self.last_pcl_callback_time)
                # print("Time diff: ", current_time - self.last_pcl_callback_time)
                if (current_time - self.last_pcl_callback_time > (1.0 / 30)):
                    # rospy.loginfo("Object is not available. Predicting...")
                    self.kf.predict()
                    predicted_state = self.kf.x
                    if predicted_state is not None:
                        # Do something with the predicted state
                        rospy.loginfo(
                            "Prediction: x={}, y={}, z={}".format(
                                predicted_state[0], predicted_state[2], predicted_state[4]
                            )
                        )

                        # Publish predicted positions
                        predicted_pos_msg = PointStamped()
                        predicted_pos_msg.header.stamp = rospy.Time.now()
                        predicted_pos_msg.point.x = predicted_state[0]
                        predicted_pos_msg.point.y = predicted_state[2]
                        predicted_pos_msg.point.z = predicted_state[4]
                        self.predicted_pos_pub.publish(predicted_pos_msg)
                else:
                    # Extract position from PointStamped message
                    self.kf.update(self.measurements)
                    self.kf.predict()
                    predicted_state = self.kf.x
                    rospy.loginfo(
                        "Measurement: x={}, y={}, z={}".format(
                            predicted_state[0], predicted_state[2], predicted_state[4]
                        )
                    )
                    # Publish predicted positions
                    predicted_pos_msg = PointStamped()
                    predicted_pos_msg.point.x = predicted_state[0]
                    predicted_pos_msg.point.y = predicted_state[2]
                    predicted_pos_msg.point.z = predicted_state[4]
                    self.predicted_pos_pub.publish(predicted_pos_msg)
            else:
                print("No drone present!!")
            rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    node = KalmanFilterNode()
    node.run()
   

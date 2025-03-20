import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import json
from cv2 import aruco
from scipy.spatial.transform import Rotation as R

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        self.id_publisher_ = self.create_publisher(String, '/aruco_id', 10)
        self.pose_publisher_ = self.create_publisher(Pose, '/aruco_pose', 10)

        with open('wyniki_kalibracji_R_3.json', 'r') as f:
            kalibracja = json.load(f)

        self.camera_matrix = np.array(kalibracja['camera_matrix'])
        self.dist_coeffs = np.array(kalibracja['distortion_coefficients'])
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.marker_length = 0.16
        self.half_length = self.marker_length / 2
        self.obj_points = np.array([
            [-self.half_length,  self.half_length, 0],
            [ self.half_length,  self.half_length, 0],
            [ self.half_length, -self.half_length, 0],
            [-self.half_length, -self.half_length, 0]
        ], dtype=np.float32)

        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            exit()

        self.timer = self.create_timer(0.1, self.detect_aruco)

    def detect_aruco(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Cannot read frame')
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                img_points = corners[i].reshape(-1, 2)

                # Obliczanie pozycji i orientacji markera
                success, rvec, tvec = cv2.solvePnP(
                    self.obj_points, img_points,
                    self.camera_matrix, self.dist_coeffs
                )

                if success:
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                    msg = String()
                    msg.data = f"ID: {ids[i][0]}"
                    self.id_publisher_.publish(msg)
                    pose = Pose()
                    pose.position.x = float(tvec[0][0])
                    pose.position.y = float(tvec[1][0])
                    pose.position.z = float(tvec[2][0])
                    r = R.from_rotvec(rvec.reshape(3))
                    quat = r.as_quat()
                    pose.orientation.x = float(quat[0])
                    pose.orientation.y = float(quat[1])
                    pose.orientation.z = float(quat[2])
                    pose.orientation.w = float(quat[3])

                    self.pose_publisher_.publish(pose)

                    self.get_logger().info(f'ID: {msg.data}')
                    self.get_logger().info(f'x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f}')

                    str_position = f"Position: x={pose.position.x:.2f}m, y={pose.position.y:.2f}m, z={pose.position.z:.2f}m"
                    cv2.putText(frame, str_position, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.6, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Aruco Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

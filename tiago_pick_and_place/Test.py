import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraImageSubscriber(Node):
    def __init__(self):
        super().__init__('camera_image_subscriber')

        # Crée une souscription à l'image RGB 
        self.rgb_subscription = self.create_subscription(
            Image,
            '/head_front_camera/image', 
            self.rgb_image_callback,
            10
        )
        # Crée une souscription à l'image de profondeur 
        self.depth_subscription = self.create_subscription(
            Image,
            '/head_front_camera/depth_image', 
            self.depth_image_callback,
            10
        )
        # Crée un publisher pour publier sur cmd_vel
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/mobile_base_controller/cmd_vel', 10)

#########################PARTIE JOINTS/BRAS####################â
        self.tiago = MoveItPy(node_name="moveit_py_pick")
        self.robot_model = self.tiago.get_robot_model()
        self.tiago_arm = self.tiago.get_planning_component("arm_torso")
        self.logger.info("MoveItPy connected!")
        print("on est connecter a moveit**************")

        self.gripper_open_joints = {"left_finger": 0.04, "right_finger": 0.04}
        self.gripper_closed_joints = {"left_finger": 0.0, "right_finger": 0.0}

        robot_initial_state = RobotState(self.robot_model)
        robot_state = RobotState(self.robot_model)
        print(robot_state,robot_initial_state)
###################################################################
        self.bridge = CvBridge()  
        self.depth_image = None  

    def get_position_in_image(self, barycenter_x):
        # Diviser l'image en 3 parties 
        image_width = 640
        left_third = image_width // 3
        right_third = image_width * 2 // 3
        # Vérifier où se trouve le barycentre
        if barycenter_x < left_third:
            return "Gauche"
        elif barycenter_x < right_third:
            return "Centre"
        else:
            return "Droite"

    def rgb_image_callback(self, msg):
        # Convertit l'image ROS en image OpenCV (RGB)
        try:
            cv_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # 'bgr8' est couramment utilisé pour les images RGB
            self.get_logger().info(f'Received RGB image with dimensions: {cv_rgb_image.shape}')
            
            # Conversion de l'image BGR en HSV
            hsv_image = cv2.cvtColor(cv_rgb_image, cv2.COLOR_BGR2HSV)
            
            # Plage pour détecter un rouge foncé
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 150])
            
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 150])
            
            # Créer un masque pour détecter la couleur rouge foncée
            mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            mask = mask1 | mask2  # Combine les deux masques

            # Afficher le masque pour vérifier la détection des zones rouges
            cv2.imshow("Red Mask", mask)
            cv2.waitKey(1)

            # Détecter les contours des objets rouges dans le masque
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Listes pour stocker les informations sur les objets rouges détectés
            barycenters = []
            distances = []
            positions = []

            # Parcourir tous les contours détectés
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Seuil ajusté pour les contours
                    # Calculer le barycentre (centre de masse) de chaque contour
                    moments = cv2.moments(contour)
                    if moments['m00'] != 0:  # Éviter la division par zéro
                        barycenter_x = int(moments['m10'] / moments['m00'])
                        barycenter_y = int(moments['m01'] / moments['m00'])

                        # Ajouter le barycentre à la liste
                        barycenters.append((barycenter_x, barycenter_y))
                        
                        # Vérifier si l'image de profondeur est disponible
                        if self.depth_image is not None:
                            # Récupérer la distance (profondeur) pour le pixel du barycentre
                            depth_value = self.depth_image[barycenter_y, barycenter_x]
                            distances.append(depth_value)
                            self.get_logger().info(f"Distance à l'objet : {depth_value} mètres")

                        # Déterminer la position du barycentre dans l'image
                        position = self.get_position_in_image(barycenter_x)
                        positions.append(position)

                        # Dessiner un cercle au barycentre pour chaque contour
                        cv2.circle(cv_rgb_image, (barycenter_x, barycenter_y), 10, (0, 255, 0), -1)  # Vert

                        # Inscrire la distance sur l'image à côté du barycentre
                        text = f"Distance: {depth_value:.2f}m"  # Affichage avec 2 décimales
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(cv_rgb_image, text, (barycenter_x + 10, barycenter_y - 10), font, 0.8, (255, 255, 255), 2)
                        
                        # Afficher la position (gauche, centre, droite) sur l'image
                        cv2.putText(cv_rgb_image, f"Position: {position}", (barycenter_x + 10, barycenter_y + 20), font, 0.8, (255, 255, 255), 2)

            # Afficher l'image avec les barycentres marqués, la distance et la position
            cv2.imshow("Detected Reds", cv_rgb_image)
            cv2.waitKey(1)

            # Log des informations pour chaque objet détecté
            for i, (barycenter, distance, position) in enumerate(zip(barycenters, distances, positions)):
                self.get_logger().info(f"Objet {i+1}: Barycentre = {barycenter}, Distance = {distance:.2f}m, Position = {position}")

            # Si au moins un barycentre a été trouvé, déplacer le robot
            if barycenters:
                self.move_robot_to_barycenter(barycenters[0])

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

    def depth_image_callback(self, msg):
        # Convertit l'image ROS en image OpenCV (profondeur)
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')  # Pour les images de profondeur en 32-bit float
            self.get_logger().info(f'Received depth image with dimensions: {self.depth_image.shape}')
        
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')

    def move_robot_to_barycenter(self, barycenter):
        # Calculer la direction du mouvement en fonction de la position du barycentre
        barycenter_x, _ = barycenter
        image_width = 640
        left_third = image_width // 3
        right_third = image_width * 2 // 3

        # Créer un message TwistStamped pour déplacer le robot
        twist_msg = TwistStamped()

        if barycenter_x < left_third:
            # Si le barycentre est à gauche, tourner à gauche (rotation)
            twist_msg.twist.angular.z = 0.5  # Rotation à gauche
            self.get_logger().info("Tourner à gauche pour centrer le barycentre.")
        elif barycenter_x > right_third:
            # Si le barycentre est à droite, tourner à droite (rotation)
            twist_msg.twist.angular.z = -0.5  # Rotation à droite
            self.get_logger().info("Tourner à droite pour centrer le barycentre.")
        else:
            # Si le barycentre est au centre, ne rien faire
            twist_msg.twist.angular.z = 0.0
            self.get_logger().info("Le barycentre est déjà au centre.")

        # Publier la commande sur /mobile_base_controller/cmd_vel
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_image_subscriber = CameraImageSubscriber()
    rclpy.spin(camera_image_subscriber)
    camera_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

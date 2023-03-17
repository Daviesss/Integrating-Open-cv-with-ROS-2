import rclpy
import cv2  
from rclpy.node import Node
from geometry_msgs.msg import Twist #A Twist messgage type to drive the robot.
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

#store the cvBridge, which is a type of the cv_bridge package.
#create a class that stores all the object and the methods we will use.
class program(Node):
    def __init__(self):
        super().__init__('face_detection')
        self.pub_camera = self.create_publisher(Image,'/camera/image_raw',100) #publish the camera image data to the topic(Note:Make sure the topic name is not the same as your robot topic , so you can use the webcam)
        #self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,10) #subscribing to the image topic of the robot camera.
        self.time = 0.05
        self.timer = self.create_timer(self.time,self.camera_callback) #subscribing to the camara feeds.
        self.image = cv2.VideoCapture(0) #reading from the webcam.
        self.face_cascade = cv2.CascadeClassifier('/home/magnum/opencv_ws/src/ros2_cv/ros2_cv/facedetection.xml') #Add the path to your open_cv face detection.xml classifier.
        print(self.image.isOpened())
        self.store_bridge = CvBridge() 

    #subscrbing to the image camera topic.
    def camera_callback(self):
        #self.get_logger().info('reading camra feeds/images: "%f"',msg.)
        #print("The image of the camera is reading")
         
        
        #reading =self.face_cascade.detectMultiScale(frame)
        while True:
            __ret,frame = self.image.read()
            reading =self.face_cascade.detectMultiScale()
            for (x,y,w,h) in reading:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.imshow("frame",frame)
            self.msg = self.store_bridge.cv2_to_imgmsg(frame,"bgr8") #This line is very important.
            self.pub_camera.publish(self.msg) #publish the message over to ros2.
            self.get_logger().info('Reading the images displayed by the camera')
            #cv2.imshow("frame_name",frame)
            if cv2.waitKey(20) and 0xFF == ord('q'):
                break

        self.image.release()
        #cv2.destroyAllWindows()

            

      
        

    # #create a method that executes the code.
    # def show(self):

    #     if not self.image.isOpened():
    #         print("This camera can't be opened")

    #     while True:
    #         __ret,frame = self.image.read()
    #         reading =self.face_cascade.detectMultiScale(frame)
    #         for (x,y,w,h) in reading:
    #             cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
    #         cv2.imshow("frame",frame)
    #         self.msg = self.store_bridge.cv2_to_imgmsg(frame,"bgr8") #This line is very important.
    #         self.pub_camera.publish(self.msg) #publish the message over to ros2.
    #         #cv2.imshow("frame_name",frame)
            

    #         #if statement that tells what key to press to stop the node.
    #         if cv2.waitKey(20) and 0xFF == ord('q'):
    #             break

    #     self.image.release()
    #     #cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    store = program() #passing in an object.
    rclpy.spin(store)


if __name__ == '__main__':
    main()
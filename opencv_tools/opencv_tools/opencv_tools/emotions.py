import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from example_interfaces.msg import Float64MultiArray
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from deepface import DeepFace

  
class Emotions(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('emotions_')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
    self.subscription # prevent unused variable warning
    self.emocion = []
    self.publisher = self.create_publisher(Float64MultiArray, 'emotions_stats',10)

    self.timer = self.create_timer(0.1, self.timer_callback)


       
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Detecting emotions')
  
    # Convert ROS Image message to OpenCV image
    img_src = self.br.imgmsg_to_cv2(data)
    img_src = cv2.cvtColor(img_src, cv2.COLOR_BGR2RGB)


    img_blur = cv2.GaussianBlur(img_src, (5, 5), 0)
    

    self.detector(img_blur)
    emociones = self.deep(img_src)

    self.floatArray(emociones)


    #self.deep(img_blur)

    # Display image
    #cv2.imshow("camera", img_rst)

    cv2.waitKey(1)


  def timer_callback(self):

    msg2 = Float64MultiArray()
    msg2.data = self.emocion
    self.publisher.publish(msg2)
    

    
  def detector(self,image):

    faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)


    faces = faceCascade.detectMultiScale(gray,1.1,4)

    for(x, y, w, h) in faces:

        cv2.rectangle(image, (x,y), (x+w, y+h), (0,255, 0), 2)
        face = image[y:y + h, x:x + w]

        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        
        cv2.imshow('Emotions',img)


    

  def deep(self,image):

    faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(gray,1.1,4)

    for i, (x, y, w, h) in enumerate(faces):
        self.get_logger().info(str(i))

        cv2.rectangle(image, (x,y), (x+w, y+h), (0,255, 0), 2)
        face = image[y:y + h, x:x + w]

        result = DeepFace.analyze(face, actions=['emotion'], enforce_detection=False)


        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, result[0]['dominant_emotion'], (x,y), font, 1, (0,0,225), 2, cv2.LINE_4)

        coloured_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        if i == 0:
          emotions = [result[0]['emotion']]
        else:
          # El problema está en cuando en el frame no se detecta una cara
          # dado que no crea la lista pero la retorna
          emotions.append(result[0]['emotion'])


        cv2.imshow("Emotions",coloured_img)
    return emotions
    
  def floatArray(self,emociones):
    self.emocion = []  # ULTIMO CAMBIOOOOO
    emotions_list = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']
    for k, emotion in enumerate(emociones):

      self.emocion.extend(list(map(emotion.get, emotions_list)))

def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  emotions = Emotions()
   
  # Spin the node so the callback function is called.
  rclpy.spin(emotions)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  emotions.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()

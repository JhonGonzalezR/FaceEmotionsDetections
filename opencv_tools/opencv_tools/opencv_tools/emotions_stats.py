#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from example_interfaces.msg import Float64MultiArray


class EmotionStats(Node):
    def __init__(self):
        super().__init__("emotion_stats")
        
        self.get_logger().info("Looking for stats")
        self.suscriber = self.create_subscription(Float64MultiArray,'emotions_stats',self.stats_callback,10)
        self.emotions_list = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']
        plt.figure()
    def stats_callback(self, data):
        
        self.get_logger().info('Showing stats')
        emotions = data.data
        person_emotion = []
        num_subplots = int(len(emotions)/7)
        for i in range(0,len(emotions),7):
            person_emotion.append(emotions[i:i+7])

        
        
        for i in range(len(person_emotion)):

            plt.subplot(1,len(person_emotion),i+1)

            plt.bar(self.emotions_list, person_emotion[i], color=['red', 'green', 'fuchsia', 'yellow', 'blue', 'pink', 'gray'])
            plt.title(f"Distribución de emociones - Cara {i+1}")
            plt.xlabel("Emociones")
            plt.ylabel("Porcentaje")
            plt.ylim([0, 100])
            plt.xticks(rotation=45)
            
        plt.tight_layout()
        #plt.show()
        plt.pause(0.01)  # Ajusta el tiempo según sea necesario

        plt.clf()

def main(args=None):
    rclpy.init(args=args)

    node = EmotionStats()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__== "__main__":
    main()
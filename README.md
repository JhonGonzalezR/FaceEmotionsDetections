# FaceEmotionsDetections
This repository contains a ROS2 package which many features for showing and monitoring face emotions and apply filters in a real time video.

In order to use this package, follow this steps in terminal.

1. `cd ros2_ws/src`
2. Clone this repository. 
3. Colcon your ros2_ws. `colcon build`
4. Source your workspace `source install/setup.bash`


# Filter implementing

3 filters are implemented for image management. To use them, it is essential to run the publishing node of the frames captured by the video camera.

`ros2 run opencv_tools img_publisher` - Frames publisher.

### Lowpass filter

This filter has the effect of softening the image and removing fine details or rapid changes in pixel intensity. For this, the OpenCv (Cv2) library and the cv2.filter2D() method are used.
The low pass filter is useful in various applications, such as noise reduction, smoothing images to improve visual appearance, or preparing images for certain processing algorithms that work better with softer data.

`ros2 run opencv_tools lowpass` - lowpass filter.

![image](https://github.com/JhonGonzalezR/FaceEmotionsDetections/assets/98565218/fa859022-66c0-40fc-8766-120e9a1c80eb)


### Highpass filter

This filter has the effect of representing rapid changes in pixel intensity, such as edges and fine details. For this, the OpenCv (Cv2) library and the cv2.filter2D() method are used.
The main function of a high-pass filter in an image is to enhance details and highlight edges. By applying a high-pass filter, you can attenuate parts of the image that contain low-pass variations.
frequency, such as uniform areas or smooth intensity transitions, and highlight parts that have abrupt changes.

`ros2 run opencv_tools highpass` - highpass filter.

![image](https://github.com/JhonGonzalezR/FaceEmotionsDetections/assets/98565218/d38671d1-a62d-4d66-9a78-d12ca5a69db8)


### Horizontal lines filter

This filter chooses to detect the horizontal lines present in each frame of the image  

`ros2 run opencv_tools custom_filter` - custom filter.

![image](https://github.com/JhonGonzalezR/FaceEmotionsDetections/assets/98565218/770f7d67-aafd-4324-b80f-933ebadfe9be)

### Filters in simultaneous

A launch type file was created to launch the nodes of each filter and the image publisher to display each filter simultaneously.

`ros2 launch opencv_tools filtros.launch.py` - Launch file.

![image](https://github.com/JhonGonzalezR/FaceEmotionsDetections/assets/98565218/1dfaf2b6-e4f0-47bf-ae2f-418fe1840850)

# Emotions detections

In order to visualize face emotions in real time and their analytics for each face.

`ros2 launch opencv_tools detector_emociones.launch.py` - Launch file.

![image](https://github.com/JhonGonzalezR/FaceEmotionsDetections/assets/98565218/54bc6572-092e-440b-b154-c85ee87b85f7)

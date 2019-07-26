# charuco_detector

Detector of [ChArUco patterns](https://docs.opencv.org/master/df/d4a/tutorial_charuco_detection.html) using the [cv::aruco::CharucoBoard](https://docs.opencv.org/master/d9/d6d/tutorial_table_of_content_aruco.html).

ChArUco patterns from A0 to A4 are available in the [boards folder](boards) with their creation parameters in the associated launch file.

The main advantage of using ChArUco patterns vs the traditional chessboards is its ability to reliably estimate the board 6 DoF origin even if some of the squares / markers are occluded with parts. This is useful for calibrating sensors and also for marking coordinate systems for robotics applications.


## Usage

- Choose and print a pattern from the [boards folder](boards) according to your camera work area or create a new one using  [ROS charuco_creator](https://github.com/ItzMeJP/charuco_creator) or the [OpenCV ChArUco board creator](https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/samples/create_board_charuco.cpp).

- Update the [launch/charuco_detector.launch](launch/charuco_detector.launch) according to the pattern you are going to use or pass the parameters to the launch file in the command line.

- Even when printing in real size, there might be a small scaling applied by the printer, which can be mitigated by measuring all the black and white scares in the widest axis and then updating the size of the squares and markers given to the ChArUco detector with:
  ```
  updated_squares_sides_size_in_meters = measure_of_all_squares_in_widest_direction / number_of_squares_in_y
  updated_markers_sides_size_in_meters = (updated_squares_sides_size_in_meters / squares_sides_size_in_meters) * markers_sides_size_in_meters
  ```

- Example for the A3 (10x14) pattern in the [boards folder](boards):
  ```
  roslaunch charuco_detector charuco_detector.launch image_topic:=/camera/image_raw camera_info_topic:=/camera/camera_info squares_sides_size_in_meters:=0.028 markers_sides_size_in_meters:=0.0168
  ```

# charuco_detector

Detector of [ChArUco patterns](https://docs.opencv.org/master/df/d4a/tutorial_charuco_detection.html) using the [cv::aruco::CharucoBoard](https://docs.opencv.org/master/d9/d6d/tutorial_table_of_content_aruco.html).

The main advantage of using ChArUco patterns vs the traditional chessboards is their ability to reliably estimate the board 6 DoF pose even if some of the squares / markers are occluded or outside of the image field of view. This is useful for calibrating cameras and also for calculating coordinate systems for robotics applications.


## Usage

- Choose and print a pattern from the [boards/vector_format](boards/vector_format) folder according to your camera work area or [create a new one](https://calib.io/pages/camera-calibration-pattern-generator).
- Update the [launch/charuco_detector.launch](launch/charuco_detector.launch) parameters according to the pattern you are going to use and your camera driver topics (image_raw and camera_info).
- Run your camera driver
- Run charuco_detector.launch (preconfigured for the A3 (10x14) pattern in the [boards/vector_format](boards/vector_format))
  - Example for the A3 (10x14) pattern in the [boards/vector_format](boards/vector_format) folder:
    ```
    roslaunch charuco_detector charuco_detector.launch image_topic:=/camera/image_raw camera_info_topic:=/camera/camera_info
    ```

- The 6 DoF pose will be published to TF (camera_frame_id -> charuco) and topic `$(arg image_topic)_charuco_pose`
- The results of the detection of the markers and ChArUco origin will be drawn into an image that is published to topic `$(arg image_topic)_charuco_detection`


## Notes

Even when printing in real size, there might be a small scaling applied by the printer, which can be mitigated by measuring all the black and white scares in the widest axis and then updating the size of the squares and markers given to the ChArUco detector with:
  ```
  updated_squares_sides_size_in_meters = measure_of_all_squares_in_widest_direction / number_of_squares_in_y
  updated_markers_sides_size_in_meters = (updated_squares_sides_size_in_meters / squares_sides_size_in_meters) * markers_sides_size_in_meters
  ```

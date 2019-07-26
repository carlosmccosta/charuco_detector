# Patterns

Folder with [ChArUco](https://docs.opencv.org/master/df/d4a/tutorial_charuco_detection.html) patterns from A0 to A4.


## ROS

Each pattern has a launch file for generating a high resolution .png that can then be converted to a real size .pdf using gimp:
- Open .png
- Image -> Auto crop
- File -> Print
- Print to file
- Page setup -> Change to desired paper size
- Image settings -> Change width to [x_squares * squares_size * 1000] and height to [y_squares * squares_size * 1000]
- Print
- Confirm .pdf measurements with InkScape
- Print in real size


## OpenCV

Alternatively to the [charuco_creator](https://github.com/ItzMeJP/charuco_creator) ROS package, you can also use the opencv [create_board_charuco](https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/samples/create_board_charuco.cpp).

Parameters:
```
-w: Number of squares in X direction
-h: Number of squares in Y direction
-sl: Square side length
-ml: Marker side length
-d: dictionary
```

Dictionary:
```
DICT_4X4_50=0,
DICT_4X4_100=1,
DICT_4X4_250=2,
DICT_4X4_1000=3,
DICT_5X5_50=4,
DICT_5X5_100=5,
DICT_5X5_250=6,
DICT_5X5_1000=7,
DICT_6X6_50=8,
DICT_6X6_100=9,
DICT_6X6_250=10,
DICT_6X6_1000=11,
DICT_7X7_50=12,
DICT_7X7_100=13,
DICT_7X7_250=14,
DICT_7X7_1000=15,
DICT_ARUCO_ORIGINAL=16
```

Examples:

- A4 5x7 (200x280 mm)
```
aruco-example-create_board_charuco.exe chboard-a4.png -w=5 -h=7 -sl=200 -ml=120 -d=10
aruco-example-detect_board_charuco.exe -w=5 -h=7 -sl=0.04 -ml=0.024 -d=10
```

- A3 7x10 (280x400 mm)
```
aruco-example-create_board_charuco.exe chboard-a3.png -w=7 -h=10 -sl=200 -ml=120 -d=10
aruco-example-detect_board_charuco.exe -w=7 -h=10 -sl=0.04 -ml=0.024 -d=10
```

- A3 10x14 (280x392 mm)
```
aruco-example-create_board_charuco.exe chboard-a3.png -w=10 -h=14 -sl=200 -ml=120 -d=10
aruco-example-detect_board_charuco.exe -w=10 -h=14 -sl=0.028 -ml=0.0168 -d=10
```

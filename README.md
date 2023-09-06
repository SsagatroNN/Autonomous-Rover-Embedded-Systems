# Autonomous-Rover-Embedded-Systems
A repository containing the source code of the Embedded systems used in the Second Year Imperial College London Project.\


## FPGA Image Processing

The image processing unit found in the Distance Quartus project was used to measure the distance in pixels from the center of the image to the first set of white pixels on the left, right, and center of the image. The data is then encoded and sent over SPI to ESP32. The camera used was the D8M-Camera module made for the DE-10 Lite with a focal length of 70cm. This was not enough to allow the rover to track the wall correctly, and hence a lens was added to increase the focal length. 

## ESP32

The ESP32 had a few tasks running in order to ensure the rover was functioning. It had 3 main threads working. The first thread was pinned to a core and was in charge of maintaining self-balance. The second core had a single thread in charge of collecting data from sensors, including data from the camera module and another thread that ran an HTTP Server used to relay its coordinates to the React App.

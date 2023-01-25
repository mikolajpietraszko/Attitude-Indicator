# <h1>Attitude-Indicator</h1>
A small project I made during my university time linking Arduino with Matlab to display IMU data on an Artificial Horizon.
It utilizes MPU-6050 6 axis IMU, DHT-11 temperature and humidity sensor and an LCD1602 Display.

Tha Artificial Horizon is a feature in Matlab that visualizes the pitch and roll angle of the aircraft.

![arthorizon](https://user-images.githubusercontent.com/97880512/214692241-f8521916-23d5-4b35-8732-a08638fe4947.gif)

But first before you visualize it, you will need to gather some data with an Inertial Measurement Unit (IMU). You can use MPU-6050 that's included in many Arduino kits.

Here is a wiring diagram of the setup

![wiring diagram](https://user-images.githubusercontent.com/97880512/214688147-baa270d3-1f22-4c25-9627-c0d07e5ebf69.png)

The IMU obtains pitch, roll and yaw data which can be used to calculate the angles and put through a Kalman filter (KalmanFilter.h library). Filtered data is serialized and then sent to Matlab.
The artificial horizon feature is available in Matlab Aerospace Toolbox so you will need it to run the project.

Project also contains a warnings.m function,which is called to display alerts in the command window when attitude indicator parameters exceed the set recommended value range.

To run theArduino code u will need 2 libraries made by Korneliusz Jarzębski which can be found here:
1.	https://github.com/jarzebski/Arduino-MPU6050
2.	https://github.com/jarzebski/Arduino-KalmanFilter

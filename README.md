ROS MPU6050 Node
================

Publishes IMU DMP sensor data from an MPU6050 connected to an I2C bus on a Raspberry Pi.

Installation
------------

First [enable i2c on your RPi](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c)

Install [I2Cdevlib](https://github.com/jrowberg/i2cdevlib):

    sudo mkdir -p /usr/share/arduino/libraries
    cd /usr/share/arduino/libraries
    sudo git clone https://github.com/chrisspen/i2cdevlib.git

*Note the fork should be used, since jrowberg's i2cdevlib is way ahead and use too much Arduino specific code to compile on Raspberry Pi*

Then install [Bcm2835](http://www.airspayce.com/mikem/bcm2835/index.html):

    cd /tmp
    wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.68.tar.gz
    tar zxvf bcm2835-1.68.tar.gz
    cd bcm2835-1.68
    ./configure
    make
    make check
    sudo make install
    
Then clone the project into your ROS workspace via:

    git clone https://github.com/Zarfab/ros_mpu6050_node.git
    
And then compile it:

    catkin_make --pkg ros_mpu6050_node

Usage
-----

Since the Raspbian kernel and BCM2835 driver restrict I2C access to only the root user, you must launch the node as root like:

    sudo bash -c "source /your/ros/path/setup.bash; roslaunch ros_mpu6050_node mpu6050.launch"

Assuming the device is properly wired, it should report "DMP ready!". Now you should be able to see the IMU stream with:

    rostopic echo /imu/data

Calibration
-----------

A handy tool based on the [Arduino IMU_zero sketch](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero) is provided to calculate the offsets (ax, ay, az, gx, gy, gz) to put as parameters in launch/mpu6050.launch.

If an MPU6050 
- is an ideal member of its tribe, 
- is properly warmed up, 
- is at rest in a neutral position, 
- is in a location where the pull of gravity is exactly 1g, and 
- has been loaded with the best possible offsets, 

then it will report 0 for all accelerations and displacements, except for 
Z acceleration, for which it will report 16384 (that is, 2^14).  Your device 
probably won't do quite this well, but good offsets will all get the baseline 
outputs close to these target values.

Put the MPU6050 on a flat and horizontal surface, and leave it operating for 
5-10 minutes so its temperature gets stabilized.

To compile on a Raspberry Pi (tested on Rpi 3B with raspios buster - 2020-08-20)

    cd utils/
    PATH_I2CDEVLIB=/usr/share/arduino/libraries/i2cdevlib/
    sudo gcc -o IMU_zero IMU_zero.cpp \
         -I ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev/I2Cdev.cpp \
         -I ${PATH_I2CDEVLIB}Arduino/MPU6050/ ${PATH_I2CDEVLIB}Arduino/MPU6050/MPU6050.cpp -l bcm2835 -l m

Then launch with root privileges :

    $ sudo ./IMU_zero

A "----- done -----" line will indicate that it has done its best.
With the current accuracy-related constants (NFast = 1000, NSlow = 10000), it will take 
a few minutes to get there.

  Along the way, it will generate a dozen or so lines of output, showing that for each 
of the 6 desired offsets, it is 
- first, trying to find two estimates, one too low and one too high, and
- then, closing in until the bracket can't be made smaller.

The line just above the "done" line will look something like

    [567,567] --> [-1,2]  [-2223,-2223] --> [0,1] [1131,1132] --> [16374,16404] [155,156] --> [-1,1]  [-25,-24] --> [0,3] [5,6] --> [0,4]

As will have been shown in interspersed header lines, the six groups making up this
line describe the optimum offsets for the X acceleration, Y acceleration, Z acceleration,
X gyro, Y gyro, and Z gyro, respectively.  In the sample shown just above, the trial showed
that +567 was the best offset for the X acceleration, -2223 was best for Y acceleration, 
and so on.

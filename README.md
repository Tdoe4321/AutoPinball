# AutoPinball
A custom pinball machine that plays itself!

[Checkout the full Instructables post!](https://www.instructables.com/id/Arduino-Pinball-Machine-That-Plays-Itself)

Check out the video here:  
[![AutoPinball Video](http://img.youtube.com/vi/dy7oVSNtaRk/0.jpg)](https://www.youtube.com/watch?v=dy7oVSNtaRk "AutoPinball")

Straight up demonstration of the autonomy here:  
[![AutoPinball Demonstration](http://img.youtube.com/vi/hW7z0yfWGfI/0.jpg)](https://www.youtube.com/watch?v=hW7z0yfWGfI "AutoPinball")

### Dependencies
This repo has the following dependencies that must be installed.
 * [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
 * [rosserial_arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
 * [OpenCV C++](https://opencv.org/)
 * [Tkinter](https://docs.python.org/3/library/tkinter.html)
 * [APScheduler](https://apscheduler.readthedocs.io/en/stable/)
 
To install ROS and rosserial_arduino, follow the links above and install as normal, for everything else, do the following:

```bash
sudo apt install libopencv-dev python3-opencv # OpenCV
sudo apt install python-tk # Tkinter
pip install APScheduler  # APScheduler
```
 
### Build
To build the system do the following:
```bash
cd /path/to/AutoPinball
mkdir build
cd build
cmake ..
make
source devel/setup.bash
```

After you build it, you can put the `source` command into your `~/.bashrc` to have it automatically source the directory whenever you start up a new bash shell. Just make sure that you give the full path, ex:  
`/home/<my-username>/AutoPinball/build/devel/setup.bash`

### RUN IT!
There's a few different launch files that you can use depending on what you want to do.
If you want to run the whole thing, run this one from the `launch` directory:  
```bash
roslaunch automatic_pinball_c.launch
```

# AutoPinball
A custom pinball machine that plays itself!

Check out the video here:  
[![AutoPinball Video](http://img.youtube.com/vi/dy7oVSNtaRk/0.jpg)](https://www.youtube.com/watch?v=dy7oVSNtaRk "AutoPinball")

### Dependencies
 * TODO 
 
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

After you build it, you can put the `source` command into your `~/.bashrc` to have it automatically source the directory whenever you start up a new bash shell. Just make sure that you give the full path, ex: `/home/<my-username>/AutoPinball/build/devel/setup.bash`

### RUN IT!
There's a few different launch files that you can use depending on what you want to do.
If you want to run the whole thing, run this one from the `launch` directory:  
```bash
roslaunch automatic_pinball_c.launch
```

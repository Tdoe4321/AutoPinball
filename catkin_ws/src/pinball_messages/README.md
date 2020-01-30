<!DOCTYPE html>
<pre>
<h1> Custom Messages and Services</h1> 
<p>Click <a href="http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv">here</a> for more information.</p>

<h2> For any packages that use custom msgs or srvs:</h2>
1. Go to the package.xml and uncomment the following:
  &lt;build_depend&gt;message_generation&lt;/build_depend&gt;
  &lt;exec_depend&gt;message_runtime&lt;/exec_depend&gt;
2. Go to the CMakeLists.txt in the section 'find_package(catkin REQUIRED COMPONENTS)', add 'message_generation'
3. In the CMakeLists.txt under 'generate_messages' add any dependencies under DEPENDENCIES


<h2> Using msgs or srvs from other packages</h2>
 In the package.xml, have the following:
&lt;build_depend&gt; package_name &lt;/build_depend&gt;
&lt;run_depend&gt; package_name &lt;/run_depend&gt;

<h2> Adding Custom Messages</h2>
1. Add the message.msg file to the msg directory
2. In the CMakeLists.txt file, go to 'add_message_files' and add MessageFileName.msg under FILES
3. Go to catkin_ws directory and execute the command: 'catkin_make'
4. Execute the command: source devel/setup.bash

<h2> Using Custom messages</h2>
<a href="http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29">Python</a>
<a href="http://wiki.ros.org/th/ROS/Tutorials/DefiningCustomMessages">C++</a>

<h3> Importing and declaring custom messages in Python</h2> 
from submarine_msgs_srvs.msg import MessageName
msg = MessageName()

A full example can be found in the scripts directory of this package

<h2> Adding Custom srvs </h2>
1. Add service.srv file to the srv directory
2. In the CMakeLists.txt, under 'add_service_files' add ServiceFileName.srv under FILES
3. Create a service node and a client node to run the service.
Learn about service nodes <a href="http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29">here</a>


# Testing the Examples
1. In the catkin_ws directory, catkin_make
2. source devel/setup.bash
In every terminal that you open in bash, source the setup.bash file.
The following assumes everything is done from the catkin_ws directory.

<h2> Testing the custom message example </h2>
1. Execute command: roscore
2. In another terminal execute command python src/submarine_msgs_srvs/scripts/testPublisher.py
3. In another terminal execute command python src/submarine_msgs_srvs/scripts/testSubscriber.py
<h2> Testing the custom service example </h2>

1. Execute command: roscore
2. In another terminal execute command: python src/submarine_msgs_srvs/scripts/ReadBoxServer.py
3. In another terminal execute command: rosservice call /readBox [0,1,2,3,4,5,6] "class_id"

</pre>
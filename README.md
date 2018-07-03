# NatNetROS

## Dependencies

* `NatNetLinux` - from: https://github.com/rocketman768/NatNetLinux

## Installation

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/tatsuya-s/natnet_ros
    $ cd ~/catkin_ws/
    $ catkin build

## Setting

### Motive

- Broadcast Frame Data set to true
- Local Interface set to the ip address of your PC (It is server address)

## Usage

    $ roslaunch natnet_ros test.launch local_address:=192.168.1.100 server_address:=192.168.1.101

Or write a launch file
```XML:run.launch
<launch>

  <node pkg="natnet_ros" type="natnet_node" name="natnet_node" output="screen">
    <rosparam subst_value="true">
      local_address: "192.168.1.100"
      server_address: "192.168.1.101"
    </rosparam>
  </node>

</launch>
```

And 

    $ roslaunch natnet_ros run.launch
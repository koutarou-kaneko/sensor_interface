

### V4L2 loopback mode

#### load the loopback kernel module
```
$ sudo modprobe v4l2loopback
```

#### start video streaming

- Nvidia Jetson TX2 with the default carrier board
```
$ roslaunch lepton_thermal_sensor leption_thermal_v4l2loopback.launch spi_id:=3.0 video_id:=1
```
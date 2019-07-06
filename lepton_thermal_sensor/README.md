

### V4L2 loopback mode

#### load the loopback kernel module
```
$ sudo modprobe v4l2loopback
```

#### start video streaming

- Nvidia Jetson TX2 with the default carrier board
```
$ roslaunch lepton_thermal_sensor v4l2loopback.launch spi_id:=3.0 i2c_id:=0 video_id:=1
```
**note**: connect SPI to `/dev/spidev3.0` (pin 19, 21, 23, 24), I2C to `/dev/i2c-0` (pin 27, 28). You can also set as `i2c_id:=1` by connecting to `/dev/i2c-1` (pin 3, 5).

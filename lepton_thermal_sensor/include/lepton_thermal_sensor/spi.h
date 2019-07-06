#pragma once

#include <string>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

namespace Lepton
{
  namespace SPI
  {
    bool openPort(std::string device);
    bool closePort(void);
    bool read(uint8_t* buffer_ptr, uint32_t buffer_size);
    int getHandler();
  };
};

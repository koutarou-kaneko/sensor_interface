#pragma once

#include <string>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

namespace Lepton
{
  namespace SPI
  {
    int openPort(char *device);
    int closePort(void);
    int getSpiHander();
  };
};

#include "lepton_thermal_sensor/spi.h"

namespace
{
  int spi_cs_fd = -1;
  uint32_t spi_speed;
  uint8_t spi_mode;
  uint8_t spi_bitsPerWord;
};

namespace Lepton
{
  namespace SPI
  {
    bool openPort (std::string spi_device, uint32_t speed)
    {
      int status_value = -1;

      //----- SET SPI MODE -----
      //SPI_MODE_0 (0,0)  CPOL=0 (Clock Idle low level), CPHA=0 (SDO transmit/change edge active to idle)
      //SPI_MODE_1 (0,1)  CPOL=0 (Clock Idle low level), CPHA=1 (SDO transmit/change edge idle to active)
      //SPI_MODE_2 (1,0)  CPOL=1 (Clock Idle high level), CPHA=0 (SDO transmit/change edge active to idle)
      //SPI_MODE_3 (1,1)  CPOL=1 (Clock Idle high level), CPHA=1 (SDO transmit/change edge idle to active)
      spi_mode = SPI_MODE_3;

      //----- SET BITS PER WORD -----
      spi_bitsPerWord = 8;

      //----- SET SPI BUS SPEED -----
      spi_speed = speed;

      spi_cs_fd = open(spi_device.c_str(), O_RDWR);

      if (spi_cs_fd < 0)
        {
          std::cout << "Error - Could not open SPI device" << std::endl;
          return false;
        }

      status_value = ioctl(spi_cs_fd, SPI_IOC_WR_MODE, &spi_mode);
      if(status_value < 0)
        {
          std::cout << "Could not set SPIMode (WR)...ioctl fail" << std::endl;
          return false;
        }

      status_value = ioctl(spi_cs_fd, SPI_IOC_RD_MODE, &spi_mode);
      if(status_value < 0)
        {
          std::cout << "Could not set SPIMode (RD)...ioctl fail" << std::endl;
          return false;
        }

      status_value = ioctl(spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
      if(status_value < 0)
        {
          std::cout << "Could not set SPI bitsPerWord (WR)...ioctl fail" << std::endl;
          return false;
        }

      status_value = ioctl(spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
      if(status_value < 0)
        {
          std::cout <<  "Could not set SPI bitsPerWord(RD)...ioctl fail" << std::endl;
          return false;
        }

      status_value = ioctl(spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
      if(status_value < 0)
        {
          std::cout << "Could not set SPI speed (WR)...ioctl fail" << std::endl;
          return false;
        }

      status_value = ioctl(spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
      if(status_value < 0)
        {
          std::cout << "Could not set SPI speed (RD)...ioctl fail" << std::endl;
          return false;
        }

      return true;
    }

    bool closePort(void)
    {
      int status_value = -1;

      status_value = close(spi_cs_fd);
      if(status_value < 0)
        {
          std::cout << "Error - Could not close SPI device" << std::endl;
          return false;
        }

      return true;
    }

    bool read(uint8_t* buffer_ptr, uint32_t buffer_size)
    {
      int ret = ::read(spi_cs_fd, buffer_ptr, buffer_size);
      if (ret < 0) return false;

      return true;
    }
  };
};

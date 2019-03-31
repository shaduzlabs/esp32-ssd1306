/*
        ##########    Copyright (C) 2019 Vincenzo Pacella
        ##      ##    Distributed under MIT license, see file LICENSE
        ##      ##    or <http://opensource.org/licenses/MIT>
        ##      ##
##########      ############################################################# shaduzlabs.com #####*/

#pragma once

// -------------------------------------------------------------------------------------------------

#include <rastr/CanvasMonochrome.hpp>

#include <driver/spi_master.h>

#include <cstdint>
#include <vector>

// -------------------------------------------------------------------------------------------------

namespace sl
{
namespace esp32
{

// -------------------------------------------------------------------------------------------------

class SSD1306 : public sl::rastr::CanvasMonochrome<128,64>
{
public:

  struct PinConfig
  {
      gpio_num_t clock;
      gpio_num_t chipSelect;
      gpio_num_t data;
      gpio_num_t dc;
      gpio_num_t reset;
      spi_host_device_t spiHost;
      int dmaChannel;
  };

  SSD1306(PinConfig);
  ~SSD1306();

  void initialize();

  void display();

  void invertDisplay(bool);
  void dim(bool);

  void startScrollRight(uint8_t, uint8_t);
  void startScrollLeft(uint8_t, uint8_t);
  void startScrollDiagRight(uint8_t, uint8_t);
  void startScrollDiagLeft(uint8_t, uint8_t);
  void stopScroll();

private:
  enum class Command : uint8_t
  {
    setContrast = 0x81,

    displayAllOnResume = 0xA4,
    displayAllOn = 0xA5,
    displayNormal = 0xA6,
    displayInvert = 0xA7,
    displayOff = 0xAE,
    displayOn = 0xAF,

    setDisplayOffset = 0xD3,
    setCOMPins = 0xDA,
    setDisplayClockDivider = 0xD5,
    setPreChargePeriod = 0xD9,
    setVCOMDetect = 0xDB,

    setMultiplexRatio = 0xA8,

    setLowColumn = 0x00,
    setHighColumn = 0x10,

    setStartLine = 0x40,

    setMemoryMode = 0x20,
    setColumnAddress = 0x21,
    setPageAddress = 0x22,

    setSegmentRemap = 0xA0,

    setCOMOutputScanDirectionUp = 0xC0,
    setCOMOutputScanDirectionDown = 0xC8,

    setChargePump = 0x8D,

    enableScroll = 0x2F,
    disableScroll = 0x2E,
    setVerticalScrollArea = 0xA3,
    rightHorizontalScroll = 0x26,
    leftHorizontalScroll = 0x27,
    verticalAndRightHorizontalScroll = 0x29,
    verticalAndLeftHorizontalScroll = 0x2A,

    nop = 0xE3,

  };

  const PinConfig m_pinConfig;

  void command(uint8_t);
  void command(std::vector<uint8_t>);

  // Configuration
  static constexpr bool s_chargePumpEnabled = true;

  spi_device_handle_t m_spiDevice{nullptr};
};

// -------------------------------------------------------------------------------------------------

} // namespace esp32
} // namsepace sl

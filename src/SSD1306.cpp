/*
        ##########    Copyright (C) 2018 Vincenzo Pacella
        ##      ##    Distributed under MIT license, see file LICENSE
        ##      ##    or <http://opensource.org/licenses/MIT>
        ##      ##
##########      ############################################################# shaduzlabs.com #####*/

#include "SSD1306.h"

#include "esp_log.h"

#include <driver/gpio.h>
#include <freertos/task.h>

//  ------------------------------------------------------------------------------------------------

#define M_CMD(cmd) static_cast<uint8_t>(Command::cmd)

// -------------------------------------------------------------------------------------------------

namespace sl
{
namespace esp32
{

//  ------------------------------------------------------------------------------------------------

SSD1306::SSD1306(PinConfig pinConfig)
  : m_pinConfig(std::move(pinConfig))
{
  initialize();
}

//  ------------------------------------------------------------------------------------------------

SSD1306::~SSD1306()
{
  spi_bus_remove_device(m_spiDevice);
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::initialize()
{
  // Initialize GPIOs that are not directly handled by SPI
  gpio_set_direction(m_pinConfig.dc, GPIO_MODE_OUTPUT);
  gpio_set_direction(m_pinConfig.reset, GPIO_MODE_OUTPUT);

  // Reset the display
  gpio_set_level(m_pinConfig.reset, 0);
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(m_pinConfig.reset, 1);
  vTaskDelay(100 / portTICK_RATE_MS);

  spi_bus_config_t bus_config{};
  bus_config.sclk_io_num = m_pinConfig.clock; // CLK
  bus_config.mosi_io_num = m_pinConfig.data;  // MOSI
  bus_config.miso_io_num = -1;                // MISO
  bus_config.quadwp_io_num = -1;              // Not used
  bus_config.quadhd_io_num = -1;              // Not used
  bus_config.max_transfer_sz = 1024;

  ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));
  ESP_LOGI("SSD1306", "SPI bus initialized");

  spi_device_interface_config_t dev_config{};
  dev_config.mode = 0;
  dev_config.clock_speed_hz = 20000000;
  dev_config.spics_io_num = m_pinConfig.chipSelect;
  dev_config.queue_size = 7;
//  dev_config.pre_cb = preTransferCallbackSPI;
  ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &m_spiDevice));

  // Initialize the display
  command({M_CMD(displayOff)});
  command({M_CMD(setDisplayClockDivider), 0x80});

  static const uint8_t multiplexRatio = static_cast<uint8_t>(static_cast<int>(height() - 1));
  command({M_CMD(setMultiplexRatio), multiplexRatio});

  command({M_CMD(setDisplayOffset), 0});
  command({M_CMD(setStartLine) | 0x00});

  command({M_CMD(setChargePump), s_chargePumpEnabled ? 0x14 : 0x10});

  command({M_CMD(setMemoryMode), 0x00});

  command({M_CMD(setSegmentRemap) | 0x01});

  command({M_CMD(setCOMOutputScanDirectionDown)});

  uint8_t COMPins = 0x02;
  uint8_t contrast = 0x8F;
  if (height() == 64)
  {
    COMPins = 0x12;
    contrast = s_chargePumpEnabled ? 0xCF : 0x9F;
  }
  else if (height() == 16 && width() == 96)
  {
    COMPins = 0x2;
    contrast = s_chargePumpEnabled ? 0xAF : 0x10;
  }
  command({M_CMD(setCOMPins), COMPins});
  command({M_CMD(setContrast), contrast});

  command({M_CMD(setPreChargePeriod), s_chargePumpEnabled ? static_cast<uint8_t>(0xF1) : 0x22});

  command({M_CMD(setVCOMDetect), 0x40});
  command({M_CMD(displayAllOnResume)});
  command({M_CMD(displayNormal)});

  command({M_CMD(setColumnAddress), 0, 127});
  command({M_CMD(setPageAddress), 0, 7});

  command({M_CMD(disableScroll)});
  command({M_CMD(displayOn)});

  ESP_LOGI("SSD1306", "SSD1306 initialized");
}

//--------------------------------------------------------------------------------------------------

void SSD1306::display()
{
  if (!m_spiDevice)
  {
    return;
  }
  static const uint8_t numColumns = static_cast<uint8_t>(static_cast<int>(width()) - 1);
  static const uint8_t numPages = static_cast<uint8_t>(static_cast<int>(height() / 8) - 1);
  static std::vector<uint8_t> columnAddress{M_CMD(setColumnAddress), 0, numColumns};
  static std::vector<uint8_t> pageAddress{M_CMD(setPageAddress), 0, numPages};

  resetInvalidRegion();

  command(columnAddress);
  command(pageAddress);

  gpio_set_level(m_pinConfig.dc, 1);

  spi_transaction_t transaction{};
  memset(&transaction, 0, sizeof(transaction));
  transaction.length = bufferSize() * 8; // Transaction length in bits
  transaction.tx_buffer = data();        // Data
  transaction.rx_buffer = nullptr;

  ESP_ERROR_CHECK(spi_device_polling_transmit(m_spiDevice, &transaction));
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::invertDisplay(bool invert)
{
  command({invert ? M_CMD(displayInvert) : M_CMD(displayNormal)});
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::dim(bool dim)
{
  uint8_t contrast;

  if (dim)
  {
    contrast = 0; // Dimmed display
  }
  else
  {
    contrast = (s_chargePumpEnabled ? 0xCF : 0x9F);
  }

  command({M_CMD(setContrast), contrast});
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::startScrollRight(uint8_t start, uint8_t stop)
{
  command({M_CMD(rightHorizontalScroll), 0x00, start, 0x00, stop, 0x00, 0xFF});
  command({M_CMD(enableScroll)});
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::startScrollLeft(uint8_t start, uint8_t stop)
{
  command({M_CMD(leftHorizontalScroll), 0x00, start, 0x00, stop, 0x00, 0xFF});
  command({M_CMD(enableScroll)});
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::startScrollDiagRight(uint8_t start, uint8_t stop)
{
  command({M_CMD(setVerticalScrollArea), 0x00, static_cast<uint8_t>(height())});
  command({M_CMD(verticalAndRightHorizontalScroll), 0x00, start, 0x00, stop, 0x01});
  command({M_CMD(enableScroll)});
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::startScrollDiagLeft(uint8_t start, uint8_t stop)
{
  command({M_CMD(setVerticalScrollArea), 0x00, static_cast<uint8_t>(height())});
  command({M_CMD(verticalAndLeftHorizontalScroll), 0x00, start, 0x00, stop, 0x01});
  command({M_CMD(enableScroll)});
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::stopScroll()
{
  command({M_CMD(disableScroll)});
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::command(uint8_t command)
{
  if (!m_spiDevice)
  {
    return;
  }

  gpio_set_level(m_pinConfig.dc, 0);

  spi_transaction_t transaction{};
  memset(&transaction, 0, sizeof(transaction));
  transaction.length = 8;
  transaction.tx_buffer = &command;
  transaction.rx_buffer = nullptr;

  ESP_ERROR_CHECK(spi_device_polling_transmit(m_spiDevice, &transaction));
}

//  ------------------------------------------------------------------------------------------------

void SSD1306::command(std::vector<uint8_t> command)
{
  if (!m_spiDevice)
  {
    return;
  }

  gpio_set_level(m_pinConfig.dc, 0);

  spi_transaction_t transaction{};
  memset(&transaction, 0, sizeof(transaction));
  transaction.length = command.size() * 8;
  transaction.tx_buffer = command.data();
  transaction.rx_buffer = nullptr;

  ESP_ERROR_CHECK(spi_device_polling_transmit(m_spiDevice, &transaction));
}

//  ------------------------------------------------------------------------------------------------

} // namespace esp32
} // namespace sl

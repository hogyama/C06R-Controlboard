#pragma once

#include <Arduino.h>
#include <driver/spi_master.h>
#include <esp_err.h>

#define FLASH_START_ADDRESS 0x000000
#define FLASH_END_ADDRESS 0xFFFFFF

// HAL for Infineon/Cypress S25FL128L / S25FL256L style SPI NOR flash.
// Initial target: S25FL128LAGMFI013, 3-byte address mode, single SPI mode 0/3.
class HalS25FL128L {
public:
    static constexpr uint32_t FLASH_SIZE_128M = 16UL * 1024UL * 1024UL; // 16 MB
    static constexpr uint32_t PAGE_SIZE       = 256UL;
    static constexpr uint32_t SECTOR_SIZE_4K  = 4UL * 1024UL;
    static constexpr uint32_t HALF_BLOCK_32K  = 32UL * 1024UL;
    static constexpr uint32_t BLOCK_SIZE_64K  = 64UL * 1024UL;

    struct Config {
        spi_host_device_t host = SPI2_HOST;
        int8_t sck  = -1;
        int8_t miso = -1;
        int8_t mosi = -1;
        int8_t cs   = -1;
        uint32_t clock_hz = 8000000;
        uint8_t mode = 3;          // S25FL supports SPI mode 0 and 3. Use 3 by default.
        int queue_size = 1;
        bool initialize_bus = true; // true: call spi_bus_initialize(). false: bus is already initialized.
        int dma_channel = SPI_DMA_CH_AUTO;
        int max_transfer_size = 4096;
    };

    struct JedecId {
        uint8_t manufacturer = 0;
        uint8_t memory_type  = 0;
        uint8_t capacity     = 0;
    };

    enum class Error : uint8_t {
        OK = 0,
        NotBegun,
        BadArgument,
        OutOfRange,
        SpiError,
        Timeout,
        WriteEnableFailed,
        ProgramOrEraseError,
        VerifyFailed,
    };

    HalS25FL128L() = default;
    ~HalS25FL128L();

    bool begin(const Config& cfg);
    void end();
    bool isBegun() const { return dev_ != nullptr; }

    Error lastError() const { return last_error_; }
    esp_err_t lastEspError() const { return last_esp_error_; }

    bool readJedecId(JedecId& id);
    bool isConnected();
    bool isLikelyS25FL128L();

    uint8_t readStatus1(); // RDSR1 05h, SR1V
    uint8_t readStatus2(); // RDSR2 07h, SR2V
    uint8_t readConfig1(); // RDCR1 35h
    uint8_t readConfig2(); // RDCR2 15h
    uint8_t readConfig3(); // RDCR3 33h

    bool waitReady(uint32_t timeout_ms);
    bool clearStatus();
    bool softwareReset();

    bool writeEnable();
    bool writeDisable();

    bool read(uint32_t addr, void* dst, size_t len);

    // Program 1..256 bytes within a single 256-byte page. Does not erase.
    // NOR flash programming can change bits from 1 to 0 only.
    bool programPage(uint32_t addr, const void* src, size_t len, bool verify = false);

    // Program arbitrary length, split at page boundaries. Does not erase.
    bool program(uint32_t addr, const void* src, size_t len, bool verify = false);

    bool erase4K(uint32_t addr);
    bool erase32K(uint32_t addr);
    bool erase64K(uint32_t addr);
    bool eraseRange4K(uint32_t addr, size_t len);
    bool chipErase(uint32_t timeout_ms = 240000UL);

    bool verify(uint32_t addr, const void* expected, size_t len);

    static uint32_t alignDown4K(uint32_t addr)  { return addr & ~(SECTOR_SIZE_4K - 1UL); }
    static uint32_t alignDown32K(uint32_t addr) { return addr & ~(HALF_BLOCK_32K - 1UL); }
    static uint32_t alignDown64K(uint32_t addr) { return addr & ~(BLOCK_SIZE_64K - 1UL); }

private:
    // Commands used in legacy 1-bit SPI, 3-byte address mode.
    static constexpr uint8_t CMD_RDID     = 0x9F;
    static constexpr uint8_t CMD_READ     = 0x03;
    static constexpr uint8_t CMD_WREN     = 0x06;
    static constexpr uint8_t CMD_WRDI     = 0x04;
    static constexpr uint8_t CMD_RDSR1    = 0x05;
    static constexpr uint8_t CMD_RDSR2    = 0x07;
    static constexpr uint8_t CMD_RDCR1    = 0x35;
    static constexpr uint8_t CMD_RDCR2    = 0x15;
    static constexpr uint8_t CMD_RDCR3    = 0x33;
    static constexpr uint8_t CMD_CLSR     = 0x30;
    static constexpr uint8_t CMD_RSTEN    = 0x66;
    static constexpr uint8_t CMD_RST      = 0x99;
    static constexpr uint8_t CMD_PP       = 0x02;
    static constexpr uint8_t CMD_SE_4K    = 0x20;
    static constexpr uint8_t CMD_HBE_32K  = 0x52;
    static constexpr uint8_t CMD_BE_64K   = 0xD8;
    static constexpr uint8_t CMD_CE       = 0x60;

    static constexpr uint8_t SR1_WIP = 0x01;
    static constexpr uint8_t SR1_WEL = 0x02;
    static constexpr uint8_t SR2_P_ERR = 0x20;
    static constexpr uint8_t SR2_E_ERR = 0x40;

    bool checkRange(uint32_t addr, size_t len);
    bool sendCommand(uint8_t cmd);
    uint8_t readReg8(uint8_t cmd);
    bool commandWithAddress(uint8_t cmd, uint32_t addr, uint32_t aligned_addr, uint32_t timeout_ms);
    bool markSpi(esp_err_t e);
    void setError(Error e);

    Config cfg_{};
    spi_device_handle_t dev_ = nullptr;
    bool bus_initialized_by_me_ = false;
    Error last_error_ = Error::OK;
    esp_err_t last_esp_error_ = ESP_OK;
};

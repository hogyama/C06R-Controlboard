#include "hal_S25FL128L.h"

HalS25FL128L::~HalS25FL128L()
{
    end();
}

void HalS25FL128L::setError(Error e)
{
    last_error_ = e;
}

bool HalS25FL128L::markSpi(esp_err_t e)
{
    last_esp_error_ = e;
    if (e == ESP_OK) {
        last_error_ = Error::OK;
        return true;
    }
    last_error_ = Error::SpiError;
    return false;
}

bool HalS25FL128L::begin(const Config& cfg)
{
    end();
    cfg_ = cfg;

    if (cfg_.cs < 0) {
        setError(Error::BadArgument);
        return false;
    }

    if (cfg_.initialize_bus) {
        if (cfg_.sck < 0 || cfg_.miso < 0 || cfg_.mosi < 0) {
            setError(Error::BadArgument);
            return false;
        }

        spi_bus_config_t bus_cfg = {};
        bus_cfg.sclk_io_num = cfg_.sck;
        bus_cfg.miso_io_num = cfg_.miso;
        bus_cfg.mosi_io_num = cfg_.mosi;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = cfg_.max_transfer_size;

        esp_err_t e = spi_bus_initialize(cfg_.host, &bus_cfg, cfg_.dma_channel);
        if (e != ESP_OK) {
            return markSpi(e);
        }
        bus_initialized_by_me_ = true;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = cfg_.clock_hz;
    dev_cfg.mode = cfg_.mode;
    dev_cfg.spics_io_num = cfg_.cs;
    dev_cfg.queue_size = cfg_.queue_size;
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;

    esp_err_t e = spi_bus_add_device(cfg_.host, &dev_cfg, &dev_);
    if (e != ESP_OK) {
        if (bus_initialized_by_me_) {
            spi_bus_free(cfg_.host);
            bus_initialized_by_me_ = false;
        }
        return markSpi(e);
    }

    // Device may still be completing a previous operation after reset/power event.
    if (!waitReady(1000)) {
        end();
        return false;
    }

    // Do not require exact ID here. Some compatible parts differ in memory_type.
    JedecId id;
    if (!readJedecId(id)) {
        end();
        return false;
    }
    if ((id.manufacturer == 0x00 && id.memory_type == 0x00 && id.capacity == 0x00) ||
        (id.manufacturer == 0xFF && id.memory_type == 0xFF && id.capacity == 0xFF)) {
        setError(Error::SpiError);
        end();
        return false;
    }

    setError(Error::OK);
    return true;
}

void HalS25FL128L::end()
{
    if (dev_ != nullptr) {
        spi_bus_remove_device(dev_);
        dev_ = nullptr;
    }
    if (bus_initialized_by_me_) {
        spi_bus_free(cfg_.host);
        bus_initialized_by_me_ = false;
    }
    last_esp_error_ = ESP_OK;
    last_error_ = Error::OK;
}

bool HalS25FL128L::checkRange(uint32_t addr, size_t len)
{
    if (len == 0) {
        return true;
    }
    if (addr >= FLASH_SIZE_128M) {
        setError(Error::OutOfRange);
        return false;
    }
    if (len > FLASH_SIZE_128M || addr > FLASH_SIZE_128M - len) {
        setError(Error::OutOfRange);
        return false;
    }
    return true;
}

bool HalS25FL128L::sendCommand(uint8_t cmd)
{
    if (dev_ == nullptr) {
        setError(Error::NotBegun);
        return false;
    }

    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.tx_data[0] = cmd;

    return markSpi(spi_device_polling_transmit(dev_, &t));
}

uint8_t HalS25FL128L::readReg8(uint8_t cmd)
{
    if (dev_ == nullptr) {
        setError(Error::NotBegun);
        return 0xFF;
    }

    uint8_t tx[2] = {cmd, 0x00};
    uint8_t rx[2] = {0x00, 0x00};

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    if (!markSpi(spi_device_polling_transmit(dev_, &t))) {
        return 0xFF;
    }
    return rx[1];
}

bool HalS25FL128L::readJedecId(JedecId& id)
{
    if (dev_ == nullptr) {
        setError(Error::NotBegun);
        return false;
    }

    uint8_t tx[4] = {CMD_RDID, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};

    spi_transaction_t t = {};
    t.length = 32;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    if (!markSpi(spi_device_polling_transmit(dev_, &t))) {
        return false;
    }

    id.manufacturer = rx[1];
    id.memory_type  = rx[2];
    id.capacity     = rx[3];
    return true;
}

bool HalS25FL128L::isConnected()
{
    JedecId id;
    if (!readJedecId(id)) {
        return false;
    }
    return !((id.manufacturer == 0x00 && id.memory_type == 0x00 && id.capacity == 0x00) ||
             (id.manufacturer == 0xFF && id.memory_type == 0xFF && id.capacity == 0xFF));
}

bool HalS25FL128L::isLikelyS25FL128L()
{
    JedecId id;
    if (!readJedecId(id)) {
        return false;
    }

    // For Cypress/Spansion/Infineon SPI NOR, manufacturer is usually 0x01.
    // 128 Mbit density code is commonly 0x18. Do not hard-fail on memory_type here.
    return (id.manufacturer == 0x01 && id.capacity == 0x18);
}

uint8_t HalS25FL128L::readStatus1()
{
    return readReg8(CMD_RDSR1);
}

uint8_t HalS25FL128L::readStatus2()
{
    return readReg8(CMD_RDSR2);
}

uint8_t HalS25FL128L::readConfig1()
{
    return readReg8(CMD_RDCR1);
}

uint8_t HalS25FL128L::readConfig2()
{
    return readReg8(CMD_RDCR2);
}

uint8_t HalS25FL128L::readConfig3()
{
    return readReg8(CMD_RDCR3);
}

bool HalS25FL128L::clearStatus()
{
    return sendCommand(CMD_CLSR);
}

bool HalS25FL128L::softwareReset()
{
    if (!sendCommand(CMD_RSTEN)) {
        return false;
    }
    if (!sendCommand(CMD_RST)) {
        return false;
    }
    delay(1);
    return waitReady(1000);
}

bool HalS25FL128L::waitReady(uint32_t timeout_ms)
{
    if (dev_ == nullptr) {
        setError(Error::NotBegun);
        return false;
    }

    const uint32_t start = millis();
    while (true) {
        const uint8_t sr2 = readStatus2();
        if (last_error_ == Error::SpiError) {
            return false;
        }
        if (sr2 & (SR2_P_ERR | SR2_E_ERR)) {
            clearStatus();
            setError(Error::ProgramOrEraseError);
            return false;
        }

        const uint8_t sr1 = readStatus1();
        if (last_error_ == Error::SpiError) {
            return false;
        }
        if ((sr1 & SR1_WIP) == 0) {
            setError(Error::OK);
            return true;
        }

        if (millis() - start >= timeout_ms) {
            setError(Error::Timeout);
            return false;
        }
        delay(1);
    }
}

bool HalS25FL128L::writeEnable()
{
    if (!waitReady(1000)) {
        return false;
    }
    if (!sendCommand(CMD_WREN)) {
        return false;
    }

    const uint8_t sr1 = readStatus1();
    if ((sr1 & SR1_WEL) == 0) {
        setError(Error::WriteEnableFailed);
        return false;
    }
    setError(Error::OK);
    return true;
}

bool HalS25FL128L::writeDisable()
{
    return sendCommand(CMD_WRDI);
}

bool HalS25FL128L::read(uint32_t addr, void* dst, size_t len)
{
    if (dev_ == nullptr) {
        setError(Error::NotBegun);
        return false;
    }
    if (dst == nullptr && len > 0) {
        setError(Error::BadArgument);
        return false;
    }
    if (!checkRange(addr, len)) {
        return false;
    }
    if (len == 0) {
        setError(Error::OK);
        return true;
    }
    if (!waitReady(1000)) {
        return false;
    }

    spi_transaction_t base = {};
    base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    base.cmd = CMD_READ;
    base.addr = addr;
    base.length = len * 8;
    base.rxlength = len * 8;
    base.rx_buffer = dst;

    spi_transaction_ext_t ext = {};
    ext.base = base;
    ext.command_bits = 8;
    ext.address_bits = 24;

    return markSpi(spi_device_polling_transmit(dev_, reinterpret_cast<spi_transaction_t*>(&ext)));
}

bool HalS25FL128L::programPage(uint32_t addr, const void* src, size_t len, bool do_verify)
{
    if (dev_ == nullptr) {
        setError(Error::NotBegun);
        return false;
    }
    if (src == nullptr || len == 0 || len > PAGE_SIZE) {
        setError(Error::BadArgument);
        return false;
    }
    if (!checkRange(addr, len)) {
        return false;
    }

    const uint32_t page_offset = addr & (PAGE_SIZE - 1UL);
    if (page_offset + len > PAGE_SIZE) {
        setError(Error::BadArgument); // page crossing is not allowed in this primitive
        return false;
    }

    if (!writeEnable()) {
        return false;
    }

    spi_transaction_t base = {};
    base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    base.cmd = CMD_PP;
    base.addr = addr;
    base.length = len * 8;
    base.tx_buffer = src;

    spi_transaction_ext_t ext = {};
    ext.base = base;
    ext.command_bits = 8;
    ext.address_bits = 24;

    if (!markSpi(spi_device_polling_transmit(dev_, reinterpret_cast<spi_transaction_t*>(&ext)))) {
        return false;
    }
    if (!waitReady(20)) {
        return false;
    }
    if (do_verify && !verify(addr, src, len)) {
        return false;
    }

    setError(Error::OK);
    return true;
}

bool HalS25FL128L::program(uint32_t addr, const void* src, size_t len, bool do_verify)
{
    if (src == nullptr && len > 0) {
        setError(Error::BadArgument);
        return false;
    }
    if (!checkRange(addr, len)) {
        return false;
    }

    const uint8_t* p = static_cast<const uint8_t*>(src);
    while (len > 0) {
        const uint32_t page_offset = addr & (PAGE_SIZE - 1UL);
        const size_t page_room = PAGE_SIZE - page_offset;
        const size_t n = (len < page_room) ? len : page_room;

        if (!programPage(addr, p, n, do_verify)) {
            return false;
        }
        addr += n;
        p += n;
        len -= n;
    }

    setError(Error::OK);
    return true;
}

bool HalS25FL128L::commandWithAddress(uint8_t cmd, uint32_t addr, uint32_t aligned_addr, uint32_t timeout_ms)
{
    if (dev_ == nullptr) {
        setError(Error::NotBegun);
        return false;
    }
    if (!checkRange(aligned_addr, 1)) {
        return false;
    }
    if (!writeEnable()) {
        return false;
    }

    spi_transaction_t base = {};
    base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    base.cmd = cmd;
    base.addr = aligned_addr;
    base.length = 0;

    spi_transaction_ext_t ext = {};
    ext.base = base;
    ext.command_bits = 8;
    ext.address_bits = 24;

    if (!markSpi(spi_device_polling_transmit(dev_, reinterpret_cast<spi_transaction_t*>(&ext)))) {
        return false;
    }
    return waitReady(timeout_ms);
}

bool HalS25FL128L::erase4K(uint32_t addr)
{
    return commandWithAddress(CMD_SE_4K, addr, alignDown4K(addr), 1000);
}

bool HalS25FL128L::erase32K(uint32_t addr)
{
    return commandWithAddress(CMD_HBE_32K, addr, alignDown32K(addr), 2000);
}

bool HalS25FL128L::erase64K(uint32_t addr)
{
    return commandWithAddress(CMD_BE_64K, addr, alignDown64K(addr), 3000);
}

bool HalS25FL128L::eraseRange4K(uint32_t addr, size_t len)
{
    if (len == 0) {
        setError(Error::OK);
        return true;
    }
    if (!checkRange(addr, len)) {
        return false;
    }

    uint32_t p = alignDown4K(addr);
    const uint32_t end = addr + len;
    while (p < end) {
        if (!erase4K(p)) {
            return false;
        }
        p += SECTOR_SIZE_4K;
    }

    setError(Error::OK);
    return true;
}

bool HalS25FL128L::chipErase(uint32_t timeout_ms)
{
    if (!writeEnable()) {
        return false;
    }
    if (!sendCommand(CMD_CE)) {
        return false;
    }
    return waitReady(timeout_ms);
}

bool HalS25FL128L::verify(uint32_t addr, const void* expected, size_t len)
{
    if (expected == nullptr && len > 0) {
        setError(Error::BadArgument);
        return false;
    }
    if (!checkRange(addr, len)) {
        return false;
    }

    const uint8_t* exp = static_cast<const uint8_t*>(expected);
    uint8_t buf[128];

    while (len > 0) {
        const size_t n = (len < sizeof(buf)) ? len : sizeof(buf);
        if (!read(addr, buf, n)) {
            return false;
        }
        if (memcmp(buf, exp, n) != 0) {
            setError(Error::VerifyFailed);
            return false;
        }
        addr += n;
        exp += n;
        len -= n;
    }

    setError(Error::OK);
    return true;
}

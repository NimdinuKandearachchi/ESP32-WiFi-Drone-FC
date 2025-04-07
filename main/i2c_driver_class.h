#pragma once /// include this library only once in compiling

#include "driver/i2c.h"
#include "esp_log.h"

#define READ_TO_BUF 0

#define I2C_MASTER_SCL_IO 3  // GPIO for SCL
#define I2C_MASTER_SDA_IO 2  // GPIO for SDA
#define I2C_MASTER_FREQ_KHZ 400 // 1mhz for faster speed
#define I2C_TIMEOUT pdMS_TO_TICKS(100)

/// @brief i2c port 1 only
class ESP_I2C_IDF{
    uint8_t i2c_sda, i2c_scl;
    uint16_t i2c_frq_khz;
    i2c_port_t i2c_port;

public:
#if READ_TO_BUF
    uint8_t readbuf[64];
#endif

    ESP_I2C_IDF(uint8_t _sda = I2C_MASTER_SDA_IO, uint8_t _scl = I2C_MASTER_SCL_IO, uint16_t _frq = I2C_MASTER_FREQ_KHZ, i2c_port_t _port = I2C_NUM_0)
    :i2c_sda(_sda), i2c_scl(_scl), i2c_frq_khz(_frq),i2c_port(_port) {

    }

    void install_i2c_driver(){
        i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = this->i2c_sda,
            .scl_io_num = this->i2c_scl,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                .clk_speed = (uint32_t)this->i2c_frq_khz * 1000,
            },
            .clk_flags = 0,
        };
        
        i2c_param_config(this->i2c_port, &i2c_conf);
        i2c_driver_install(this->i2c_port, i2c_conf.mode, 0,0,0);
    }
    void reset_fifo(){
        i2c_reset_rx_fifo(this->i2c_port);
        i2c_reset_tx_fifo(this->i2c_port);
    }
    esp_err_t write_bytes(uint8_t dev_add, uint8_t *buf, size_t size){
        return i2c_master_write_to_device(this->i2c_port, dev_add, buf, size, I2C_TIMEOUT);
    }
    esp_err_t read_bytes(uint8_t dev_add, uint8_t *buf, size_t size){
        return i2c_master_read_from_device(this->i2c_port, dev_add, buf, size, I2C_TIMEOUT);
    }
    esp_err_t write_bytes_read_bytes(uint8_t dev_add, uint8_t *wbuf, size_t wsize, uint8_t *rbuf, size_t rsize){
        return i2c_master_write_read_device(this->i2c_port, dev_add, wbuf, wsize, rbuf, rsize, I2C_TIMEOUT);
    }
    esp_err_t write_byte_to_reg(uint8_t dev_add, uint8_t reg, uint8_t byte){
        uint8_t trdt[2] = {reg, byte};
        return i2c_master_write_to_device(this->i2c_port, dev_add, trdt, 2, I2C_TIMEOUT);
    }
    uint8_t read_byte_fm_reg(uint8_t dev_add, uint8_t reg){
        uint8_t rbuf = 0;
        i2c_master_write_read_device(this->i2c_port, dev_add, &reg, 1, &rbuf, 1, I2C_TIMEOUT);
        return rbuf;
    }
    esp_err_t read_bytes_fm_reg(uint8_t dev_add, uint8_t reg, uint8_t *rbuf, size_t rsize){
        return i2c_master_write_read_device(this->i2c_port, dev_add, &reg, 1, rbuf, rsize, I2C_TIMEOUT);
    }
    
    void i2c_scan() {
        ESP_LOGI("I2C", "Starting I2C scan...");
        for (uint8_t address = 1; address < 127; address++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(this->i2c_port, cmd, 10 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                ESP_LOGI("I2C", "Device found at address 0x%02X", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGW("I2C", "Timeout at address 0x%02X", address);
            }
        }
        ESP_LOGI("I2C", "I2C scan complete");
    }
#if READ_TO_BUF
    void read_from_dev(uint8_t dev_add){
        i2c_master_read_from_device(this->i2c_port, dev_add, this->readbuf, sizeof(this->readbuf), I2C_TIMEOUT);
    }
    void get_from_dev(uint8_t dev_add, uint8_t *wbuf, size_t wsize){
        i2c_master_write_read_device(this->i2c_port, dev_add, wbuf, wsize, this->readbuf, sizeof(this->readbuf), I2C_TIMEOUT);
    }
#endif
};

#include "MLX90614.hpp"
#include <cstring>

static uint8_t MLX90614_CalcPEC(const uint8_t *data, uint8_t len){
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++){
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++){
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

MLX90614::MLX90614(){
    bus = nullptr;
    dev = nullptr;
    address = 0x5A;
}

esp_err_t MLX90614::init(i2c_master_bus_handle_t _bus, uint8_t slaveAddr, uint32_t sclSpeed) {
    if (!_bus) return ESP_ERR_INVALID_ARG;

    bus = _bus;
    address = slaveAddr;

    i2c_master_dev_config_t cfg{};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = address;
    cfg.scl_speed_hz = sclSpeed;

    return i2c_master_bus_add_device(bus, &cfg, &dev);
}

esp_err_t MLX90614::read(uint16_t reg, uint16_t *out){
    if (!bus || !dev) return ESP_ERR_INVALID_STATE;
    if (!out) return ESP_ERR_INVALID_ARG;

    uint8_t cmd = (uint8_t)reg;
    uint8_t rx[3];

    esp_err_t err = i2c_master_transmit_receive(
        dev, &cmd, 1, rx, 3, 1000);

    if (err != ESP_OK)
        return err;

    uint8_t addrW = (address << 1) | 0;
    uint8_t addrR = (address << 1) | 1;

    uint8_t pecData[5] = {addrW, cmd, addrR, rx[0], rx[1]};

    uint8_t pec = MLX90614_CalcPEC(pecData, 5);

    if (pec != rx[2])
        return ESP_ERR_INVALID_CRC;

    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    return ESP_OK;
}

esp_err_t MLX90614::write(uint16_t reg, uint16_t data){
    if (!bus || !dev) return ESP_ERR_INVALID_STATE;

    uint8_t tx[4];

    tx[0] = (uint8_t)reg;
    tx[1] = (uint8_t)(data & 0xFF);
    tx[2] = (uint8_t)(data >> 8);

    uint8_t pecData[4] = {(uint8_t)(address << 1), tx[0], tx[1], tx[2]};

    tx[3] = MLX90614_CalcPEC(pecData, 4);
    return i2c_master_transmit(dev, tx, sizeof(tx), 1000);
}

esp_err_t MLX90614::writeEEPROM(uint16_t reg, uint16_t data){
    if (!bus || !dev) return ESP_ERR_INVALID_STATE;

    esp_err_t err;

    err = write(reg, 0x0000);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    err = write(reg, data);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

esp_err_t MLX90614::readFlags(uint16_t *flags){
    return read(0xF0, flags);
}


esp_err_t MLX90614::enterSleep(){
    uint8_t cmd = 0xFF;
    return i2c_master_transmit(dev, &cmd, 1, 1000);
}

esp_err_t MLX90614::wake(){
    // Datasheet: SDA low pulse > 33ms or bus reset
    // Sketchy to impliment, need to take over clk and sda pins for extended periods
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

float MLX90614::toCelsius(uint16_t raw){
    return (raw * 0.02f) - 273.15f;
}

float MLX90614::toFahrenheit(float c){
    return (c * 1.8f) + 32.0f;
}

esp_err_t MLX90614::getAmbientTemp(float *t){
    uint16_t raw;
    esp_err_t err = read(0x06, &raw);
    if (err != ESP_OK) return err;

    *t = toCelsius(raw);
    return ESP_OK;
}

esp_err_t MLX90614::getObjectTemp(float *t){
    uint16_t raw;
    esp_err_t err = read(0x07, &raw);
    if (err != ESP_OK) return err;

    *t = toCelsius(raw);
    return ESP_OK;
}

esp_err_t MLX90614::setEmissivity(float e){
    if (e <= 0.0f || e > 1.0f)
        return ESP_ERR_INVALID_ARG;

    uint16_t raw = (uint16_t)(e * 65535.0f);
    return writeEEPROM(0x24, raw);
}

esp_err_t MLX90614::getEmissivity(float *e){
    uint16_t raw;
    esp_err_t err = read(0x24, &raw);
    if (err != ESP_OK) return err;

    *e = (float)raw / 65535.0f;
    return ESP_OK;
}
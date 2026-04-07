#ifndef _MLX90614_H
#define _MLX90614_H

#include <cstdint>
#include <esp_err.h>
#include <driver/i2c_master.h>

#define ESP_ERR_MLX90614_I2C_TIMEOUT    (ESP_FAIL + 0x10)
#define ESP_ERR_MLX90614_PROTECTED_MEM  (ESP_FAIL + 0x11)
#define ESP_ERR_MLX90614_BUS_UNDEFINED  (ESP_FAIL + 0x12)
#define ESP_ERR_MLX90614_DEV_UNDEFINED  (ESP_FAIL + 0x13)
#define ESP_ERR_MLX90614_INVALID_CRC    (ESP_FAIL + 0x14)

class MLX90614 {
public:
    MLX90614();

    /** Initialize MLX90614 device on an existing I2C bus
     *
     * @param[in] bus I2C bus handle
     * @param[in] slaveAddr 7-bit I2C address of the sensor (default 0x5A)
     * @param[in] sclSpeed I2C clock speed in Hz (10kHz–100kHz)
     *
     * @retval ESP_OK on success
     * @retval ESP_ERR_INVALID_ARG if parameters are invalid
     * @retval ESP_FAIL if device could not be added to bus
     */
    esp_err_t init(i2c_master_bus_handle_t bus, uint8_t slaveAddr = 0x5A, uint16_t sclSpeed = 100000);

    /** Read one or more 16-bit registers from MLX90614
     *
     * @param[in] startAddress First register address to read
     * @param[in] nAddrRead Number of consecutive registers to read
     * @param[out] rData Pointer to output buffer (must be preallocated)
     *
     * @retval ESP_OK on success
     * @retval ESP_ERR_INVALID_ARG invalid parameters
     * @retval ESP_ERR_INVALID_CRC PEC validation failed
     */
    esp_err_t read(uint16_t startAddress, uint16_t nAddrRead, uint16_t *rData);

    /** Write a 16-bit value to MLX90614 EEPROM/RAM register
     *
     * @param[in] startAddress Register address to write to
     * @param[in] wData 16-bit data to write
     *
     * @retval ESP_OK on success
     * @retval ESP_FAIL if I2C transaction fails
     */
    esp_err_t write(uint16_t startAddress, uint16_t wData);

    /** Send a raw MLX90614 command (sleep, reset, etc.)
     *
     * @param[in] cmd Command byte to send
     *
     * @retval ESP_OK on success
     */
    esp_err_t command(uint16_t cmd);

    /** Dump entire EEPROM memory */
    esp_err_t dumpEE(uint16_t *eeData);

    /** Dump entire RAM memory */
    esp_err_t dumpRAM(uint16_t *ramData);

    /** Get object temperature scaling maximum */
    esp_err_t getObjectMaxTemp(uint16_t *objMaxTemp);

    /** Set object temperature scaling maximum */
    esp_err_t setObjectMaxTemp(uint16_t wObjMaxTemp);

    /** Get object temperature scaling minimum */
    esp_err_t getObjectMinTemp(uint16_t *objMinTemp);

    /** Set object temperature scaling minimum */
    esp_err_t setObjectMinTemp(uint16_t wObjMinTemp);

    /** Get ambient temperature scaling range */
    esp_err_t getAmbientRange(uint16_t *ambRange);

    /** Set ambient temperature scaling range */
    esp_err_t setAmbientRange(uint16_t wAmbRange);

    /** Get emissivity correction factor */
    esp_err_t getEmissivity(float *emissivity);

    /** Set emissivity correction factor */
    esp_err_t setEmissivity(float wEmissivity);

    /** Get FIR filter setting */
    esp_err_t getFIR(uint8_t *fir);

    /** Set FIR filter setting */
    esp_err_t setFIR(uint8_t wFir);

    /** Get IIR filter setting */
    esp_err_t getIIR(uint8_t *iir);

    /** Set IIR filter setting */
    esp_err_t setIIR(uint8_t wIir);

    /** Read ambient temperature (C) */
    esp_err_t getAmbientTemp(float *amb);

    /** Read primary object temperature (C) */
    esp_err_t getObjectTemp(float *obj);

    /** Read secondary object temperature (C, if supported) */
    esp_err_t getObjectTemp2(float *obj2);

    /** Read raw IR data (sensor 1) */
    esp_err_t getIRdata(uint16_t *ir);

    /** Read raw IR data (sensor 2, if supported) */
    esp_err_t getIRdata2(uint16_t *ir2);

    /** Read status flags register */
    esp_err_t readFlags(uint16_t *flags);

    /** Put sensor into sleep mode */
    esp_err_t enterSleep();

    /** Wake sensor from sleep mode */
    esp_err_t wake();

    /** Convert Celsius to Fahrenheit */
    float toFahrenheit(float celsius);

    /** Convert raw IR value to signed temperature representation */
    int16_t convertIR(uint16_t ir);

private:
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t addr;

    /** Compute SMBus Packet Error Code (PEC) */
    static uint8_t calcPEC(uint8_t *data, uint8_t len);
};

#endif
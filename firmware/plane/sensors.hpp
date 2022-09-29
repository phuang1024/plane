#pragma once

#include <Wire.h>

#include "utils.hpp"


struct IMURead {
    int16_t ax, ay, az;  // Accelerometer
    int16_t gx, gy, gz;  // Gyroscope
};

class IMU {
public:
    void init() {
        Wire.beginTransmission(I2C_IMU);
        Wire.write(0x6B);
        Wire.write(0);
        Wire.endTransmission(true);
    }

    IMURead read() {
        // Request data
        Wire.beginTransmission(I2C_IMU);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(I2C_IMU, 14, true);
    
        IMURead res;
        res.ax = read_u16();
        res.ay = read_u16();
        res.az = read_u16();
        read_u16();  // temp
        res.gx = read_u16();
        res.gy = read_u16();
        res.gz = read_u16();
    
        return res;
    }
};


struct BaroRead {
    // decicelcius (309 = 30.9C)
    int16_t temp;
    // pascal (N / m^2)
    uint32_t pressure;
};

/**
 * Followed https://www.sparkfun.com/tutorials/253
 * Max read speed: 100/sec = 10ms/read (based on my estimate)
 */
class Baro {
public:
    void init() {
        ac1 = read16_addr(0xAA);
        ac2 = read16_addr(0xAC);
        ac3 = read16_addr(0xAE);
        ac4 = read16_addr(0xB0);
        ac5 = read16_addr(0xB2);
        ac6 = read16_addr(0xB4);
        b1 = read16_addr(0xB6);
        b2 = read16_addr(0xB8);
        mb = read16_addr(0xBA);
        mc = read16_addr(0xBC);
        md = read16_addr(0xBE);
    }

    BaroRead read() {
        BaroRead res;
        res.temp = read_temp();
        res.pressure = read_pressure();
        return res;
    }

    int16_t read_temp() {
        // Request temp reading.
        Wire.beginTransmission(I2C_BARO);
        Wire.write(0xF4);
        Wire.write(0x2E);
        Wire.endTransmission();
        delay(5);
        int16_t raw = read16_addr(0xF6);

        // Convert
        int32_t x1 = (((int32_t)raw - (int32_t)ac6) * (int32_t)ac5) >> 15,
                x2 = ((long)mc << 11) / (x1 + md);
        b5 = x1 + x2;
        int16_t temp = (b5 + 8) >> 4;

        return temp;
    }

    uint32_t read_pressure() {
        // Request pressure reading
        Wire.beginTransmission(I2C_BARO);
        Wire.write(0xF4);
        Wire.write(0x34 + (oversamp << 6));
        Wire.endTransmission();
        delay(2 + (3 << oversamp));
        //uint32_t raw = read24_addr(0xF6) >> (8 - oversamp);
        Wire.beginTransmission(I2C_BARO);
        Wire.write(0xF6);
        Wire.endTransmission();
        Wire.requestFrom(I2C_BARO, 3);
        uint32_t raw = (((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | (uint32_t)Wire.read()) >> (8 - oversamp);

        // Convert
        // TODO bug i think
        int32_t x1, x2, x3, b3, b6, p;
        uint32_t b4, b7;

        b6 = b5 - 4000;

        x1 = (b2 * (b6*b6) >> 12) >> 11;
        x2 = (ac2 * b6) >> 11;
        x3 = x1 + x2;
        b3 = ((((int32_t)ac1 * 4 + x3) << oversamp) + 2) >> 2;

        x1 = (ac3 * b6) >> 13;
        x2 = (b1 * ((b6*b6) >> 12)) >> 16;
        x3 = (x1 + x2 + 2) >> 2;
        b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

        b7 = (uint32_t)(raw - b3) * (50000 >> oversamp);
        if (b7 < 0x80000000)
            p = (b7 << 1) / b4;
        else
            p = (b7 / b4) << 1;

        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        p += (x1 + x2 + 3791) >> 4;

        return p;
    }

private:
    // Pressure oversampling (0 to 3 incl).
    const uint8_t oversamp = 2;

    // Calibration constants
    int16_t ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md;
    // This is set by temp and used by baro
    int32_t b5;

    void request_addr(int addr, int num_bytes) {
        Wire.beginTransmission(I2C_BARO);
        Wire.write(addr);
        Wire.endTransmission();
        Wire.requestFrom(I2C_BARO, num_bytes);
    }

    uint16_t read16_addr(int addr) {
        request_addr(addr, 2);
        return read_u16();
    }

    uint32_t read24_addr(int addr) {
        request_addr(addr, 3);
        return read_u24();
    }
};


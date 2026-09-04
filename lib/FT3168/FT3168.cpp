#include "FT3168.h"
#include <Wire.h>

FT3168::FT3168(int8_t sda_pin, int8_t scl_pin, int8_t rst_pin, int8_t int_pin)
{
    _sda = sda_pin;
    _scl = scl_pin;
    _rst = rst_pin;
    _int = int_pin;
}

// void FT3168::begin(void)
// {
//     // Initialize I2C
//     if (_sda != -1 && _scl != -1)
//     {
//         Wire.begin(_sda, _scl);
//     }
//     else
//     {
//         Wire.begin();
//     }

//     // Int Pin Configuration
//     if (_int != -1)
//     {
//         pinMode(_int, OUTPUT);
//         digitalWrite(_int, HIGH); // 高电平
//         delay(1); 
//         digitalWrite(_int, LOW); // 低电平
//         delay(1);
//     }

//     // Reset Pin Configuration
//     if (_rst != -1)
//     {
//         pinMode(_rst, OUTPUT);
//         digitalWrite(_rst, LOW);
//         delay(10);
//         digitalWrite(_rst, HIGH);
//         delay(300);
//     }

//     // Initialize Touch
//     i2c_write(0x00, 0x00); // 切换到工厂模式
// }


void FT3168::begin(void)
{
    // NOTE: assumes Wire.begin() has already been called in main.cpp

    // Int Pin Configuration
    if (_int != -1)
    {
        pinMode(_int, OUTPUT);
        digitalWrite(_int, HIGH);
        delay(1); 
        digitalWrite(_int, LOW);
        delay(1);
    }

    // Reset Pin Configuration
    if (_rst != -1)
    {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, LOW);
        delay(10);
        digitalWrite(_rst, HIGH);
        delay(300);
    }

    // Initialize Touch – switch to factory mode
    i2c_write(0x00, 0x00);
}


bool FT3168::getTouch(uint16_t *x, uint16_t *y, uint8_t *gesture)
{
    bool FingerIndex = (bool)i2c_read(0x02);

    // The hardware gesture register (0xD1) isn't used by the project -
    // gesture navigation is computed by LVGL itself
    // (lv_indev_get_gesture_dir from the x/y points), so we skip this
    // read: one less I2C transaction on EVERY touch poll, i.e. fewer
    // chances to hang on the shared bus with the IMU.
    *gesture = None;

    // Last known position - if a single I2C read fails (glitch on the
    // shared bus), we'd rather return the previous point than stack
    // garbage, which would break gesture detection in LVGL (a sudden
    // position "jump" mid-swipe).
    static uint16_t lastX = 0, lastY = 0;

    uint8_t data[4];
    if (i2c_read_continuous(0x03, data, 4) == 0) {
        lastX = ((data[0] & 0x0F) << 8) | data[1];
        lastY = ((data[2] & 0x0F) << 8) | data[3];
    }
    *x = lastX;
    *y = lastY;

    return FingerIndex;
}

uint8_t FT3168::i2c_read(uint8_t addr)
{
    uint8_t rdData = 0;
    uint8_t rdDataCount = 0;

    // NOTE: this used to be `do {...} while (rdDataCount == 0)` with NO
    // retry limit. On a failed I2C transaction (shared bus with the IMU)
    // it could spin indefinitely, blocking the whole loop() (LVGL
    // rendering and touch reading together - hence the "screen freezing"
    // and "poor responsiveness" feel). Capped to a few attempts.
    for (uint8_t attempt = 0; attempt < 3 && rdDataCount == 0; attempt++) {
        Wire.beginTransmission(I2C_ADDR_FT3168);
        Wire.write(addr);
        Wire.endTransmission(false); // Restart
        rdDataCount = Wire.requestFrom(I2C_ADDR_FT3168, 1);
    }
    while (Wire.available())
    {
        rdData = Wire.read();
    }
    return rdData;
}

uint8_t FT3168::i2c_read_continuous(uint8_t addr, uint8_t *data, uint32_t length)
{
    Wire.beginTransmission(I2C_ADDR_FT3168);
    Wire.write(addr);
    if (Wire.endTransmission(true)) return -1;

    uint8_t got = Wire.requestFrom(I2C_ADDR_FT3168, length);
    if (got < length) return -1; // incomplete read - don't overwrite data with garbage

    for (uint32_t i = 0; i < length; i++)
    {
        data[i] = Wire.read();
    }
    return 0;
}

void FT3168::i2c_write(uint8_t addr, uint8_t data)
{
    Wire.beginTransmission(I2C_ADDR_FT3168);
    Wire.write(addr);
    Wire.write(data);
    Wire.endTransmission();
}

// 添加新的方法来读取寄存器

// uint8_t FT3168::readModeSwitch()
// {
//     return i2c_read(0x00);
// }

// uint8_t FT3168::readTDStatus()
// {
//     return i2c_read(0x02);
// }

// uint16_t FT3168::readP1X()
// {
//     uint8_t high = i2c_read(0x03) & 0x0F;
//     uint8_t low = i2c_read(0x04);
//     return (high << 8) | low;
// }

// uint16_t FT3168::readP1Y()
// {
//     uint8_t high = i2c_read(0x05) & 0x0F;
//     uint8_t low = i2c_read(0x06);
//     return (high << 8) | low;
// }

// 根据需要添加更多的方法以读取其他寄存器...

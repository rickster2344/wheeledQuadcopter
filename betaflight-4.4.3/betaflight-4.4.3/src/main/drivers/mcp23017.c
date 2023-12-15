#include "bus.h"
#include "bus_i2c.h"
#include "mcp23017.h"

expander_t exp;

void deviceConfigure(extDevice_t *dev) {
    uint8_t address, reg, length;
    uint8_t data = 0b11110000; // 1: input 0: output
    reg = MCP23017_IODIRA;
    length = 1;
    i2cWrite(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, reg, data);
}

_Bool MCP23017Detect() {
    extDevice_t *dev = &exp->dev;

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for BMP280
        dev->busType_u.i2c.address = MCP23017_ADDRESS;
    }

    deviceConfigure(dev);

    busDeviceRegister(dev);


    return 1;
}
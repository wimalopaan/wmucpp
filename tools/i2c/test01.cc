#include <unistd.h>
#define Sleep(ms) usleep(ms * 1000)

#include <stdio.h>
#include <string.h>
#include "libmcp2221.h"
#include <hidapi/hidapi.h>

int main() {
    mcp2221_init();
    printf("Looking for devices... ");
    int count = mcp2221_find(MCP2221_DEFAULT_VID, MCP2221_DEFAULT_PID, NULL, NULL, NULL);
    printf("found %d devices\n", count);
    printf("Opening device... ");
    mcp2221_t* myDev = mcp2221_open();
    
    if(!myDev) {
        mcp2221_exit();
        puts("No MCP2221s found");
        return 0;
    }
    
    
    // Print out info
    printf("MCP2221 device opened: %s\n", myDev->path);
    printf("  Manufacturer:  %ls\n",	myDev->usbInfo.manufacturer);
    printf("  Product:       %ls\n",	myDev->usbInfo.product);
    printf("  Serial:        %ls\n",	myDev->usbInfo.serial);
    printf("  Fact serial:   %.*s\n",	myDev->usbInfo.factorySerialLen, myDev->usbInfo.factorySerial);
    printf("  VID:           %hx\n",	myDev->usbInfo.vid);
    printf("  PID:           %hx\n",	myDev->usbInfo.pid);
    printf("  Firmware:      %c.%c\n",	myDev->usbInfo.firmware[0], myDev->usbInfo.firmware[1]);
    printf("  Hardware:      %c.%c\n",	myDev->usbInfo.hardware[0], myDev->usbInfo.hardware[1]);
    printf("  Power source:  %s\n",		myDev->usbInfo.powerSource == MCP2221_PWRSRC_SELFPOWERED ? "Self powered" : "Bus powered");
    printf("  Remote wakeup: %s\n",		myDev->usbInfo.remoteWakeup == MCP2221_WAKEUP_ENABLED ? "Enabled" : "Disabled");
    printf("  Current:       %dmA\n",	myDev->usbInfo.milliamps);
    
    uint8_t data[8];

    if(mcp2221_isConnected(myDev) != MCP2221_SUCCESS) {
        puts("Device is not connected!");
    }
    else {
        puts("Device is OK");
        
        mcp2221_error res{};
        
        Sleep(200);
        puts("~~~~~~~~~~~~~");
        
        
//        // Read I2C pin values
//        mcp2221_i2cpins_t i2cPins{};
//        res = mcp2221_i2cReadPins(myDev, &i2cPins);
//        if(res != MCP2221_SUCCESS)
//            goto error;
        
//        printf("SDA: %hhu\n", i2cPins.SDA);
//        printf("SCL: %hhu\n", i2cPins.SCL);
        
        data[0] = 0;
        data[1] = 33;
//        res = mcp2221_i2cWrite(myDev, 50, data, 2, MCP2221_I2CRW_NORMAL);
        res = mcp2221_i2cWrite(myDev, 0x54, data, 2, MCP2221_I2CRW_NORMAL);
        if(res != MCP2221_SUCCESS)
            goto error;
        
//        res = mcp2221_i2cRead(myDev, 50, 1, MCP2221_I2CRW_NORMAL);
//        if(res != MCP2221_SUCCESS)
//            goto error;
        
        error:
        switch(res) {
        case MCP2221_SUCCESS:
            puts("No error");
            break;
        case MCP2221_ERROR:
            puts("General error");
            break;
        case MCP2221_INVALID_ARG:
            puts("Invalid argument, probably null pointer");
            break;
        case MCP2221_ERROR_HID:
            printf("USB HID Error: %ls\n", hid_error((hid_device*) myDev->handle));
            break;
        default:
            printf("Unknown error %d\n", res);
            break;
        }
    }
    
    mcp2221_exit();
    
    return 0;
}

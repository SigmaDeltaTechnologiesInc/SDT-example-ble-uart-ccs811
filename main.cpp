/* SDT-example-ble-uart-echo
 * 
 * Copyright (c) 2018 Sigma Delta Technologies Inc.
 * 
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "mbed.h"
#include "ble/BLE.h"
#include "ble/services/UARTService.h"
#include "CCS811.h"

/* Serial */
#define BAUDRATE 9600
Serial serial_pc(USBTX, USBRX, BAUDRATE);

/* DigitalOut */
#define LED_ON      0
#define LED_OFF     1
DigitalOut do_ledRed(LED_RED, LED_OFF);       // For SDT52832B, the GPIO0 operates as an NFC function and cannot serve as LED0
DigitalOut do_ledGreen(LED_GREEN, LED_OFF);   // For SDT52832B, the GPIO1 operates as an NFC function and cannot serve as LED1
DigitalOut do_ledBlue(LED_BLUE, LED_OFF);
DigitalOut* pDO_led = &do_ledBlue;

/* Ticker */
Ticker ticker;

/* EventQueue */
EventQueue eventQueue(/* event count */ 16 * /* event size */ 32);

/* BLE */
#define BLE_DEVICE_NAME "SDT Device"
BLE& ble_SDTDevice = BLE::Instance();      // you can't use this name, 'ble', because this name is already declared in UARTService.h

/* UART service */
#define UARTSERVICE_WRITE_DATA_MAX  20
UARTService* pUartService;

/* Variable */
bool b_bleConnect = false;
bool b_uartStart = false;
char uc_rxBuf1[50] = {'\0', };           // DataBuffer to write by BLE
char uc_rxBuf2[50] = {'\0', };           // DataBuffer to write by BLE

/* Sensor */
CCS811 ccs811(I2C1_SDA, I2C1_SCL);
DigitalOut do_ccs811_wake(SPI1_SS0, 0);     // nWAKE pin is an active low input and should be asserted by the host prior to an I²C transaction and held low throughout.
uint16_t eco2, tvoc;
char status = 0;



void callbackTicker(void) {
    *pDO_led = !(*pDO_led);
}

void readCCS811(void) {
    status = ccs811.getStatus(false);       // 0x98 : There is a new data to be read
    while(!(status & 0x08));                // This bit is cleared when data is read on the I²C interface
    ccs811.getData(&eco2, &tvoc);
    serial_pc.printf("eCO2 reading :%dppm, TVOC reading :%dppb\r\n", eco2, tvoc);
}

void callbackEventQueue(void) {
    if (b_uartStart) {
        readCCS811();
        wait(1);

        if (pUartService) {
            sprintf(uc_rxBuf1, "%d", eco2);      // decimal to char[]
            pUartService->write(uc_rxBuf1, UARTSERVICE_WRITE_DATA_MAX);
            sprintf(uc_rxBuf2, "%d", tvoc);      // decimal to char[]
            pUartService->write(uc_rxBuf2, UARTSERVICE_WRITE_DATA_MAX);
        }
    }
}

void callbackPeriodicEvent(void) {
    if (ble_SDTDevice.gap().getState().connected) {
        eventQueue.call(callbackEventQueue);
    }
}

void callbackEventsToProcess(BLE::OnEventsToProcessCallbackContext* context) {
    eventQueue.call(Callback<void()>(&ble_SDTDevice, &BLE::processEvents));
}

void callbackBleDataWritten(const GattWriteCallbackParams* params) {
    if ((pUartService != NULL) && (params->handle == pUartService->getTXCharacteristicHandle())) {
        // uint16_t bytesRead = params->len;
        const unsigned char* pBleRxBuf = params->data;
        // g_Serial_Pc.printf("data from BLE : %s\r\n", params->data);  // For debug

        if (pBleRxBuf[0] == 's') {
            b_uartStart = true;
        }
        else if (pBleRxBuf[0] == 'p') {
            b_uartStart = false;
        }
    }
}

void callbackBleConnection(const Gap::ConnectionCallbackParams_t* params) {
    serial_pc.printf("Connected!\n");
    b_bleConnect = true;
    ticker.attach(callbackTicker, 1);
}

void callbackBleDisconnection(const Gap::DisconnectionCallbackParams_t* params) {
    serial_pc.printf("Disconnected!\n");
    serial_pc.printf("Restarting the advertising process\n\r");
    b_bleConnect = false;
    ticker.detach();
    *pDO_led = LED_ON;
    ble_SDTDevice.gap().startAdvertising();
}

void callbackBleInitComplete(BLE::InitializationCompleteCallbackContext* params) {
    BLE& ble = params->ble;                     // 'ble' equals ble_SDTDevice declared in global
    ble_error_t error = params->error;          // 'error' has BLE_ERROR_NONE if the initialization procedure started successfully.

    if (error == BLE_ERROR_NONE) {
        serial_pc.printf("Initialization completed successfully\n");
    }
    else {
        /* In case of error, forward the error handling to onBleInitError */
        serial_pc.printf("Initialization failled\n");
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        serial_pc.printf("ID of BLE instance is not DEFAULT_INSTANCE\n");
        return;
    }

    /* Setup UARTService */
    pUartService = new UARTService(ble);

    /* Setup and start advertising */
    ble.gattServer().onDataWritten(callbackBleDataWritten);
    ble.gap().onConnection(callbackBleConnection);
    ble.gap().onDisconnection(callbackBleDisconnection);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000);     // Advertising interval in units of milliseconds
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (const uint8_t *)BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));
    ble.gap().startAdvertising();
    serial_pc.printf("Start advertising\n");
}

int main(void) {
    serial_pc.printf("< Sigma Delta Technologies Inc. >\n\r");

    /* Init sensor */
    serial_pc.printf("Status before init() : ");
    status = ccs811.getStatus(true);        // 0x10 : not ready to working
    
    ccs811.init();
    serial_pc.printf("Status after init() : ");
    status = ccs811.getStatus(true);        // 0x90 : reports everything working
    while(!(status & 0x90));
    serial_pc.printf("\n");
    wait(1);

    /* Init BLE */
    ble_SDTDevice.onEventsToProcess(callbackEventsToProcess);
    ble_SDTDevice.init(callbackBleInitComplete);

    /* Check whether IC is running or not */
    *pDO_led = LED_ON;

    eventQueue.call_every(1000, callbackPeriodicEvent);
    eventQueue.dispatch_forever();

    return 0;
}

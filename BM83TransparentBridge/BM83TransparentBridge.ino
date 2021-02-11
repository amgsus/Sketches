#include <ArduinoBLE.h>

#define EXPECTED_DEVICE_NAME            "MCHP BM83"
#define TRANSPARENT_UART_SERVICE_UUID   "49535343-fe7d-4ae5-8fa9-9fafd205e455"
#define BTM_TRANSMITTER_CHARACTERISTIC  "49535343-1e4d-4bd9-ba61-23c647249616"
#define BTM_RECEIVER_CHARACTERISTIC     "49535343-8841-43f4-a8d4-ecbe34729bb3"

#define LED_PIN 13

#define BTM_TX_BUFFER_SIZE 32
#define BTM_RX_BUFFER_SIZE 32

typedef enum {
  STA_START_SCANNING = 0,
  STA_SCANNING,
  STA_CONNECTING,
  STA_DISCOVERING,
  STA_READING_CHARACTERISTICS,
  STA_BRIDGING,
  STA_ERROR
} BRIDGE_STATE;

static BRIDGE_STATE      __state = STA_START_SCANNING;
static BLEDevice         __connectedDevice;
static BLECharacteristic __c_btmTx;
static BLECharacteristic __c_btmRx;
static byte              __btmTxBuffer[BTM_TX_BUFFER_SIZE];
static byte              __btmRxBuffer[BTM_RX_BUFFER_SIZE];

void 
setup(void) 
{
  Serial.begin(115200);
  while (!Serial);
  BLE.begin();
}

void 
loop(void) 
{
  switch (__state) {
    case STA_START_SCANNING:
      Serial.print("> Starting scan for peripheral advertised service (UUID ");
      Serial.print(TRANSPARENT_UART_SERVICE_UUID);
      Serial.print(")... ");
      if (BLE.scanForUuid(TRANSPARENT_UART_SERVICE_UUID)) {
        __state = STA_SCANNING;
        Serial.println("OK!");
      } else {
        Serial.println("Failed!");
        __state = STA_ERROR;
      }
      break;

    case STA_SCANNING:
      __connectedDevice = BLE.available();
      if (__connectedDevice) {
        Serial.print("> Found ");
        Serial.print(__connectedDevice.address());
        Serial.print(" '");
        Serial.print(__connectedDevice.localName());
        Serial.print("' UUID: ");
        Serial.print(__connectedDevice.advertisedServiceUuid());
        Serial.println();
        if (__connectedDevice.localName() != EXPECTED_DEVICE_NAME) {
          Serial.println("> Device name does not match the expected one! Scanning next.");
          break;
        }
        BLE.stopScan();
        __state = STA_CONNECTING;
      }
      break;

    case STA_CONNECTING:
      Serial.print("> Scanning is stopped. Connecting... ");
      if (__connectedDevice.connect()) {
        __state = STA_DISCOVERING;
        Serial.println("OK!");
      } else {
        Serial.println("Failed!");
        __state = STA_ERROR;
      }
      break;

    case STA_DISCOVERING:
      Serial.print("> Discovering attributes... ");
      if (__connectedDevice.discoverAttributes()) {
        Serial.println("OK!");
        __state = STA_READING_CHARACTERISTICS;
      } else {
        Serial.println("Failed!");
        __state = STA_ERROR;
      }
      break;

    case STA_READING_CHARACTERISTICS:
      __c_btmTx = __connectedDevice.characteristic(BTM_TRANSMITTER_CHARACTERISTIC);
      __c_btmRx = __connectedDevice.characteristic(BTM_RECEIVER_CHARACTERISTIC);

      if (!__c_btmTx) {
        __state = STA_ERROR;
        Serial.print("> ERROR: Peripheral does not have desired characteristic (");
        Serial.print(BTM_TRANSMITTER_CHARACTERISTIC);
        Serial.println(")!");
      } else if (!__c_btmTx.canSubscribe()) {
        __state = STA_ERROR;
        Serial.print("> ERROR: Characteristic (");
        Serial.print(BTM_TRANSMITTER_CHARACTERISTIC);
        Serial.println(") is not subscribable!");
      }

      if (!__c_btmRx) {
        __state = STA_ERROR;
        Serial.print("> ERROR: Peripheral does not have desired characteristic (");
        Serial.print(BTM_RECEIVER_CHARACTERISTIC);
        Serial.println(")!");
      } else if (!__c_btmRx.canWrite()) {
        __state = STA_ERROR;
        Serial.print("> ERROR: Characteristic (");
        Serial.print(BTM_RECEIVER_CHARACTERISTIC);
        Serial.println(") is not writable!");
      }

      if (__state != STA_ERROR) {
        Serial.println("> Entering the bridge mode: now all serial data will be sent/received to/from BTM.");
        if (!__c_btmTx.subscribe()) {
          Serial.print("> ERROR: Failed to subscribe to BTM transmitter characteristic (");
          Serial.print(BTM_TRANSMITTER_CHARACTERISTIC);
          Serial.println(")!");
          __state = STA_ERROR;
        } else {
          __state = STA_BRIDGING;  
        }
      }
      break;

    case STA_BRIDGING:
      if (!__connectedDevice.connected()) {
        Serial.println("> Device disconnected!");
        Serial.println();
        __state = STA_START_SCANNING;
        delay(3000);
        break;
      }
      if (__c_btmTx.valueUpdated()) {
        int txSize = __c_btmTx.readValue(__btmTxBuffer, BTM_TX_BUFFER_SIZE);
        if (txSize > 0) {
          Serial.write(__btmTxBuffer, txSize);
        }
      }
      if (Serial.available()) {
        int rxSize = Serial.readBytes((char*)__btmRxBuffer, BTM_RX_BUFFER_SIZE);
        if (rxSize > 0) {
          __c_btmRx.writeValue(__btmRxBuffer, rxSize);  
        }
      }
      break;

    case STA_ERROR:
      if (__connectedDevice) {
        Serial.println("> Disconnecting...");
        Serial.println();
        __connectedDevice.disconnect();
      }
      __state = STA_START_SCANNING;
      delay(3000);
      break;
  }
}

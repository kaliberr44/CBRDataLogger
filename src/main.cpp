#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

const char* DATALOGGER_SERVICE_NAME = "CBR DataLogger";
const char* BLE_SERVICE_UUID = "1ff8";
const char* BLE_CHARACTERISTIC_MAIN_UUID = "0001";
const char* BLE_CHARACTERISTIC_FILTER_UUID = "0002";

const int MSG_BYTE_LENGTH_PID = 4;
const int MSG_BYTE_LENGTH_DATA = 16;
const int SERIAL_BAUD_RATE = 9600;

int rpm = 0;

BLEService dataLoggerService = BLEService(BLE_SERVICE_UUID);
BLECharacteristic canBusMainCharacteristic = BLECharacteristic(BLE_CHARACTERISTIC_MAIN_UUID, BLENotify | BLERead, MSG_BYTE_LENGTH_PID + MSG_BYTE_LENGTH_DATA, true);
BLECharacteristic canBusFilterCharacteristic = BLECharacteristic(BLE_CHARACTERISTIC_FILTER_UUID, BLEWrite, 256, false);

#pragma pack(push)
#pragma pack(4)
typedef struct
{
    uint32_t id;
    uint64_t dataHalf1;
    uint64_t dataHalf2;
} raceChronoMsg;
#pragma pack(pop)

void startBLE()
{
    if (!BLE.begin())
    {
        Serial.println("Starting BLE failed!");
        while (1)
            ;
    }
}

void startIMU()
{
    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }
}

void onRxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic)
{
    // central wrote new value to characteristic, update LED
    Serial.println("Characteristic event, read: ");
    uint8_t tmp[256];
    int dataLength = canBusFilterCharacteristic.readValue(tmp, 256);

    for (int i = 0; i < dataLength; i++)
    {
        Serial.print("Byte[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.println((uint8_t)tmp[i], HEX);
    }

    Serial.print("Value length = ");
    Serial.println(canBusFilterCharacteristic.valueLength());
}

void setupBLE()
{
     // Create BLE service and characteristics.
    BLE.setLocalName(DATALOGGER_SERVICE_NAME);
    BLE.setAdvertisedService(dataLoggerService);
    dataLoggerService.addCharacteristic(canBusMainCharacteristic);
    dataLoggerService.addCharacteristic(canBusFilterCharacteristic);
    BLE.addService(dataLoggerService);

    // Event driven reads.
    canBusFilterCharacteristic.setEventHandler(BLEWritten, onRxCharValueUpdate);

    BLE.advertise();

    // Print out full UUID and MAC address.
    Serial.println("Peripheral advertising info: ");
    Serial.print("Name: ");
    Serial.println(DATALOGGER_SERVICE_NAME);
    Serial.print("MAC: ");
    Serial.println(BLE.address());
    Serial.print("Service UUID: ");
    Serial.println(dataLoggerService.uuid());
    Serial.print("canBusMainCharacteristic UUID: ");
    Serial.println(BLE_CHARACTERISTIC_MAIN_UUID);
    Serial.print("canBusFilterCharacteristic UUID: ");
    Serial.println(BLE_CHARACTERISTIC_FILTER_UUID);

    Serial.println("Bluetooth device active, waiting for connections...");
}

void setup()
{
    // Start USB Serial
    Serial.begin(SERIAL_BAUD_RATE);

    // Ensure serial port is ready
    while (!Serial)
        ;

    // Start BLE
    startBLE();
    // Start IMU
    startIMU();

    // Setup BLE
    setupBLE();
}

void sensorNotifyLatestPacket(uint32_t id)
{

    raceChronoMsg motoMessage;
    motoMessage.id = id;
    motoMessage.dataHalf1 = 0;
    motoMessage.dataHalf2 = 0;

    // Temperature
    int temp = 25;

    // Brake postion
    int brakePos = 400;
    brakePos = map(brakePos, 100, 560, 0, 255); // map to calibration
    brakePos = constrain(brakePos, 0, 255);     // constrain to percentage range

    // TPS
    int throttlePos = 70;
    throttlePos = map(throttlePos, 70, 720, 0, 255); // map to calibration
    throttlePos = constrain(throttlePos, 0, 255);    // constrain to percentage range

    // // RPM
    // if(rpm>=16000){
    //     rpm = 0;
    // }
    // rpm++;
    rpm = 12000;
    rpm = constrain(rpm, 0, 16000); // constrain to percentage range

    int accelx = 0;
    int accely = 0;
    int accelz = 0;
    // if (IMU.accelerationAvailable())
    // {
    //   IMU.readAcceleration(accelx,accely,accelz);
    //   accelx = map(accelx, -16, +16, -127, +127);
    //   accely = map(accely, -16, +16, -127, +127);
    //   accelz = map(accelz, -16, +16, -127, +127);
    //   accelx = constrain(accelx, -127, +127);
    //   accely = constrain(accely, -127, +127);
    //   accelz = constrain(accelz, -127, +127);
    // }

    motoMessage.dataHalf1 |= ((uint64_t)(temp & 0xFF)) & 0x00000000000000FF;
    motoMessage.dataHalf1 |= ((uint64_t)(brakePos & 0xFF) << 8) & 0x000000000000FF00;
    motoMessage.dataHalf1 |= ((uint64_t)(throttlePos & 0xFF) << 16) & 0x0000000000FF0000;
    motoMessage.dataHalf1 |= ((uint64_t)(rpm & 0xFFFF) << 24) & 0x000000FFFF000000;
    motoMessage.dataHalf1 |= ((uint64_t)(accelx & 0xFF) << 40) & 0x0000FF0000000000;
    motoMessage.dataHalf1 |= ((uint64_t)(accely & 0xFF) << 48) & 0x00FF000000000000;
    motoMessage.dataHalf1 |= ((uint64_t)(accelz & 0xFF) << 56) & 0xFF00000000000000;
    motoMessage.dataHalf2 |= ((uint64_t)(temp & 0xFF)) & 0x00000000000000FF;
    motoMessage.dataHalf2 |= ((uint64_t)(brakePos & 0xFF) << 8) & 0x000000000000FF00;
    motoMessage.dataHalf2 |= ((uint64_t)(throttlePos & 0xFF) << 16) & 0x0000000000FF0000;
    motoMessage.dataHalf2 |= ((uint64_t)(rpm & 0xFFFF) << 24) & 0x000000FFFF000000;
    motoMessage.dataHalf2 |= ((uint64_t)(accelx & 0xFF) << 40) & 0x0000FF0000000000;
    motoMessage.dataHalf2 |= ((uint64_t)(accely & 0xFF) << 48) & 0x00FF000000000000;
    motoMessage.dataHalf2 |= ((uint64_t)(accelz & 0xFF) << 56) & 0xFF00000000000000;

    // int motoMessageSize = sizeof(motoMessage);

    // uint8_t *motoPtr = (uint8_t *)&motoMessage;

    // for(int i=0;i<motoMessageSize;i++)
    // {
    //     Serial.print("Byte[");
    //     Serial.print(i);
    //     Serial.print("] = ");
    //     Serial.println(*(motoPtr+i),HEX);
    // }

    // Notify
    canBusMainCharacteristic.writeValue((uint8_t *)&motoMessage, sizeof(motoMessage));
}

void loop()
{
    BLEDevice central = BLE.central();
    if (central)
    {
        // Only send data if we are connected to a central device.
        while (central.connected())
        {
            sensorNotifyLatestPacket(0x000000C8); // Decimal PID=200
        }
    }
}
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

const char *DATALOGGER_SERVICE_NAME = "CBR DataLogger";
const char *BLE_SERVICE_UUID = "1ff8";
const char *BLE_CHARACTERISTIC_MAIN_UUID = "0001";
const char *BLE_CHARACTERISTIC_FILTER_UUID = "0002";

const uint8_t MSG_BYTE_LENGTH_PID = 4;
const uint8_t MSG_BYTE_LENGTH_DATA = 16;
const uint8_t RESPONSE_BUFFER_SIZE = 20;
const uint32_t SERIAL_DEBUG_BAUD_RATE = 115200;
const uint16_t SERIAL_KLINE_BAUD_RATE = 10400;
const uint16_t TIMEOUT_MS = 1000;
const uint8_t REFRESH_SPEED_HZ = 10;
const uint8_t DELAY_AFTER_SEND_MS = 1000 / REFRESH_SPEED_HZ;

#define SERIAL_DEBUG Serial  //Serial USB for monitor
#define SERIAL_KLINE Serial1 //Hardware serial for K-Line
#define TX_PIN SERIAL1_TX    // K-Line input
#define RX_PIN SERIAL1_RX    // K-Line output
#define BOARD_LED PIN_LED    // Board in build LED

#pragma pack(push)
#pragma pack(4)
typedef struct
{
    uint32_t id = 200;         // little endian PID
    uint16_t rpm = 0;          // bytesToUintLe(raw,0,2)
    uint8_t tps = 0;           // bytesToUint(raw,2,1) * 5 / 256
    uint8_t batt = 0x7A;       // bytesToUint(raw,3,1) / 10
    uint8_t nceg = 0b00001000; // Gear = bitsToUint(raw,37,3), Neutral/Clutch = bitsToUint(raw,36,1), Engine On = bitsToUint(raw,35,1)
    uint8_t dataB06 = 0;
    uint8_t dataB07 = 0;
    uint8_t dataB08 = 0;
    uint8_t dataB09 = 0;
    uint8_t dataB10 = 0;
    uint8_t dataB11 = 0;
    uint8_t dataB12 = 0;
    uint8_t dataB13 = 0;
    uint8_t dataB14 = 0;
    uint8_t dataB15 = 0;
    uint8_t dataB16 = 0;
} RaceChronoMsg;
#pragma pack(pop)

// Bluetooth settings
BLEService dataLoggerService = BLEService(BLE_SERVICE_UUID);
BLECharacteristic canBusMainCharacteristic = BLECharacteristic(BLE_CHARACTERISTIC_MAIN_UUID, BLENotify | BLERead, MSG_BYTE_LENGTH_PID + MSG_BYTE_LENGTH_DATA, true);
BLECharacteristic canBusFilterCharacteristic = BLECharacteristic(BLE_CHARACTERISTIC_FILTER_UUID, BLEWrite, 256, false);

// Diffrent connection states
enum ECUConnectionState
{
    UNKNOWN,
    KLINE_STATE_SET,
    INITIALIZED
};

ECUConnectionState ECUConnection = UNKNOWN;

uint8_t ECUInitMsgPart1[] = {0xFE, 0x04, 0xFF, 0xFF};
// ECU: NO RESPONSE
uint8_t ECUInitMsgPart2[] = {0x72, 0x05, 0x00, 0xF0, 0x99};
// ECU: 0x02 ,0x04, 0x00, 0xFA
uint8_t ECUInitResponse[] = {0x02, 0x04, 0x00, 0xFA};
// ECU: Initialised Response, no Decode.

uint8_t ECURequestMsgDataTable11[] = {0x72, 0x07, 0x72, 0x11, 0x00, 0x0E, 0xF6};
// Get as far as Speed - ignore rest of memory bank
// OUT: CMD,  LEN,  QUR,  TABN, BEG,  END,  CHK
// OUT: 0x72, 0x07, 0x72, 0x11, 0x00, 0x0E, 0xF6
// ECU: CMD,  LEN,  QUR,  TABN, STR,  RPMH, RPML, TPSV, TPS%, ECTV, ECTC, IATH, IATL, MAPH, MAPL, ????, ????, BAT,  MPH,  CHK
// ECU: 0x02, 0x14, 0x72, 0x11, 0x00, 0x05, 0x5D, 0x19, 0x00, 0x2A, 0x7B, 0xAF, 0x36, 0x68, 0x45, 0xFF, 0xFF, 0x8B, 0x14, 0x18

uint8_t ECURequestMsgDataTableD1[] = {0x72, 0x07, 0x72, 0xD1, 0x00, 0x05, 0x3F};
// Get Gear Engaged + Engine Running
// OUT: CMD,  LEN,  QUR,  TABN, BEG,  END,  CHK
// OUT: 0x72, 0x07, 0x72, 0xD1, 0x00, 0x05, 0x3F
// ECUL CMD,  LEN,  QUR,  TABN, STR,  NEUT, ????, ????, ????, ENG,  CHK
// ECU: 0x02, 0x0B, 0x72, 0xD1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xAF

uint8_t GearRatios[] = {125, 91, 76, 66, 59, 55};
uint8_t Response[RESPONSE_BUFFER_SIZE]; // Buffer for response from ECU
RaceChronoMsg RaceChronoDataMsg;        // Last RC data message if we get no or bad response data from ECU
uint8_t lastGear = 0;                   // Last gear value for when downshifting with clutch pulled in

void debugByteData(char const *msg, uint8_t *byteBuffer, uint8_t bufferLen);
void debugSingleByte(uint8_t singleByte);

void startSerialDebug();

void startSerialKLine();
void ecuInitBusAndTransceiver();
bool ecuSendInitSequence();
void ecuSendData(uint8_t *byteBuffer, uint8_t bufferLen);
uint8_t ecuGetResponse(uint8_t *response);
bool ecuGetDataTable(uint8_t *tableMsg, uint8_t tableMsgLen);
bool checkChecksum(uint8_t checksum, uint8_t *data, uint8_t dataLength);

uint8_t calculateGear(uint16_t rpm, uint8_t speed, bool clutchOrNeutral);

void startBLE();
void setupBLE();
void bleFilterCharacteristicCallback(BLEDevice central, BLECharacteristic characteristic);
void sensorNotifyLatestPacket(uint8_t rpmh, uint8_t rpml, uint8_t tps, uint8_t batt, uint8_t nck, uint8_t eng);

void startIMU();

void startSerialDebug()
{
    SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD_RATE);
    while (!SERIAL_DEBUG)
        ;
}

void startSerialKLine()
{
    SERIAL_KLINE.begin(SERIAL_KLINE_BAUD_RATE);
    while (!SERIAL_KLINE)
        ;
}

void startBLE()
{
    if (!BLE.begin())
    {
        SERIAL_DEBUG.println("Starting BLE failed!");
        while (1)
            ;
    }
}

void startIMU()
{
    if (!IMU.begin())
    {
        SERIAL_DEBUG.println("Failed to initialize IMU!");
        while (1)
            ;
    }
}

void debugByteData(char const *msg, uint8_t *byteBuffer, uint8_t bufferLen)
{
    SERIAL_DEBUG.println(msg);
    SERIAL_DEBUG.print(" - BEGGIN: ");
    for (uint8_t i = 0; i < bufferLen; i++)
    {
        debugSingleByte(*(byteBuffer + i));
        if (i != bufferLen - 1)
        {
            SERIAL_DEBUG.print(", ");
        }
    }
    SERIAL_DEBUG.print(" END");
}

void debugSingleByte(uint8_t singleByte)
{
    if (singleByte < 16)
    {
        SERIAL_DEBUG.print("0"); // add leading 0 for single digit bytes
    }
    SERIAL_DEBUG.print(singleByte, HEX);
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
    canBusFilterCharacteristic.setEventHandler(BLEWritten, bleFilterCharacteristicCallback);

    BLE.advertise();

    // Print out full UUID and MAC address.
    SERIAL_DEBUG.println("Peripheral advertising info: ");
    SERIAL_DEBUG.print("Name: ");
    SERIAL_DEBUG.println(DATALOGGER_SERVICE_NAME);
    SERIAL_DEBUG.print("MAC: ");
    SERIAL_DEBUG.println(BLE.address());
    SERIAL_DEBUG.print("Service UUID: ");
    SERIAL_DEBUG.println(dataLoggerService.uuid());
    SERIAL_DEBUG.print("canBusMainCharacteristic UUID: ");
    SERIAL_DEBUG.println(canBusMainCharacteristic.uuid());
    SERIAL_DEBUG.print("canBusFilterCharacteristic UUID: ");
    SERIAL_DEBUG.println(canBusFilterCharacteristic.uuid());

    SERIAL_DEBUG.println("Bluetooth device active, waiting for connections...");
}

void bleFilterCharacteristicCallback(BLEDevice central, BLECharacteristic characteristic)
{
    // central wrote new value to characteristic, update LED
    SERIAL_DEBUG.println("Characteristic event, read: ");
    uint8_t tmp[256];
    int dataLength = canBusFilterCharacteristic.readValue(tmp, 256);

    for (int i = 0; i < dataLength; i++)
    {
        SERIAL_DEBUG.print("Byte[");
        SERIAL_DEBUG.print(i);
        SERIAL_DEBUG.print("] = ");
        SERIAL_DEBUG.println((uint8_t)tmp[i], HEX);
    }

    SERIAL_DEBUG.print("Value length = ");
    SERIAL_DEBUG.println(canBusFilterCharacteristic.valueLength());
}

// Enable Honda ECU bus - ensure Serial not enabled!
void ecuInitBusAndTransceiver()
{
    SERIAL_KLINE.end();
    SERIAL_DEBUG.println("K-Line Serial disabled");
    delay(350);

    digitalWrite(TX_PIN, LOW);
    delay(70);
    digitalWrite(TX_PIN, HIGH);
    delay(120);
    pinMode(TX_PIN, OUTPUT);
    SERIAL_DEBUG.println("K-Line state LOW(70) -> HIGH(120) change done");

    SERIAL_KLINE.begin(SERIAL_KLINE_BAUD_RATE);
    SERIAL_DEBUG.println("K-Line Serial enabled");

    // No way to check if it worked !!!
}

bool ecuSendInitSequence()
{
    //   Send first init string
    SERIAL_DEBUG.println("Sending init sequence");
    SERIAL_DEBUG.println("Sending first part of init message sent");
    ecuSendData(ECUInitMsgPart1, sizeof(ECUInitMsgPart1));
    ecuGetResponse(Response);

    SERIAL_DEBUG.println("Sending second part init message sent");
    ecuSendData(ECUInitMsgPart2, sizeof(ECUInitMsgPart2));
    uint8_t responseLen = ecuGetResponse(Response);

    if (responseLen == sizeof(ECUInitResponse) && checkChecksum(ECUInitResponse[sizeof(ECUInitResponse) - 1], Response, responseLen))
    {
        SERIAL_DEBUG.println("Init succeeded");
        return true;
    }

    SERIAL_DEBUG.println("Init failed");
    SERIAL_DEBUG.println("Response size: ");
    SERIAL_DEBUG.print(responseLen);
    if (responseLen > 0)
    {
        SERIAL_DEBUG.print(". Response checksum: ");
        SERIAL_DEBUG.print(Response[responseLen - 1]);
    }
    return false;
}

void ecuSendData(uint8_t *byteBuffer, uint8_t bufferLen)
{
    debugByteData("Data to send", byteBuffer, bufferLen);
    for (uint8_t i = 0; i < bufferLen; i++)
    {
        SERIAL_KLINE.write(*(byteBuffer + i));
    }
    SERIAL_KLINE.flush(); //Block Arduino until all data sended

    ulong waitTimeMs = millis();
    while (!SERIAL_KLINE.available() && (millis() - waitTimeMs < DELAY_AFTER_SEND_MS))
    {
        waitTimeMs = millis();
    }
    SERIAL_DEBUG.println("Sent echo ");
    SERIAL_DEBUG.print(" - BEGGIN: ");
    for (uint8_t i = 0; i < bufferLen; i++)
    {
        if (SERIAL_KLINE.available())
        {
            debugSingleByte(SERIAL_KLINE.read());
        
            if (i != bufferLen - 1)
            {
                SERIAL_DEBUG.print(", ");
            }
        }
    }
    SERIAL_DEBUG.print(" END");
}

uint8_t ecuGetResponse(uint8_t *response)
{
    memset(response, 0, RESPONSE_BUFFER_SIZE);
    ulong waitTimeMs = millis();
    while (!SERIAL_KLINE.available() && (millis() - waitTimeMs < DELAY_AFTER_SEND_MS))
    {
        waitTimeMs = millis();
    }

    uint8_t index = 0;
    while (SERIAL_KLINE.available() && index < RESPONSE_BUFFER_SIZE)
    {
        uint8_t dataByte = SERIAL_KLINE.read();

        response[index] = dataByte;

        index++;
    }

    debugByteData("Response data", response, index);

    return index;
}

bool ecuGetDataTable(uint8_t *tableMsg, uint8_t tableMsgLen)
{
    SERIAL_DEBUG.println("Sending request for data table: ");
    SERIAL_DEBUG.print(tableMsg[3]);

    // Expected data length = actual data length (end register - start register) + cmd + len + qur + tabn + str + chk (6 const msg bytes)
    uint8_t expectedLen = tableMsg[5] - tableMsg[4] + 6;

    ecuSendData(tableMsg, tableMsgLen);
    uint8_t responseLen = ecuGetResponse(Response);

    if (responseLen == expectedLen && checkChecksum(Response[responseLen - 1], Response, responseLen))
    {
        SERIAL_DEBUG.println("Getting data table succeeded");
        return true;
    }

    SERIAL_DEBUG.println("Getting data table failed!");
    SERIAL_DEBUG.println("Response size: ");
    SERIAL_DEBUG.print(responseLen);
    if (responseLen > 0)
    {
        SERIAL_DEBUG.print(". Response checksum: ");
        SERIAL_DEBUG.print(Response[responseLen - 1]);
    }
    return false;
}

bool checkChecksum(uint8_t checksum, uint8_t *data, uint8_t dataLength)
{
    uint8_t calculatedChecksum = 0;

    for (uint8_t i = 0; i < dataLength - 1; i++)
    {
        calculatedChecksum += data[i];
    }
    calculatedChecksum = (uint8_t)(calculatedChecksum ^ 0xFF) + 1;

    return checksum == calculatedChecksum;
}

void sensorNotifyLatestPacket(uint8_t rpmh, uint8_t rpml, uint8_t speed, uint8_t tps, uint8_t batt, uint8_t nck, uint8_t eng)
{
    uint16_t rpm = (uint16_t)(rpmh << 8) | rpml;
    uint8_t gear = (eng) ? calculateGear(rpm, speed, nck) : 0;


    RaceChronoDataMsg.rpm = rpm;
    RaceChronoDataMsg.tps = tps;
    RaceChronoDataMsg.batt = batt;
    RaceChronoDataMsg.nceg = eng << 4 | (nck & 0x01 << 3) | gear; 

    SERIAL_DEBUG.println("Real values:");
    SERIAL_DEBUG.print(" RPM = ");
    SERIAL_DEBUG.print(rpm);
    SERIAL_DEBUG.print(" TPS% = ");
    SERIAL_DEBUG.print(tps * 5 / 256);
    SERIAL_DEBUG.print(" BATT[V] = ");
    SERIAL_DEBUG.print(batt / 10);
    SERIAL_DEBUG.print(" GEAR = ");
    SERIAL_DEBUG.print(gear);
    SERIAL_DEBUG.print(" NEUTRAL/CLUTCH = ");
    SERIAL_DEBUG.print((nck) ? "ON" : "OFF");
    SERIAL_DEBUG.print(" ENGINE = ");
    SERIAL_DEBUG.print((eng) ? "ON" : "OFF");


    // Notify
    canBusMainCharacteristic.writeValue((uint8_t *)&RaceChronoDataMsg, sizeof(RaceChronoDataMsg));
}

uint8_t calculateGear(uint16_t rpm, uint8_t speed, bool clutchOrNeutral)
{
    if (clutchOrNeutral)
    {
        if (speed < 5)
        {
            return 0;
        }
        else
        {
            return lastGear;
        }
    }

    uint8_t currentRatio = rpm / speed;
    uint8_t maxGear = sizeof(GearRatios);
    uint8_t lastRatioDelta = 255;

    for (uint8_t i = 0; i < maxGear; i++)
    {
        uint8_t delta = abs(GearRatios[i] - currentRatio);
        if (delta > lastRatioDelta)
        {
            return i;
        }
        lastRatioDelta = delta;
    }
    return maxGear;
}

void setup()
{
    startSerialDebug();

    startSerialKLine();

    startIMU();

    startBLE();
    setupBLE();
}

void loop()
{
    if (ECUConnection == UNKNOWN)
    {
        ecuInitBusAndTransceiver();
        ECUConnection = KLINE_STATE_SET;
    }
    if (ECUConnection == KLINE_STATE_SET)
    {
        ECUConnection = (ecuSendInitSequence()) ? INITIALIZED : UNKNOWN;
    }
    if (ECUConnection == INITIALIZED)
    {
        ECUConnection = (ecuGetDataTable(ECURequestMsgDataTable11, sizeof(ECURequestMsgDataTable11))) ? INITIALIZED : UNKNOWN;
    }
}
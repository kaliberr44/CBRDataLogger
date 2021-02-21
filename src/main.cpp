#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

const char *DATALOGGER_SERVICE_NAME = "CBR DataLogger";
const char *BLE_SERVICE_UUID = "1FF8";
const char *BLE_CHARACTERISTIC_MAIN_UUID = "0001";
const char *BLE_CHARACTERISTIC_FILTER_UUID = "0002";
const uint8_t MSG_BYTE_LENGTH_PID = 4;                  // RaceChrono message pid size
const uint8_t MSG_BYTE_LENGTH_DATA = 16;                // RaceChrono message data size

const uint8_t RESPONSE_BUFFER_SIZE = 20;
const uint8_t GET_DATA_FAILED_ATTEMPS = 10;

const uint32_t SERIAL_DEBUG_BAUD_RATE = 115200;
const uint32_t SERIAL_KLINE_BAUD_RATE = 10400;
const ulong TIME_FOR_ECU_TO_START_MS = 1000;
const ulong TIMEOUT_BETWEN_BYTES_MS = 20;               // K-Line specs allow up to 20 ms between next bytes in response
const ulong REFRESH_SPEED_HZ = 25;
const ulong DELAY_AFTER_SEND_MS = 1000 / REFRESH_SPEED_HZ;

/*
    UART0 Base address 0x40002000
    UART Baudrate register address offset 0x524

    Baudrate formula according Nordic devzone

    Baudrate = desired baudrate * 2^32 / 16000000

    Baudrate = 10400 * 4294967296 / 16000000
    Baudrate = 2791729 = 0x2A9931 

    Rounded by 0x001000
    Rounded to 0x2A9000 gives 10391 baud 
    Rounded to 0x2AA000 gives 10406 baud 
*/

#define UARTE0_BASE_ADDR 0x40002000    // As per nRF52840 Product spec - UARTE
#define UART_BAUDRATE_REG_OFFSET 0x524 // As per nRF52840 Product spec - UARTE

#define UART0_BAUDRATE_REGISTER (*((uint32_t *)(UARTE0_BASE_ADDR + UART_BAUDRATE_REG_OFFSET)))

#define BAUD10391 0x2A9000
#define BAUD10406 0x2AA000

#define SERIAL_DEBUG Serial    // Serial USB for monitor
#define SERIAL_KLINE Serial1   // Hardware serial for K-Line
#define K_LINE_PULLDOWN_PIN D2 // K-Line input
#define BOARD_LED PIN_LED      // Board in build LED

#pragma pack(push)
#pragma pack(4)
typedef struct
{
    uint32_t id = 200;         // little endian PID
    uint16_t rpm = 0;          // bytesToUintLe(raw,0,2)
    uint8_t tps = 0;           // bytesToUint(raw,2,1) / 1.6
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

typedef struct
{
    uint8_t speed = 0;
} NonRaceChronoPacket;

// Bluetooth settings
BLEService dataLoggerService = BLEService(BLE_SERVICE_UUID);
BLECharacteristic canBusMainCharacteristic = BLECharacteristic(BLE_CHARACTERISTIC_MAIN_UUID, BLENotify | BLERead, MSG_BYTE_LENGTH_PID + MSG_BYTE_LENGTH_DATA, true);
BLECharacteristic canBusFilterCharacteristic = BLECharacteristic(BLE_CHARACTERISTIC_FILTER_UUID, BLEWrite, 256, false);

// Diffrent connection states
enum ECUConnectionState
{
    DEBUG_MODE,
    UNKNOWN,
    KLINE_STATE_SET,
    INITIALIZED
};

ECUConnectionState ECUConnection = DEBUG_MODE;

// uint8_t ECUWakeUpMsg[] = {0xFE, 0x04, 0xFF, 0xFF}; ECU: NO Response. Wait 50 ms
uint8_t ECUWakeUpMsg[] = {0xFE, 0x04, 0x72, 0x8C};
// ECU: 0x0E, 0x04, 0x72, 0x7C
uint8_t ECUWakeUpResponse[] = {0x0E, 0x04, 0x72, 0x7C};


/* maybe not necessary - test
uint8_t ECUInitMsg[] = {0x72, 0x05, 0x00, 0xF0, 0x99};
// ECU: 0x02 ,0x04, 0x00, 0xFA
uint8_t ECUInitResponse[] = {0x02, 0x04, 0x00, 0xFA};
*/

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

uint8_t GearRatios[] = {125, 91, 76, 66, 59, 55}; // RPM/SPD Calculated from https://www.gearingcommander.com/ - latter on we will use real values

uint8_t Response[RESPONSE_BUFFER_SIZE];      // Buffer for response from ECU
RaceChronoMsg RaceChronoDataMsg;             // Last RC data message if we get no or bad response data from ECU
NonRaceChronoPacket NonRaceChronoDataPacket; // Last non RC buffer data if we get no or bad response data from ECU

uint8_t getDataFailedAttempts = 0;


// Debug
void startSerialDebug();
void debugByteData(char const *msg, uint8_t *byteBuffer, uint8_t bufferLen);
void debugSingleByte(uint8_t singleByte);
void debugMode();

// K-Line
void startSerialKLine();
void clearSerialBuffer();
bool serialBufferTimeout(ulong timeout);
void ecuSendData(uint8_t *byteBuffer, uint8_t bufferLen);
uint8_t ecuGetResponse(uint8_t *response, uint8_t expectedRespLen);
bool validateResponse(char const *failMsg, uint8_t firstByte, uint8_t *response, uint8_t responseLen, uint8_t expectedLen);
bool checkChecksum(uint8_t checksum, uint8_t *data, uint8_t dataLength);
void ecuInitBusAndTransceiver();
bool ecuSendInitSequence();
bool ecuGetDataAndFillPacket();
bool ecuGetDataTable(uint8_t *tableMsg, uint8_t tableMsgLen);
uint8_t calculateGear(uint16_t rpm, uint8_t speed, bool clutchOrNeutral);
void sensorCreateLatestPacket(uint8_t rpmh, uint8_t rpml, uint8_t speed, uint8_t tps, uint8_t batt, uint8_t nck, uint8_t eng);

// BLE
void startBLE();
void setupBLE();
bool bleDeviceConnected();
void notifyBluetoothCharacteristics();
void bleFilterCharacteristicCallback(BLEDevice central, BLECharacteristic characteristic);

// IMU
void startIMU();

void startSerialDebug()
{
    SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD_RATE);
    while (!SERIAL_DEBUG)
        ;
}

void debugByteData(char const *msg, uint8_t *byteBuffer, uint8_t bufferLen)
{
    SERIAL_DEBUG.print(msg);
    SERIAL_DEBUG.print(" - BEGGIN[ ");
    for (uint8_t i = 0; i < bufferLen; i++)
    {
        debugSingleByte(*(byteBuffer + i));
    }
    SERIAL_DEBUG.println("]END");
}

void debugSingleByte(uint8_t singleByte)
{
    SERIAL_DEBUG.print("0x");
    if (singleByte < 16)
    {
        SERIAL_DEBUG.print("0"); // add leading 0 for single digit bytes
    }
    SERIAL_DEBUG.print(singleByte, HEX);
    SERIAL_DEBUG.print(" ");
}

void debugMode()
{
    if (SERIAL_DEBUG.available())
    {
        uint8_t input = SERIAL_DEBUG.read();
        switch (input)
        {
            case 'd':
                ECUConnection = DEBUG_MODE;
                SERIAL_DEBUG.println("Debug mode ON");
                break;
            case 'l':
                ECUConnection = DEBUG_MODE;
                SERIAL_KLINE.end();
                digitalWrite(K_LINE_PULLDOWN_PIN, LOW);
                break;
            case 'h':
                ECUConnection = DEBUG_MODE;
                SERIAL_KLINE.end();
                digitalWrite(K_LINE_PULLDOWN_PIN, HIGH);
                break;
            case 'i':
                ECUConnection = DEBUG_MODE;
                ecuInitBusAndTransceiver();
                ecuSendInitSequence();
                break;
            case 'g':
                ECUConnection = DEBUG_MODE;
                ecuGetDataAndFillPacket();
                break;
            case 'b':
                ECUConnection = DEBUG_MODE;
                notifyBluetoothCharacteristics();
                break;
            case 'a':
                ECUConnection = DEBUG_MODE;
                ecuInitBusAndTransceiver();
                if (ecuSendInitSequence() && ecuGetDataAndFillPacket())
                {   
                    notifyBluetoothCharacteristics();
                }
                break;
            case 'n':
                ECUConnection = UNKNOWN;
                break;
            default:
                ECUConnection = DEBUG_MODE;
                SERIAL_DEBUG.println("Unknown debug command");
                break;
        }
    }
}

// K-Line
void startSerialKLine()
{
    SERIAL_KLINE.begin(SERIAL_KLINE_BAUD_RATE);

    SERIAL_DEBUG.print("Old Baudrate UART0: 0x");
    SERIAL_DEBUG.println(UART0_BAUDRATE_REGISTER, HEX);

    UART0_BAUDRATE_REGISTER = BAUD10406;

    SERIAL_DEBUG.print("New Baudrate UART0: 0x");
    SERIAL_DEBUG.println(UART0_BAUDRATE_REGISTER, HEX);

    while (!SERIAL_KLINE)
        ;
}

void clearSerialBuffer()
{
    SERIAL_DEBUG.print("K-Line Serial buffer");
    SERIAL_DEBUG.print(" - BEGGIN[ ");
    while (SERIAL_KLINE.available())
    {
        debugSingleByte(SERIAL_KLINE.read());
    }
    SERIAL_DEBUG.println("]END");
    SERIAL_KLINE.flush();
    SERIAL_DEBUG.println("K-Line Serial buffer cleared");
}

bool serialBufferTimeout(ulong timeout)
{
    if (SERIAL_KLINE.available())
    {
        return false;
    }

    ulong waitTimeMs = millis();

    while (millis() - waitTimeMs < timeout)
    {
        if (SERIAL_KLINE.available())
        {
            return false;
        }
    }
    SERIAL_DEBUG.print("TIME ");
    return true;
}

void ecuSendData(uint8_t *byteBuffer, uint8_t bufferLen)
{
    debugByteData("Data to send        ", byteBuffer, bufferLen);
    for (uint8_t i = 0; i < bufferLen; i++)
    {
        SERIAL_KLINE.write(*(byteBuffer + i));
    }
    SERIAL_KLINE.flush(); //Block Arduino until all data sended

    delay(DELAY_AFTER_SEND_MS);
    SERIAL_DEBUG.print("Sent echo           ");
    SERIAL_DEBUG.print(" - BEGGIN[ ");
    for (uint8_t i = 0; i < bufferLen; i++)
    {
        if (!serialBufferTimeout(TIMEOUT_BETWEN_BYTES_MS))
        {
            debugSingleByte(SERIAL_KLINE.read());
        }
    }
    SERIAL_DEBUG.println("]END");
}

uint8_t ecuGetResponse(uint8_t *response, uint8_t expectedRespLen)
{
    memset(response, 0, RESPONSE_BUFFER_SIZE);

    uint8_t index = 0;
    uint8_t actualLength = 0;

    SERIAL_DEBUG.print("Response            ");
    SERIAL_DEBUG.print(" - BEGGIN[ ");
    while (index < expectedRespLen)
    {
        if (!serialBufferTimeout(TIMEOUT_BETWEN_BYTES_MS))
        {
            uint8_t dataByte = SERIAL_KLINE.read();
            debugSingleByte(dataByte);
            if (actualLength < RESPONSE_BUFFER_SIZE)
            {
                response[actualLength] = dataByte;
                actualLength++;
            }
        }
        index++;
    }
    SERIAL_DEBUG.println("]END");

    debugByteData("Actual response data", response, actualLength);

    return actualLength;
}

bool validateResponse(char const *failMsg, uint8_t firstByte, uint8_t *response, uint8_t responseLen, uint8_t expectedLen)
{
     if (responseLen == expectedLen && Response[0] == firstByte && checkChecksum(response[responseLen - 1], response, responseLen))
    {
        return true;
    }
    SERIAL_DEBUG.print(failMsg);
    SERIAL_DEBUG.print(" - Response size: ");
    SERIAL_DEBUG.print(responseLen);
    SERIAL_DEBUG.print(" Expected size: ");
    SERIAL_DEBUG.print(expectedLen);
    if (responseLen == expectedLen)
    {
        SERIAL_DEBUG.print(" | Response checksum: ");
        debugSingleByte(Response[responseLen - 1]);
    }
    else
    {
        SERIAL_DEBUG.println("");
    }
    
    return false;
}

bool checkChecksum(uint8_t checksum, uint8_t *data, uint8_t dataLength)
{
    // No overflow - just 2 last sum bytes count
    uint8_t calculatedChecksum = 0;

    for (uint8_t i = 0; i < dataLength - 1; i++)
    {
        calculatedChecksum += data[i];
    }
    calculatedChecksum = (uint8_t)(calculatedChecksum ^ 0xFF) + 1;

    return checksum == calculatedChecksum;
}

// Enable Honda ECU bus - ensure Serial not enabled!
void ecuInitBusAndTransceiver()
{
    clearSerialBuffer();
    SERIAL_KLINE.end();
    SERIAL_DEBUG.println("K-Line Serial disabled");

    digitalWrite(K_LINE_PULLDOWN_PIN, LOW);
    delay(70);
    digitalWrite(K_LINE_PULLDOWN_PIN, HIGH);
    delay(130);
    pinMode(K_LINE_PULLDOWN_PIN, OUTPUT);
    SERIAL_DEBUG.println("K-Line state LOW(70) -> HIGH(130) change done");

    startSerialKLine();
    SERIAL_DEBUG.println("K-Line Serial enabled");

    // No way to check if it worked - no response from ECU!!!
}

bool ecuSendInitSequence()
{
    // Send first init string
    SERIAL_DEBUG.println("INIT sequence");
    SERIAL_DEBUG.println("Sending wake up");
    ecuSendData(ECUWakeUpMsg, sizeof(ECUWakeUpMsg));
    uint8_t responseLen = ecuGetResponse(Response, sizeof(ECUWakeUpResponse));

    return validateResponse("Init failed!        ", ECUWakeUpResponse[0], Response, responseLen, sizeof(ECUWakeUpResponse));
    /*
    SERIAL_DEBUG.println("Waiting 50 ms");
    delay(50);
    SERIAL_DEBUG.println("Sending init message");
    ecuSendData(ECUInitMsg, sizeof(ECUInitMsg));
    uint8_t responseLen = ecuGetResponse(Response, sizeof(ECUInitResponse));

    return validateResponse("Init failed!        ", 0x02, Response, responseLen, sizeof(ECUInitResponse));
    */
}

bool ecuGetDataAndFillPacket()
{
    bool success = false;
    uint8_t rpmh = (RaceChronoDataMsg.rpm & 0xFF00) >> 8;
    uint8_t rpml = RaceChronoDataMsg.rpm & 0x00FF;
    uint8_t speed = NonRaceChronoDataPacket.speed;
    uint8_t tps = RaceChronoDataMsg.tps;
    uint8_t batt = RaceChronoDataMsg.batt;
    uint8_t nck = (RaceChronoDataMsg.nceg & 0b00001000) >> 3;
    uint8_t eng = (RaceChronoDataMsg.nceg & 0b00010000) >> 4;

    if (ecuGetDataTable(ECURequestMsgDataTableD1, sizeof(ECURequestMsgDataTableD1)))
    {
        nck = Response[5];
        eng = Response[9];

        sensorCreateLatestPacket(rpmh, rpml, speed, tps, batt, nck, eng);
        success = true;
    }

    if (ecuGetDataTable(ECURequestMsgDataTable11, sizeof(ECURequestMsgDataTable11)))
    {
        sensorCreateLatestPacket(Response[5], Response[6], Response[18], Response[8], Response[17], nck, eng);
        success = true;
    }

    return success;
}

bool ecuGetDataTable(uint8_t *tableMsg, uint8_t tableMsgLen)
{
    SERIAL_DEBUG.print("Sending request for data table: ");
    debugSingleByte(tableMsg[3]);

    // Expected data length = actual data length (end register - start register) + cmd + len + qur + tabn + str + chk (6 const msg bytes)
    uint8_t expectedLen = tableMsg[5] - tableMsg[4] + 6;

    ecuSendData(tableMsg, tableMsgLen);
    uint8_t responseLen = ecuGetResponse(Response, expectedLen);

    return validateResponse("Getting data failed!", 0x02, Response, responseLen, expectedLen);
}

uint8_t calculateGear(uint16_t rpm, uint8_t speed, bool clutchOrNeutral)
{
    static uint8_t lastGear = 0; // Last gear value for when downshifting with clutch pulled in

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
            lastGear = i;
            return i;
        }
        lastRatioDelta = delta;
    }

    lastGear = maxGear;
    return maxGear;
}

void sensorCreateLatestPacket(uint8_t rpmh, uint8_t rpml, uint8_t speed, uint8_t tps, uint8_t batt, uint8_t nck, uint8_t eng)
{
    uint16_t rpm = (uint16_t)(rpmh << 8) | rpml;
    uint8_t gear = (eng) ? calculateGear(rpm, speed, nck) : 0;

    NonRaceChronoDataPacket.speed = speed;

    RaceChronoDataMsg.rpm = rpm;
    RaceChronoDataMsg.tps = tps;
    RaceChronoDataMsg.batt = batt;
    RaceChronoDataMsg.nceg = eng << 4 | (nck & 0x01 << 3) | gear;

    SERIAL_DEBUG.println("Real values:");
    SERIAL_DEBUG.print("RPM = ");
    SERIAL_DEBUG.print(rpm);
    SERIAL_DEBUG.print(" TPS% = ");
    SERIAL_DEBUG.print(tps / 1.6);
    SERIAL_DEBUG.print(" BATT[V] = ");
    SERIAL_DEBUG.print((float)batt / 10);
    SERIAL_DEBUG.print(" GEAR = ");
    SERIAL_DEBUG.print(gear);
    SERIAL_DEBUG.print(" NEUTRAL/CLUTCH = ");
    SERIAL_DEBUG.print((nck & 0x01) ? "ON" : "OFF");
    SERIAL_DEBUG.print(" ENGINE = ");
    SERIAL_DEBUG.println((eng) ? "ON" : "OFF");
}


// BLE
void startBLE()
{
    if (!BLE.begin())
    {
        SERIAL_DEBUG.println("Starting BLE failed!");
        while (1)
            ;
    }
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

bool bleDeviceConnected()
{
    BLEDevice raceChronoAndroidApp = BLE.central();
    if (raceChronoAndroidApp && raceChronoAndroidApp.connected())
    {
        digitalWrite(BOARD_LED, HIGH);
        return true;
    }
    else
    {
        digitalWrite(BOARD_LED, LOW);
        SERIAL_DEBUG.println("BLE device not connected!");
        return false;
    }
}

void notifyBluetoothCharacteristics()
{
    if (bleDeviceConnected())
    {
        SERIAL_DEBUG.println("Data sent to BLE device");
        canBusMainCharacteristic.writeValue((uint8_t *)&RaceChronoDataMsg, sizeof(RaceChronoDataMsg));
    }
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


// IMU
void startIMU()
{
    if (!IMU.begin())
    {
        SERIAL_DEBUG.println("Failed to initialize IMU!");
        while (1)
            ;
    }
}


// Setup and Loop
void setup()
{
    startSerialDebug();

    if(ECUConnection == DEBUG_MODE)
    {
        SERIAL_DEBUG.println("Debug mode ON");
    }

    startSerialKLine();

    startIMU();

    startBLE();
    setupBLE();

    delay(TIME_FOR_ECU_TO_START_MS);
}

void loop()
{
    debugMode();

    if (ECUConnection == UNKNOWN)
    {
        getDataFailedAttempts = 0;
        ecuInitBusAndTransceiver();
        ECUConnection = KLINE_STATE_SET;
    }
    if (ECUConnection == KLINE_STATE_SET)
    {
        ECUConnection = (ecuSendInitSequence()) ? INITIALIZED : UNKNOWN;
    }
    if (ECUConnection == INITIALIZED)
    {
        ECUConnection = INITIALIZED;
        if (ecuGetDataAndFillPacket())
        {
            notifyBluetoothCharacteristics();
        }
        else
        {
            getDataFailedAttempts++;
            if (getDataFailedAttempts >= GET_DATA_FAILED_ATTEMPS)
            {
                ECUConnection = UNKNOWN;
            }
        }
    }
}
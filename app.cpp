#include "Arduino.h"
/*
 * DO NOT TOUCH THIS. It is arithmetic operation constants
 */
#define ANGLE_RANGE_MULTIPLIER_MSB_MASK (uint16_t) 0b0000000000111111
#define ANGLE_RANGE_MULTIPLIER_LSB_MASK (uint16_t) 0b0001111111111111
#define CLAMP_LO_MASK                   (uint16_t) 0b0001111111111111
#define CLAMP_HI_MASK                   (uint16_t) 0b0001111111111111

/*
 * DO NOT TOUCH THIS. It is arithmetic operation constants
 */
#define MECHANICAL_ZERO_ANGLE_POINT         16
#define ANGLE_RANGE_MULTIPLIER_MSB_SHIFT    13
#define ANGLE_RANGE_MULTIPLIER_POINT        14
#define CLAMP_SW_ANGLE_SHIFT                6

/*
 * Adjust only in case of issues with reading/writing KMA210 (refer to data sheet for details)
 */
#define T_TURN_ON   1       // MAX 5 ms
#define T_START     10      // MIN 5 ns
#define T_STOP      10      // MIN 5 ns
#define T_BIT       60     // 10 to 100 ns
#define T_ZERO      15      // 0.175 to 0.375 T_BIT (typ 0.25)
#define T_ONE       45      // 0.625 to 0.825 T_BIT (typ 0.75)
#define T_TAKEOVER  25      // 0 to 0.5 T_BIT
#define T_PROG      20      // MIN 20 ms

/*
 * Set pins according to your wiring scheme
 */
#define VDD_PIN     10
#define DATA_PIN    18

/*
 * Those values are only for initial preset setup. You are free to adjust them
 */
#define DEFAULT_ZERO_ANGLE      0.0f    //degrees
#define DEFAULT_ANGLE_RANGE     90.0f   //degrees
#define DEFAULT_LO_VOLTAGE      0.05f   //Vdd
#define DEFAULT_HI_VOLTAGE      0.95f   //Vdd

struct Settings {
    float mechanicalZeroAngle;
    float mechanicalAngleRange;
    float loVoltageClamp;
    float hiVoltageClamp;
} settings;

void startCondition() {
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(T_START);
}

void stopCondition() {
    digitalWrite(DATA_PIN, HIGH);
    delayMicroseconds(T_STOP);
}

void handover(void) {
    digitalWrite(DATA_PIN, HIGH);
    delayMicroseconds(T_ZERO);
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(T_ZERO * 2);
    pinMode(DATA_PIN, INPUT);
    delayMicroseconds(T_ZERO);
}

void takeover(void) {
    delayMicroseconds(T_ONE);
    pinMode(DATA_PIN, OUTPUT);
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(T_TAKEOVER);
}

void sendZero() {
    digitalWrite(DATA_PIN, HIGH);
    delayMicroseconds(T_ZERO);
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(T_BIT - T_ZERO);
}

void sendOne() {
    digitalWrite(DATA_PIN, HIGH);
    delayMicroseconds(T_ONE);
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(T_BIT - T_ONE);
}

void writeValue(byte value) {
    for (byte mask = 0x8000; mask; mask >>= 1) {
        if (mask & value)
            sendOne();
        else
            sendZero();
    }
}

void writeValueBig(uint16_t value) {
    for (byte mask = 0x80; mask; mask >>= 1) {
        if (mask & value)
            sendOne();
        else
            sendZero();
    }
}

uint16_t readValue() {
    uint16_t result = 0;
    for (int i = 15; i >= 0; i--) {
        delayMicroseconds(T_BIT / 2);
        result |= digitalRead(DATA_PIN) << i;
        delayMicroseconds(T_BIT / 2);
    }
    return result;
}

uint16_t readCommand(byte command) {
    startCondition();
    writeValue(command);
    handover();
    uint16_t result = readValue();
    takeover();
    startCondition();
    return result;
}

void startCommandMode() {
    pinMode(VDD_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    digitalWrite(VDD_PIN, HIGH);
    digitalWrite(DATA_PIN, HIGH);
    delay(T_TURN_ON);
    startCondition();
    writeValue(0x94);
    writeValue(0x16);
    writeValue(0xF4);
    stopCondition();
    delay(500);
}

void stopCommandMode() {
    digitalWrite(VDD_PIN, LOW);
    digitalWrite(DATA_PIN, LOW);
}

void writeCommand(byte command, uint16_t value){
    startCondition();
    writeValue(command);
    writeValueBig(value);
    startCondition();
    delay(500);
}

float toFloat(uint16_t value, uint8_t pointPosition) {
    return value * 1.0f / (1 << pointPosition);
}

uint16_t toUnsignedInt(float value, uint8_t pointPosition) {
    return (uint16_t) (value * (1 << pointPosition));
}

int calcValueCrc(int crc, unsigned int data) {
    const int poly = 0x107;
    for (int i = 15; i >= 0; i--) {
        crc <<= 1;
        crc |= (int) ((data & (1u << i)) >> i);
        if (crc & 0x100) crc ^= poly;
    }
    return crc;
}

int calcBufferCrc(const uint16_t *buffer) {
    int crc = 0xFF;
    for (int i = 0; i <= 7; i++) {
        crc = calcValueCrc(crc, i < 7 ? buffer[i] : 0x00);
    }
    return crc;
}

void printAsHex(uint16_t value) {
    for (int i = 0; i < 4; ++i) {
        Serial.print(((value) >> (3 - i) * 4) & 0xF, HEX);
    }
    Serial.print("h");
}

void printValue(const char *description, const char *measurementUnit, const float value) {
    Serial.print(description);
    Serial.print(value);
    Serial.println(measurementUnit);
}

void printSettings(const Settings &settings) {
    printValue("Mechanical zero angle is ", " degrees", settings.mechanicalZeroAngle);
    printValue("Mechanical angle range is ", " degrees", settings.mechanicalAngleRange);
    printValue("Low output voltage is ", " Vdd", settings.loVoltageClamp);
    printValue("High output voltage is ", " Vdd", settings.hiVoltageClamp);
    Serial.println();
}

void printReadingCommand(byte command) {
    Serial.print("Reading ");
    Serial.print(command, HEX);
    Serial.print("h - ");
}

void printWritingCommand(byte command) {
    Serial.print("Writing ");
    Serial.print(command, HEX);
    Serial.print("h - ");
}

void printHelp() {
    Serial.println("R - read sensor settings and print it");
    Serial.println("P - print current settings");
    Serial.println("Z [value] - set preset mechanical zero angle to [value] degrees");
    Serial.println("A [value] - set preset mechanical angle range to [value] degrees");
    Serial.println("L [value] - set preset low output voltage to [value] Vdd");
    Serial.println("H [value] - set preset high output voltage  to [value] Vdd");
    Serial.println("W - write current settings to sensor");
    Serial.println();
}

float readSerialFloat() {
    return Serial.readString().toFloat();
}

uint16_t extractCrcValue(const uint16_t *buffer) {
    return buffer[7] & 0x00FF;
}

void extractSettings(const uint16_t *buffer, Settings &settings) {

    uint16_t clampLo = buffer[3] & CLAMP_LO_MASK;
    uint16_t clampHi = buffer[4] & CLAMP_HI_MASK;

    uint32_t angRngRawValue = ((buffer[1] & ANGLE_RANGE_MULTIPLIER_MSB_MASK) << ANGLE_RANGE_MULTIPLIER_MSB_SHIFT) | (buffer[2] & ANGLE_RANGE_MULTIPLIER_LSB_MASK);
    float angRngMultiplier = toFloat(angRngRawValue, ANGLE_RANGE_MULTIPLIER_POINT);

    settings.mechanicalZeroAngle = toFloat(buffer[0], MECHANICAL_ZERO_ANGLE_POINT) * 180;
    settings.loVoltageClamp = clampLo / 5120.0f;
    settings.hiVoltageClamp = clampHi / 5120.0f;
    settings.mechanicalAngleRange = 180.0f / 8192 / angRngMultiplier * (clampHi - clampLo);

}

void readSensor(Settings &settings) {

    uint16_t values[8];

    for (int i = 0; i < 8; ++i) {
        byte command = (byte) (17 + i * 2);
        values[i] = readCommand(command);
        printReadingCommand(command);
        printAsHex(values[i]);
        Serial.println();
        delay(100);
    }
    if (calcBufferCrc(values) == extractCrcValue(values)) {
        extractSettings(values, settings);
        Serial.println();
        printSettings(settings);
    } else {
        Serial.println();
        Serial.println("Readings not correct, print 'R' to retry");
        Serial.println();
    }
}

void readSensorSettings(Settings &settings) {
    startCommandMode();
    readSensor(settings);
    stopCommandMode();
}

void setZeroAngle(Settings &settings, float value) {
    settings.mechanicalZeroAngle = value;
    printValue("Preset mechanical zero angle set to ", " degrees", settings.mechanicalZeroAngle);
    Serial.println();
    printSettings(settings);
}

void setAngleRange(Settings &settings, float value) {
    settings.mechanicalAngleRange = value;
    printValue("Preset mechanical angle range set to ", " degrees", settings.mechanicalAngleRange);
    Serial.println();
    printSettings(settings);
}

void setLoVoltage(Settings &settings, float value) {
    settings.loVoltageClamp = value;
    printValue("Low output voltage set to ", " Vdd", settings.loVoltageClamp);
    Serial.println();
    printSettings(settings);

}

void setHighVoltage(Settings &settings, float value) {
    settings.hiVoltageClamp = value;
    printValue("High output voltage set to ", " Vdd", settings.hiVoltageClamp);
    Serial.println();
    printSettings(settings);
}


void writeSensorSettings(Settings &settings) {

    Serial.println("Writing preset to memory...");
    Serial.println();

    uint16_t clampLo = ((uint16_t) (settings.loVoltageClamp * 5120)) & CLAMP_LO_MASK;
    uint16_t clampHi = ((uint16_t) (settings.hiVoltageClamp * 5120)) & CLAMP_HI_MASK;
    uint16_t mechanicalZeroAngle = toUnsignedInt(settings.mechanicalZeroAngle / 180, MECHANICAL_ZERO_ANGLE_POINT);
    float angleRangeMultiplier = (clampHi - clampLo) / 8192.0f * 180 / settings.mechanicalAngleRange;
    uint32_t rangeMultiplierValue = toUnsignedInt(angleRangeMultiplier, ANGLE_RANGE_MULTIPLIER_POINT);
    float clampAngle = 0.5f * (1 + 1.0f / 8192 / angleRangeMultiplier * (clampHi - clampLo));
    uint16_t clampAngleValue = (uint16_t) (clampAngle * 0x3ff);

    uint16_t buffer[8] = {
        mechanicalZeroAngle,
        (clampAngleValue << CLAMP_SW_ANGLE_SHIFT) | ((rangeMultiplierValue >> ANGLE_RANGE_MULTIPLIER_MSB_SHIFT) & ANGLE_RANGE_MULTIPLIER_MSB_MASK),
        rangeMultiplierValue & ANGLE_RANGE_MULTIPLIER_LSB_MASK,
        clampLo,
        clampHi,
        0,
        0,
        0
    };

    buffer[7] = 0 | (uint16_t) calcBufferCrc(buffer);

    for (int i = 0; i < 8; ++i) {
        byte command = (byte) (16 + i * 2);
        printWritingCommand(command);
        printAsHex(buffer[i]);
        writeCommand(command, buffer[i]);
        Serial.println();
    }
}


void initSettings(Settings &settings, float zeroAngle, float angleRange, float loVoltage, float hiVoltage) {
    settings.mechanicalZeroAngle = zeroAngle;
    settings.mechanicalAngleRange = angleRange;
    settings.loVoltageClamp = loVoltage;
    settings.hiVoltageClamp = hiVoltage;
}

void setup() {
    Serial.begin(9600);
    initSettings(settings, DEFAULT_ZERO_ANGLE, DEFAULT_ANGLE_RANGE, DEFAULT_LO_VOLTAGE, DEFAULT_HI_VOLTAGE);
}

void loop() {
    analogRead()
    if (Serial.available() > 0) {
        switch ((char) Serial.read()) {
            case 'R':
                readSensorSettings(settings);
                break;
            case 'Z':
                setZeroAngle(settings, readSerialFloat());
                break;
            case 'A':
                setAngleRange(settings, readSerialFloat());
                break;
            case 'L':
                setLoVoltage(settings, readSerialFloat());
                break;
            case 'H':
                setHighVoltage(settings, readSerialFloat());
                break;
            case 'P':
                printSettings(settings);
                break;
            case 'W':
                writeSensorSettings(settings);
                break;
            default:
                printHelp();
                break;
        }
    }
}


#include "driverlib.h"
#include "device.h"

void decodeCanMessageToErpm(const uint8_t *CanMsgData, int32_t *ErpmData, float *RpmData) {
    if (CanMsgData == NULL || ErpmData == NULL) return;

    uint32_t temp = 0;

    temp |= ((uint32_t)CanMsgData[0] << 24);
    temp |= ((uint32_t)CanMsgData[1] << 16);
    temp |= ((uint32_t)CanMsgData[2] << 8);
    temp |= (uint32_t)CanMsgData[3];


    *ErpmData = *(int32_t*)&temp;

    int32_t temp_value = *(int32_t*)&temp;
    float result = (float)temp_value / 10.0f;
    *RpmData = result;

}

void decodeCanMessageToDuty(const uint8_t *CanMsgData, float *DutyData) {
    if (CanMsgData == NULL || DutyData == NULL) return;

    uint16_t temp = 0;

    temp |= ((uint16_t)CanMsgData[4] << 8);
    temp |=  (uint16_t)CanMsgData[5];

    int16_t temp_value = *(int16_t*)&temp;
    float result = (float)temp_value / 10.0f;
    *DutyData = result;
}

void decodeCanMessageToDCVoltage(const uint8_t *CanMsgData, float *DCVoltage) {
    if (CanMsgData == NULL || DCVoltage == NULL) return;

    uint16_t temp = 0;

    temp |= ((uint16_t)CanMsgData[6] << 8);
    temp |=  (uint16_t)CanMsgData[7];

    int16_t temp_value = *(int16_t*)&temp;
    float result = (float)temp_value / 10.0f;
    *DCVoltage = result;
}

void decodeCanMessageToDCCurrent(const uint8_t *CanMsgData, float *DCCurrent) {
    if (CanMsgData == NULL || DCCurrent== NULL) return;

    uint16_t temp = 0;

    temp |= ((uint16_t)CanMsgData[2] << 8);
    temp |=  (uint16_t)CanMsgData[3];

    int16_t temp_value = *(int16_t*)&temp;
    float result = (float)temp_value / 10.0f;
    *DCCurrent = result;
}

void decodeCanMessageToACCurrent(const uint8_t *CanMsgData, float *ACCurrent) {
    if (CanMsgData == NULL || ACCurrent== NULL) return;

    uint16_t temp = 0;

    temp |= ((uint16_t)CanMsgData[0] << 8);
    temp |=  (uint16_t)CanMsgData[1];

    int16_t temp_value = *(int16_t*)&temp;
    float result = (float)temp_value / 10.0f;
    *ACCurrent = result;
}

void decodeCanMessageToTEMP(const uint8_t *CanMsgData, float *TEMPERATURE) {
    if (CanMsgData == NULL || TEMPERATURE == NULL) return;

    uint16_t temp = 0;

    temp |= ((uint16_t)CanMsgData[0] << 8);
    temp |=  (uint16_t)CanMsgData[1];

    int16_t temp_value = *(int16_t*)&temp;
    float result = (float)temp_value / 10.0f;
    *TEMPERATURE = result;
}

void decodeCanMessageToDQCurr(const uint8_t *CanMsgData, float *dCurrent, float *qCurrent) {
    if (CanMsgData == NULL || dCurrent == NULL || qCurrent == NULL) return;

    uint32_t temp = 0;

    temp |= ((uint32_t)CanMsgData[0] << 24);
    temp |= ((uint32_t)CanMsgData[1] << 16);
    temp |= ((uint32_t)CanMsgData[2] << 8);
    temp |= (uint32_t)CanMsgData[3];

    int32_t temp_value = *(int32_t*)&temp;
    float result = (float)temp_value / 100.0f;
    *dCurrent = result;

    temp = 0;
    temp |= ((uint32_t)CanMsgData[4] << 24);
    temp |= ((uint32_t)CanMsgData[5] << 16);
    temp |= ((uint32_t)CanMsgData[6] << 8);
    temp |= (uint32_t)CanMsgData[7];

    temp_value = *(int32_t*)&temp;
    result = (float)temp_value / 100.0f;
    *qCurrent = result;
}

void SetupMsgPhaseCurrent(uint8_t *CanMsgData, const float *ArmsCurr)
{
    if (CanMsgData == NULL || ArmsCurr == NULL) return;
    int16_t tempInt = (int16_t)(*ArmsCurr * 10.0f);
    uint16_t temp = (uint16_t)tempInt;
    CanMsgData[0] = temp & 0xFF;        // Byte bajo
    CanMsgData[1] = (temp >> 8) & 0xFF; // Byte alto

}


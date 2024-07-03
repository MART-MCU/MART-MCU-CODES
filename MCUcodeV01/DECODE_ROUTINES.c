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
<<<<<<< HEAD
/*void Interpolation(const float Data[][2], size_t size, float x0, float *y0) {
=======
void Interpolation(const float Data[][2], int size, float x0, float *y0) {
>>>>>>> 45e086287a3c373d4f5c9594d1e9cfb802171309
    if (Data == NULL || y0 == NULL) return;

    uint8_t indx = 0;

    while (indx < size && x0 > Data[indx][0]) {
        ++indx;
    }

    if (indx == 0) {
        *y0 = Data[0][1];
    } else if (indx == size) {
        *y0 = Data[size - 1][1];
    } else {

        float x1 = Data[indx - 1][0];
        float y1 = Data[indx - 1][1];
        float x2 = Data[indx][0];
        float y2 = Data[indx][1];

        *y0 = y1 + (y2 - y1) * (x0 - x1) / (x2 - x1);
    }
}

void DynTorSat(int slip,const float **mu){
    /*float mu0 = 1;
    float adherence = Interpolation(mu,67,slip,&mu0);
<<<<<<< HEAD
    float T_wh =
    float Fz = (M_VEH*GRAV*L1 + (M_VEH*accel + f_x_a)*H - T_wh + 2*J_D*accel/r)/L;
}*/
=======
    float T_wh;
    float Fz = (M_VEH*GRAV*L1 + (M_VEH*accel + f_x_a)*H - T_wh + 2*J_D*accel/r)/L;*/

}
>>>>>>> 45e086287a3c373d4f5c9594d1e9cfb802171309


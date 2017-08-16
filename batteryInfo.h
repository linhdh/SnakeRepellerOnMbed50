#ifndef __BLE_BATTERRY_H__
#define __BLE_BATTERRY_H__

#include <mbed.h>

class batteryInfoInternal {
public:
    void initBattery() {        
        NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
        NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                          (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                          (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                          (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                          (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
    };


    uint16_t getVoltage() {
        initBattery();
        
        NRF_ADC->CONFIG     &= ~ADC_CONFIG_PSEL_Msk;
        NRF_ADC->CONFIG     |= ADC_CONFIG_PSEL_AnalogInput3 << ADC_CONFIG_PSEL_Pos;
        NRF_ADC->TASKS_START = 1;
        while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {};    //68us for 10-bit res.
        return (uint16_t)NRF_ADC->RESULT; // 10 bit
    };
};

class batteryExternal : public batteryInfoInternal {
public:
    batteryExternal(PinName vPin, AnalogIn& sensePin, int r1=1000000, int r2=200000)
        : _vPin(vPin), _sensePin(sensePin), _r1(r1), _r2(r2) 
    {
        initBattery();
    };
    
    uint16_t getExtVoltage() 
    {
        //move all calculation to client side. Sensors send raw data.               
        return getVoltage();
    };   

private:
    PinName _vPin;        
    AnalogIn& _sensePin;
    int _r1;
    int _r2;         
};

#endif

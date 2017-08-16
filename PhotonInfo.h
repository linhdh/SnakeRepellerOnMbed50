/* 
 * File:   PhotonInfo.h
 * Author: linhdh
 *
 * Created on December 8, 2016, 6:53 PM
 */

#ifndef PHOTONINFO_H
#define	PHOTONINFO_H

class PhotonInfoInternal {
public:
    void initPhoton() {        
        NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
        NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                          (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                          (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                          (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                          (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
    };


    uint16_t getVoltage() {
        initPhoton();
        
        NRF_ADC->CONFIG     &= ~ADC_CONFIG_PSEL_Msk;
        NRF_ADC->CONFIG     |= ADC_CONFIG_PSEL_AnalogInput7 << ADC_CONFIG_PSEL_Pos;
        NRF_ADC->TASKS_START = 1;
        while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {};    //68us for 10-bit res.
        return (uint16_t)NRF_ADC->RESULT; // 10 bit
    };
};

class PhotonExternal : PhotonInfoInternal {
public:
    PhotonExternal(PinName vPin, AnalogIn& sensePin, int r1=10000)
        : _vPin(vPin), _sensePin(sensePin), _r1(r1)
    {
        initPhoton();        
    };
    
    uint16_t getExtVoltage() 
    {
        return getVoltage();      
    };     

private:
    PinName _vPin;
    AnalogIn& _sensePin;        
    int _r1;             
};



#endif	/* PHOTONINFO_H */


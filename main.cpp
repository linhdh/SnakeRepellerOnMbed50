#include "mbed.h"
#include "ble/BLE.h"
#include "batteryInfo.h"
#include "PhotonInfo.h"
#include "nrf_soc.h"

#define DEBUG 1
#define VERSION_04

#ifdef VERSION_04
#define LED1_PIN        p30
#define LED2_PIN        p29
#define BUZZER_PIN      p8
#define SPK_PIN         p4
#define VIB_PIN1        p23
#define VIB_PIN2        p28
#define LIGHTING_PIN    p5
#define LIGHT_CTROL     p6
#define BUTTON_PIN      p21
#define VSENSE_PIN      p2
#define CTRL_VSENSE_PIN p3
#define TX_PIN          p10
#define RX_PIN          p11
#endif

#if DEBUG
Serial pc(TX_PIN, RX_PIN);
#endif

//Led lighting:
DigitalOut led1(LED1_PIN, 1);
DigitalOut led2(LED2_PIN, 1);
//Motors:
DigitalOut vibControl1(VIB_PIN1, 0);
DigitalOut vibControl2(VIB_PIN2, 0);
//For Ultrasonic sound:
PwmOut spk(SPK_PIN);
//Buzzer For feedback sound:
PwmOut buzzer(BUZZER_PIN);
//Battery voltage sense and control
DigitalOut vsenseCtrl(CTRL_VSENSE_PIN, 1);
AnalogIn vsenseSensor(VSENSE_PIN);
batteryExternal bat(VSENSE_PIN, vsenseSensor);
//Photon sensor and controller
AnalogIn photonSensor(LIGHTING_PIN);
DigitalOut lightCtrl(LIGHT_CTROL, 0);
PhotonExternal photon(LIGHTING_PIN, photonSensor);
//Timers and Tickers:
Timeout timeout1, timeout2, timeout3;
Ticker ticker1, ticker2, ticker3, ticker4, ticker5, ticker6, ticker7, ticker8;
//On off button:
InterruptIn onOffBtn(BUTTON_PIN);

//Control variables:
bool isOff = true;
bool doUpdateData = false;
bool isPressed = false;
bool doSampleLight = false;
bool doSampleBat = false;
bool bleInitOk = false;
//Storing value variables:
uint16_t photonValue = 0;
uint16_t vsenseValue = 0;
uint16_t percentValue = 0;

//Configurable parameters:
uint16_t speakerPeriod = 37;       //us
uint16_t speakerDuration = 5;      //second
uint16_t speakerInterval = 11;     //second
uint16_t motorDuration = 3;        //second
uint16_t motorInterval = 30;       //second

//int photonR = 0;
//Some constants:
//const float VCC = 3.2; // Measured voltage of 5V line
//const float R_DIV = 10000; // Measured resistance of 10k resistor
// Set this to the minimum resistance require to turn an LED on:
//const float DARK_THRESHOLD = 10000.0;

//BLE data:
typedef struct Data {
    uint16_t    companyId;
    uint16_t    sequenceId;
    uint16_t    commandId;
    uint8_t     gateway;
    uint16_t    photonValue;
    uint8_t     batPercent;
} data_t;

data_t data;


void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising();
}

void periodicCallback(void)
{
    if(!doUpdateData)
        doUpdateData = true;
}

void updateData(void)
{
    if(!doSampleLight)
        doSampleLight = true;
    if(!doSampleBat)
        doSampleBat = true;
}

void motorOnOff()
{
    if(vibControl1 == 0)
        timeout1.attach(&motorOnOff, motorDuration);
    vibControl1 = vibControl2 = !vibControl1;    
}

void spkOff(void)
{
    spk.write(0.0f);
}

void spkOn(void)
{
    spk.period_us(speakerPeriod);
    spk.write(0.5f);
    timeout3.attach(spkOff, speakerDuration);
}

void pressBtn(void)
{
    if(isPressed)
        return;
    isPressed = true;
}

void releaseBtn(void)
{
    if(!isPressed)
        return;
    isPressed = false;
    isOff = !isOff;
    buzzerOn();
    if(isOff)
    {
        //if softdevice is disabled we can use:
        //NRF_POWER->SYSTEMOFF = 1;
        //for softdevice and ble:
        sd_power_system_off();
    }
    else
    {
        if(bleInitOk)
            BLE::Instance().gap().startAdvertising();
        ticker1.attach(periodicCallback, 1);
        ticker2.attach(updateData, 4);
        ticker4.attach(motorOnOff, motorInterval);
        ticker7.attach(spkOn, speakerInterval);
    }
}

/**
 * This function is called when the ble initialization process has failed
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE &ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (const uint8_t *)&data, sizeof(Data));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms. */
    bleInitOk = true;
}

/** @brief Function for converting the input voltage (in milli volts) into percentage of 3.7 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.7V returns 100% battery level. The limit for power failure is 2.5V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for li-ion is non-linear. In this model it is split into
 *           2 linear sections:
 *           - Section 1: 3.7V - 2.9V = 100% - 14.7% (85.3% drop on 800 mV)
 *           - Section 2: 2.9V - 2.5V = 14.7% - 0% (14.7% drop on 400 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3700)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = mvolts*0.106625 - 294.5125; /*100 - ((3700 - mvolts) * 85.3) / 800;*/
    }
    else if (mvolts > 2500)
    {
        battery_level = mvolts*0.03675 - 91.875; /*14.7 - ((2900 - mvolts) * 14.7) / 400;*/
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}

// main() runs in its own thread in the OS
int main() {
#if DEBUG
    pc.baud(38400);
#endif
    onOffBtn.fall(&pressBtn);
    onOffBtn.rise(&releaseBtn);
    //clear sense bitfield and set it to sense high mode.
    NRF_GPIO->PIN_CNF[BUTTON_PIN] &= GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos | 
            GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos | 
            GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos |
            GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos |
            GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos;    
    NRF_POWER->RESETREAS &= POWER_RESETREAS_OFF_Detected << POWER_RESETREAS_OFF_Pos |
            POWER_RESETREAS_DIF_NotDetected << POWER_RESETREAS_DIF_Pos | 
            POWER_RESETREAS_LPCOMP_NotDetected << POWER_RESETREAS_LPCOMP_Pos |
            POWER_RESETREAS_LOCKUP_NotDetected << POWER_RESETREAS_LOCKUP_Pos |
            POWER_RESETREAS_SREQ_NotDetected << POWER_RESETREAS_SREQ_Pos |
            POWER_RESETREAS_DOG_NotDetected << POWER_RESETREAS_DOG_Pos |
            POWER_RESETREAS_RESETPIN_NotDetected << POWER_RESETREAS_RESETPIN_Pos;
    
    //Init lux data
    data.companyId = 65295;
    data.sequenceId = 0;
    data.commandId = 8704;
    data.gateway = 0;
    data.photonValue = 0;
    data.batPercent = 100;
    
    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);
    
    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }
    
    //main loop   
    while (true) {
        // read photon sensors
        if(doSampleLight)
        {
            lightCtrl = 1;
            photonValue = photon.getExtVoltage();
            doSampleLight = false;
            lightCtrl = 0;
            //photonR = R_DIV * (2521 / photonValue - 1);   //This is the correct fomular.
#if DEBUG
            printf("photonValue = %d.\n", photonValue);            
            //printf("lux = %f.\n", photonR);
#endif
            data.photonValue = photonValue;
            //if(photonR >= DARK_THRESHOLD && (led1 == 1 || led2 == 1))
            //    led1 = led2 = 0;        //ON                
            //else if(photonR < DARK_THRESHOLD && (led1 == 0 || led2 == 0))
            //    led1 = led2 == 1;       //OFF
            if(photonValue <= 130 && (led1 == 1 || led2 == 1))
                led1 = led2 = 0;        //ON
            else if(photonValue > 130 && (led1 == 0 || led2 == 0))
                led1 = led2 = 1;        //OFF
        }
        
        // read battery voltage
        if(doSampleBat)
        {
            vsenseCtrl = 0;
            vsenseValue = bat.getExtVoltage();
            doSampleBat = false;
            vsenseCtrl = 1;
            float vBat = vsenseValue * 5.2734375; // 1.2 / 1024 * 1.5 * 3 * 1000 mV
            percentValue = battery_level_in_percent((uint16_t)round(vBat)); 
            
#if DEBUG
            printf("vsenseValue = %d, vBat = %f.\n", vsenseValue, vBat);            
            printf("percent = %0d.\n", percentValue);                
#endif
            data.batPercent = percentValue;
        }
        
        if(doUpdateData)
        {            
            doUpdateData = false;
            ble.gap().clearAdvertisingPayload();
            ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, (const uint8_t *)&data, sizeof(Data));
            data.sequenceId++;
#if DEBUG
            printf("%d %d %d %d %d %d\n", data.companyId, data.sequenceId, data.commandId, data.gateway, data.photonValue, data.batPercent);
            //printf("%x %x %x %x %x %x\n", data.companyId, data.sequenceId, data.commandId, data.gateway, data.lux, data.vBat);
#endif            
        }
    }
}


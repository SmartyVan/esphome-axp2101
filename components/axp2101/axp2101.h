#ifndef __AXP2101_H__
#define __AXP2101_H__

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#define AXP2101_ADDR 0x34

// ── AXP2101 Register Map ────────────────────────────────────────────────────
// Status
#define AXP2101_STATUS1              0x00  // Battery connect (bit 3)
#define AXP2101_STATUS2              0x01  // Charging state (bits 7:5)
#define AXP2101_IC_TYPE              0x03  // Chip ID (expect 0x4B)

// Input limits
#define AXP2101_INPUT_VOL_LIMIT      0x15  // VBUS voltage limit (bits 3:0)
#define AXP2101_INPUT_CUR_LIMIT      0x16  // VBUS current limit (bits 2:0)

// Watchdog & gauge
#define AXP2101_CHARGE_GAUGE_WDT     0x18  // Button batt (bit 2), watchdog (bit 0)
#define AXP2101_WDT_CTRL             0x19  // Config (5:4), clear (3), timeout (2:0)

// DC over/under-voltage protection
#define AXP2101_DC_OVP_UVP_CTRL     0x23  // DC high/low voltage protection

// System power-down voltage
#define AXP2101_VOFF_SET             0x24  // Bits 2:0, 2600-3300mV, 100mV steps

// Power key timing
#define AXP2101_IRQ_OFF_ON_LEVEL     0x27  // Off time (3:2), On time (1:0)

// ADC channel control
#define AXP2101_ADC_CHANNEL_CTRL     0x30  // Temp(4), SysV(3), VbusV(2), TS(1), BatV(0)

// ADC data
#define AXP2101_BAT_VOLTAGE_H        0x34  // Battery voltage high 5 bits
#define AXP2101_BAT_VOLTAGE_L        0x35  // Battery voltage low 8 bits

// IRQ enable / status
#define AXP2101_INTEN1               0x40
#define AXP2101_INTEN2               0x41
#define AXP2101_INTEN3               0x42
#define AXP2101_INTSTS1              0x48
#define AXP2101_INTSTS2              0x49
#define AXP2101_INTSTS3              0x4A

// TS pin control
#define AXP2101_TS_PIN_CTRL          0x50

// Charger settings
#define AXP2101_IPRECHG_SET          0x61  // Precharge current (bits 1:0)
#define AXP2101_ICC_CHG_SET          0x62  // Constant current (bits 4:0)
#define AXP2101_ITERM_CHG_SET        0x63  // Termination current (bits 3:0)
#define AXP2101_CV_CHG_VOL_SET       0x64  // Target voltage (bits 2:0)

// Battery detection
#define AXP2101_BAT_DET_CTRL         0x68  // Battery detect enable (bit 0)

// Charge LED
#define AXP2101_CHGLED_SET_CTRL      0x69

// Button battery
#define AXP2101_BTN_BAT_CHG_VOL     0x6A  // Voltage (bits 2:0), 2600-3300mV

// DC enable/disable
#define AXP2101_DC_ONOFF_CTRL        0x80  // DC1(0), DC2(1), DC3(2), DC4(3), DC5(4)

// DC voltage registers
#define AXP2101_DC1_VOLTAGE          0x82  // Bits 4:0, 1500-3400mV, 100mV steps
#define AXP2101_DC2_VOLTAGE          0x83  // Bits 6:0, dual range
#define AXP2101_DC3_VOLTAGE          0x84  // Bits 6:0, triple range
#define AXP2101_DC4_VOLTAGE          0x85  // Bits 6:0, dual range
#define AXP2101_DC5_VOLTAGE          0x86  // Bits 4:0, 1400-3700mV, 100mV steps

// LDO enable/disable
#define AXP2101_LDO_ONOFF_CTRL0     0x90  // ALDO1(0),ALDO2(1),ALDO3(2),ALDO4(3),BLDO1(4),BLDO2(5),CPUSLDO(6),DLDO1(7)
#define AXP2101_LDO_ONOFF_CTRL1     0x91  // DLDO2(0)

// LDO voltage registers (all bits 4:0, mask 0xE0)
#define AXP2101_ALDO1_VOLTAGE        0x92  // 500-3500mV, 100mV steps
#define AXP2101_ALDO2_VOLTAGE        0x93
#define AXP2101_ALDO3_VOLTAGE        0x94
#define AXP2101_ALDO4_VOLTAGE        0x95
#define AXP2101_BLDO1_VOLTAGE        0x96
#define AXP2101_BLDO2_VOLTAGE        0x97
#define AXP2101_CPUSLDO_VOLTAGE      0x98  // 500-1400mV, 50mV steps
#define AXP2101_DLDO1_VOLTAGE        0x99
#define AXP2101_DLDO2_VOLTAGE        0x9A

// Battery percentage
#define AXP2101_BAT_PERCENT          0xA4

// ── Bit positions for LDO_ONOFF_CTRL0 (0x90) ────────────────────────────────
#define AXP2101_LDO_ALDO1_BIT       0
#define AXP2101_LDO_ALDO2_BIT       1
#define AXP2101_LDO_ALDO3_BIT       2
#define AXP2101_LDO_ALDO4_BIT       3
#define AXP2101_LDO_BLDO1_BIT       4
#define AXP2101_LDO_BLDO2_BIT       5
#define AXP2101_LDO_CPUSLDO_BIT     6
#define AXP2101_LDO_DLDO1_BIT       7

// ── Enums / constants matching XPowersLib values ─────────────────────────────
// VBUS voltage limit (register 0x15 bits 3:0)
#define VBUS_VOL_LIM_4V36           6

// VBUS current limit (register 0x16 bits 2:0)
#define VBUS_CUR_LIM_1500MA         4

// Precharge current (register 0x61 bits 1:0)
#define PRECHARGE_50MA               2

// Constant charge current (register 0x62 bits 4:0)
#define CHG_CUR_200MA                8

// Termination current (register 0x63 bits 3:0)
#define CHG_ITERM_25MA               1

// Charge target voltage (register 0x64 bits 2:0)
#define CHG_VOL_4V1                  2

// Power key press-off time (register 0x27 bits 3:2)
#define POWEROFF_4S                  0

// Power key press-on time (register 0x27 bits 1:0)
#define POWERON_128MS                0

// Watchdog config (register 0x19 bits 5:4)
#define WDT_IRQ_TO_PIN              2

// Watchdog timeout (register 0x19 bits 2:0)
#define WDT_TIMEOUT_4S               2

// Charge LED modes
#define CHG_LED_OFF                  0
#define CHG_LED_BLINK_1HZ           1
#define CHG_LED_BLINK_4HZ           2
#define CHG_LED_ON                   3
#define CHG_LED_CTRL_CHG             4

namespace esphome {
namespace axp2101 {

enum AXP2101Model {
  AXP2101_M5CORE2,
};

#define SLEEP_MSEC(us) (((uint64_t)us) * 1000L)
#define SLEEP_SEC(us)  (((uint64_t)us) * 1000000L)
#define SLEEP_MIN(us)  (((uint64_t)us) * 60L * 1000000L)
#define SLEEP_HR(us)   (((uint64_t)us) * 60L * 60L * 1000000L)

#define CURRENT_100MA  (0b0000)
#define CURRENT_190MA  (0b0001)
#define CURRENT_280MA  (0b0010)
#define CURRENT_360MA  (0b0011)
#define CURRENT_450MA  (0b0100)
#define CURRENT_550MA  (0b0101)
#define CURRENT_630MA  (0b0110)
#define CURRENT_700MA  (0b0111)

class AXP2101Component : public PollingComponent, public i2c::I2CDevice {
public:
  void set_batteryvoltage_sensor(sensor::Sensor *batteryvoltage_sensor) { batteryvoltage_sensor_ = batteryvoltage_sensor; }
  void set_batterylevel_sensor(sensor::Sensor *batterylevel_sensor) { batterylevel_sensor_ = batterylevel_sensor; }
  void set_batterycharging_bsensor(binary_sensor::BinarySensor *batterycharging_bsensor) { batterycharging_bsensor_ = batterycharging_bsensor; }
  void set_brightness(float brightness) { brightness_ = brightness; }
  void set_model(AXP2101Model model) { this->model_ = model; }
  void set_lcd_enabled(bool on);
  void set_blue_led(bool on);
  void set_speaker_enabled(bool on);
      // -- sleep
  void SetSleep(void);
  void DeepSleep(uint64_t time_in_us = 0);
  void LightSleep(uint64_t time_in_us = 0);

  // ========== INTERNAL METHODS ==========
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

private:
    static std::string GetStartupReason();

protected:
    sensor::Sensor *batteryvoltage_sensor_;
    sensor::Sensor *batterylevel_sensor_;
    binary_sensor::BinarySensor *batterycharging_bsensor_;
    float brightness_{1.0f};
    float curr_brightness_{-1.0f};
    AXP2101Model model_;

    // ── Register helpers ─────────────────────────────────────────────────
    void Write1Byte(uint8_t Addr, uint8_t Data);
    uint8_t Read8bit(uint8_t Addr);
    uint16_t Read12Bit(uint8_t Addr);
    uint16_t Read13Bit(uint8_t Addr);
    uint16_t Read16bit(uint8_t Addr);
    uint32_t Read24bit(uint8_t Addr);
    uint32_t Read32bit(uint8_t Addr);
    void ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff);

    // Read-modify-write helpers for single bits
    void setRegisterBit(uint8_t reg, uint8_t bit);
    void clrRegisterBit(uint8_t reg, uint8_t bit);
    bool getRegisterBit(uint8_t reg, uint8_t bit);

    // ── Power rail helpers ───────────────────────────────────────────────
    void setLdoVoltage(uint8_t reg, uint16_t millivolt);   // 500-3500mV, 100mV steps
    void setLdoEnabled(uint8_t reg, uint8_t bit, bool on);

    // ── Feature methods ──────────────────────────────────────────────────
    void UpdateBrightness();
    void SetChargeCurrent(uint8_t);
    void PowerOff();
    void SetAdcState(bool State);

    // ── Coulomb counter ──────────────────────────────────────────────────
    void EnableCoulombcounter(void);
    void DisableCoulombcounter(void);
    void StopCoulombcounter(void);
    void ClearCoulombcounter(void);
    uint32_t GetCoulombchargeData(void);
    uint32_t GetCoulombdischargeData(void);
    float GetCoulombData(void);

    // ── Measurement ──────────────────────────────────────────────────────
    float GetBatVoltage();
    float GetBatCurrent();
    float GetVinVoltage();
    float GetVinCurrent();
    float GetVBusVoltage();
    float GetVBusCurrent();
    float GetTempInAXP2101();
    float GetBatPower();
    float GetBatChargeCurrent();
    float GetAPSVoltage();
    float GetBatCoulombInput();
    float GetBatCoulombOut();
    uint8_t GetWarningLevel(void);
    void SetCoulombClear();
    uint8_t GetBtnPress(void);

    // Battery
    bool isBatteryConnect();
    int  getBatteryPercent();
    bool isCharging();
};

}
}

#endif

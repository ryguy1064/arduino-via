#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "pins.h"
#include "registers.h"
#include "reg_gpio.h"
#include "reg_serial.h"
#include "reg_lcd.h"
#include "reg_rtc.h"
#include "reg_irq.h"

#define STEP_CLK_OUT()                      (TCCR1C = _BV(FOC1A))
#define GET_PIN_STATE(port, pin)            (*port & (pin))
#define SET_PIN_STATE(port, pin, state)     do { if (state) *port |= (pin); else *port &= ~(pin); } while (0)

// void ceIsr();
// void finishReadRequest(uint8_t dataToSend);
// void ceComplete(bool isReadRequest);
void handleCEAssertion();

// Register 0
// uint8_t reg0_SerialRead();
// void reg0_SerialWrite(uint8_t data);

volatile uint8_t *PIN_CE_inport;
uint8_t PIN_CE_pinmask;
volatile uint8_t *REG_ADDR_inport;
#define REG_ADDR_MASK   0xF0
#define REG_ADDR_SHIFT  4
volatile uint8_t *PIN_RW_inport;
uint8_t PIN_RW_pinmask;
volatile uint8_t *PIN_HALT_EN_outport;
uint8_t PIN_HALT_EN_pinmask;
volatile uint8_t *PIN_CLK_OUT_inport;
uint8_t PIN_CLK_OUT_pinmask;
volatile uint8_t *LED_BUILTIN_outport;
uint8_t LED_BUILTIN_pinmask;

device_register_t *reg_serialData, *reg_irqFlags, *reg_irqEn,
                  *reg_gpioData, *reg_gpioDir;

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
bool lcdInitialClear = false;
uint8_t lcdLine = 0;
uint8_t lcdCol = 0;

void setup()
{
    // Pin setup
    {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
        LED_BUILTIN_outport = portOutputRegister(digitalPinToPort(LED_BUILTIN));
        LED_BUILTIN_pinmask = digitalPinToBitMask(LED_BUILTIN);

        PIN_CE_inport = portInputRegister(digitalPinToPort(PIN_CE));
        PIN_CE_pinmask = digitalPinToBitMask(PIN_CE);

        PIN_RW_inport = portInputRegister(digitalPinToPort(PIN_RW));
        PIN_RW_pinmask = digitalPinToBitMask(PIN_RW);

        REG_ADDR_inport = &PIND;

        pinMode(PIN_HALT_EN, OUTPUT);
        digitalWrite(PIN_HALT_EN, HIGH);
        PIN_HALT_EN_outport = portOutputRegister(digitalPinToPort(PIN_HALT_EN));
        PIN_HALT_EN_pinmask = digitalPinToBitMask(PIN_HALT_EN);

        pinMode(PIN_CLK_OUT, OUTPUT);
        PIN_CLK_OUT_inport = portInputRegister(digitalPinToPort(PIN_CLK_OUT));
        PIN_CLK_OUT_pinmask = digitalPinToBitMask(PIN_CLK_OUT);

        pinMode(PIN_IRQ, INPUT);
    }

    Serial.begin(115200);
    // Wire.begin();

    // Initialize the LCD (also starts Wire)
    lcd.init();

    lcd.backlight();
    lcd.print("Arduino VIA v0.2");

    Serial.println("Arduino VIA v0.2");
    Serial.println();

    // Set I/O bus to inputs
    Wire.beginTransmission(0x20);
    Wire.write(0xFF);
    Wire.endTransmission();

    // Populate registers
    //   Serial TX/RX @ 0
    reg_serialData = addRegister(0, reg0_SerialRead, reg0_SerialWrite);

    //   Interrupt Flags @ 1
    reg_irqFlags = addRegister(1, reg1_IrqFlagsRead, NULL, true);

    //   Interrupt Enable @ 2
    reg_irqEn = addRegister(2);

    //   32-bit RTC millis byte 0..3 @ 3..6
    static_assert((RTC_BASE_ADDRESS == 3), "RTC base address needs to be at 3");
    addRegister(3, regN_RtcRead, NULL, true);
    addRegister(4, regN_RtcRead, NULL, true);
    addRegister(5, regN_RtcRead, NULL, true);
    addRegister(6, regN_RtcRead, NULL, true);

    // GPIO Data @ 7
    reg_gpioData = addRegister(7, reg_gpioIORead, reg_gpioIOWrite);

    // GPIO Dir @ 8
    reg_gpioDir = addRegister(8, reg_gpioDirRead, reg_gpioDirWrite);

    // LCD output @ 9
    addRegister(9, NULL, reg_LcdWrite, true, &lcd);

    // Set all GPIOs to inputs
    reg_gpioDirWrite(reg_gpioDir, 0);

    // Start CLK_OUT (1 MHz, 50% duty cycle)
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 7;
    TCCR1A = _BV(COM1A0);
    TCCR1B = _BV(WGM12) | _BV(CS10);
}

void loop()
{
    auto irqFlags = (irq_flags_t *)&reg_irqFlags->data;
    auto irqEn = (irq_flags_t *)&reg_irqEn->data;

    // CE was asserted by address selection
    if (!GET_PIN_STATE(PIN_CE_inport, PIN_CE_pinmask))
    {
        handleCEAssertion();

        // if (!lcdInitialClear)
        // {
        //     lcd.clear();
        //     lcdInitialClear = true;
        // }
    }

    // Send serial TX data
    if (needToSendChar)
    {
        // if (charToSend == '\r')
        // {
        //     lcdLine++;
        //     lcdLine &= 0x3;
        //     lcd.setCursor(0, lcdLine);
        // }
        // else
        // {
        //     lcdCol++;
        //     if (lcdCol == 20)
        //     {
        //         lcdCol = 0;
        //         lcdLine++;
        //         lcdLine &= 0x3;
        //         lcd.setCursor(lcdCol, lcdLine);
        //     }
        //     lcd.write(charToSend);
        // }
        Serial.write(charToSend);
        needToSendChar = false;
    }

    // Check for new serial RX data
    irqFlags->flags.serialRxDataAvailable = (Serial.available()) ? 1 : 0;

    // Assert IRQ (open-drain output) according to flags and enable states
    pinMode(PIN_IRQ, (irqFlags->all & irqEn->all) ? OUTPUT : INPUT);
}

void handleCEAssertion()
{
    uint8_t regAddr;
    uint8_t busData;
    bool isReadRequest;

    // Stop CLK_OUT
    TCCR1B &= ~_BV(CS10);
    // TCNT0 = 0;

    // Ensure CLK_OUT is stopped in a high state
    if (!GET_PIN_STATE(PIN_CLK_OUT_inport, PIN_CLK_OUT_pinmask))
    {
        STEP_CLK_OUT();
    }

    // Debug LED
    // SET_PIN_STATE(LED_BUILTIN_outport, LED_BUILTIN_pinmask, HIGH);

    // Get register address from A3:A0
    regAddr = *REG_ADDR_inport;
    regAddr &= REG_ADDR_MASK;
    regAddr >>= REG_ADDR_SHIFT;

    // Determine request type
    isReadRequest = GET_PIN_STATE(PIN_RW_inport, PIN_RW_pinmask);

    // READ request
    if (isReadRequest)
    {
        // Get data from register
        busData = handleRegisterRead(regAddr);

        // Set data to bus
        Wire.beginTransmission(0x20);
        Wire.write(busData);
        Wire.endTransmission();
    }

    // WRITE Request
    else
    {
        // Get data from bus
        Wire.requestFrom(0x20, 1);
        busData = Wire.read();

        // Save data to register
        handleRegisterWrite(regAddr, busData);
    }

    // Release HALT_EN on processor (will allow RDY to go high even if CE is still low, clock is still stopped)
    SET_PIN_STATE(PIN_HALT_EN_outport, PIN_HALT_EN_pinmask, LOW);

    // RDY pin needs some time to be pulled up to VCC
    delayMicroseconds(5);

    // Step clock once to create final falling edge for processor to latch data
    STEP_CLK_OUT();

    // Set I/O bus back to inputs if request was a READ
    if (isReadRequest)
    {
        Wire.beginTransmission(0x20);
        Wire.write(0xFF);
        Wire.endTransmission();
    }

    // Re-enable HALT_EN to catch processor on next device access
    SET_PIN_STATE(PIN_HALT_EN_outport, PIN_HALT_EN_pinmask, HIGH);

    // Resume CLK_OUT at full speed
    TCCR1B |= _BV(CS10);

    // Debug LED
    // SET_PIN_STATE(LED_BUILTIN_outport, LED_BUILTIN_pinmask, LOW);

    // Debug
#if 0
    lcd.setCursor(0, 0);
    if (isReadRequest)
    {
        lcd.print("R ");
        lcd.print(regAddr, HEX);
        lcd.print(": ");
        lcd.print(busData, HEX);
        lcd.print("      ");
    }
    else
    {
        lcd.print("W ");
        lcd.print(regAddr, HEX);
        lcd.print(": ");
        lcd.println(busData, HEX);
        lcd.print("      ");
    }
#endif
}

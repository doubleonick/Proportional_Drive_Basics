
volatile int motor1stepCount = 0;
volatile int motor2stepCount = 0;
volatile int motor1continuous = false;
volatile int motor2continuous = false;
volatile int counter1top = 0;
volatile int counter2top = 0;
volatile int counter1 = 0;
volatile int counter2 = 0;

const int stepPin1 = 44;
const int dirPin1 = 27;
const int stepPin2 = 45;
const int dirPin2 = 30;
const int enablePin = 26; //nSLEEP
const int configPin = 31;


void pinInit() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(configPin, OUTPUT);
  digitalWrite(enablePin, LOW);
  delay(1);
}

void stepperInit() {
  pinInit();
  tcDisable();
  tcConfigure(2000);
  tcStartCounter();
  digitalWrite(configPin, HIGH);
  digitalWrite(enablePin, HIGH);
}

void DcMotorInit(uint16_t maxPWM) {
  pinInit();
  initPWM(maxPWM);
  digitalWrite(configPin, LOW);
  digitalWrite(enablePin, HIGH);
}

void setMotorCurrent(byte val) {
  val = constrain(val, 0, 127);
  Wire.beginTransmission(0x2F);
  Wire.write(val);
  Wire.endTransmission();
}

void setMotor(int motor, int stepSpeed) {
  noInterrupts();
  if (motor == 1) {
    if (stepSpeed < 0) {
      digitalWrite(dirPin1, HIGH);
    } else {
      digitalWrite(dirPin1, LOW);
    }
    counter1top = 11 - abs(stepSpeed);
    if (stepSpeed == 0) {
      counter1top = 0;
    }
    counter1 = 0;
    motor1continuous = true;
    motor1stepCount = 0;
  } else {
    if (stepSpeed > 0) {
      digitalWrite(dirPin2, HIGH);
    } else {
      digitalWrite(dirPin2, LOW);
    }
    counter2top = 11 - abs(stepSpeed);
    if (stepSpeed == 0) {
      counter2top = 0;
    }
    counter2 = 0;
    motor2continuous = true;
    motor2stepCount = 0;
  }
  interrupts();
}

void setMotor(int motor, int stepSpeed, int stepCount) {
  noInterrupts();
  if (motor == 1) {
    if (stepSpeed < 0) {
      digitalWrite(dirPin1, HIGH);
    } else {
      digitalWrite(dirPin1, LOW);
    }
    counter1top = 11 - abs(stepSpeed);
    if (stepSpeed == 0) {
      counter1top = 0;
    }
    counter1 = 0;
    motor1continuous = false;
    motor1stepCount = abs(stepCount);
  } else {
    if (stepSpeed > 0) {
      digitalWrite(dirPin2, HIGH);
    } else {
      digitalWrite(dirPin2, LOW);
    }
    counter2top = 11 - abs(stepSpeed);
    if (stepSpeed == 0) {
      counter2top = 0;
    }
    counter2 = 0;
    motor2continuous = false;
    motor2stepCount = abs(stepCount);
  }
  interrupts();
}

int isMotorSpinning() { //if no value is passed, default to both motors
  return isMotorSpinning(1 | 2);
}

int isMotorSpinning(int motor) {
  noInterrupts();
  int returnVal = 0;
  if (motor & 1) {
    returnVal |= motor1stepCount;
  } if (motor & 2)   {
    returnVal |= motor2stepCount;
  }
  interrupts();
  return returnVal;
}

void tcConfigure(uint32_t sampleRate)
{
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset();

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;

  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (tcIsSyncing());

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing());
}

bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void tcStartCounter()
{
  // Enable TC

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

void tcReset()
{
  // Reset TCx
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

void tcDisable()
{
  // Disable TC5
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

#ifdef __cplusplus
extern "C" {
#endif

extern void Stepper_Handler (void);

void Stepper_Handler (void)
{
  if (counter1top) {
    counter1++;
    if (counter1 >= counter1top) {
      if (motor1stepCount || motor1continuous) {
        if (motor1stepCount)
          motor1stepCount--;
        counter1 = 0;
        digitalWrite(stepPin1, HIGH);
        digitalWrite(stepPin1, LOW);
      }
    }
  }
  if (counter2top) {
    counter2++;
    if (counter2 >= counter2top) {
      if (motor2stepCount || motor2continuous) {
        if (motor2stepCount)
          motor2stepCount--;
        counter2 = 0;
        digitalWrite(stepPin2, HIGH);
        digitalWrite(stepPin2, LOW);
      }
    }
  }
  // Clear the interrupt
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}

void TC5_Handler (void) __attribute__ ((weak, alias("Stepper_Handler")));

#ifdef __cplusplus
}
#endif

#include <wiring_private.h>
#ifndef syncTCC
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}
#endif

void initPWM(uint16_t maxPWM) {
  pinPeripheral(stepPin1, PIO_TIMER);
  pinPeripheral(stepPin2, PIO_TIMER);

  Tcc* TCCx = (Tcc*) TCC1;
  if (!TCCx->CTRLA.bit.ENABLE) {
    // Enable clock
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC0_TCC1));
    while (GCLK->STATUS.bit.SYNCBUSY == 1);
    // Disable TCCx
    TCCx->CTRLA.bit.ENABLE = 0;
    syncTCC(TCCx);
    // Set TCCx as normal PWM
    TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
    syncTCC(TCCx);
    // Set the initial value
    TCCx->CC[0].reg = (uint32_t) 0;
    syncTCC(TCCx);
    TCCx->CC[1].reg = (uint32_t) 0;
    syncTCC(TCCx);
    // Set PER to maximum counter value (resolution : 0xFFFF)
    TCCx->PER.reg = maxPWM;
    syncTCC(TCCx);
    // Enable TCCx
    TCCx->CTRLA.bit.ENABLE = 1;
    syncTCC(TCCx);
  }
}
/*
  void MotorDriver::setMotor(uint8_t motor, int16_t val){
  setMotor(motor,val,false);
  }
*/
void setDCMotor(uint8_t motor, int16_t val) { //, bool useBrakes){
  if (motor == 1) {
    if (val > 0) {
      digitalWrite(dirPin1, HIGH);
      writePWMforDC(stepPin1, abs(val));
    } else {
      digitalWrite(dirPin1, LOW);
      writePWMforDC(stepPin1, abs(val));
    }
  } else if (motor == 2) {
    if (val > 0) {
      digitalWrite(dirPin2, HIGH);
      writePWMforDC(stepPin2, abs(val));
    } else {
      digitalWrite(dirPin2, LOW);
      writePWMforDC(stepPin2, abs(val));
    }
  }
}
void writePWMforDC(uint32_t pin, uint32_t value) {
  if (pin == stepPin1 || pin == stepPin2) {
    pin -= stepPin1;
    Tcc* TCCx = (Tcc*) TCC1;
    TCCx->CTRLBSET.bit.LUPD = 1;
    syncTCC(TCCx);
    TCCx->CCB[pin].reg = (uint32_t) value;
    syncTCC(TCCx);
    TCCx->CTRLBCLR.bit.LUPD = 1;
    syncTCC(TCCx);
  }
}

//  { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_NONE }, // SW
//  { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_NONE }, // DAC/VOUT
/*
  #include <wiring_private.h>

  static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
  static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
  }

  void writeMotor(uint32_t motor, uint32_t value)
  {
  //PinDescription pinDesc = g_APinDescription[pin];
  //uint32_t attr = pinDesc.ulPinAttribute;
  uint32_t pin;
  uint32_t tcNum;
  uint8_t tcChannel;
  Tc* TCx;
  if (motor == 1) {
    TCx = (Tc*) 6;
    tcNum=6;
    tcChannel=1;
    pin = 25;
  } else {
    TCx = (Tc*) 7;
    tcNum=7;
    tcChannel=1;
    pin = 31;
  }

  value=12000000/value;

  static bool tcEnabled[TC_INST_NUM];

  pinPeripheral(pin, PIO_TIMER);

  //PWM3_CH1
  if (!tcEnabled[tcNum]) {
    tcEnabled[tcNum] = true;
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC6_TC7));
    while (GCLK->STATUS.bit.SYNCBUSY == 1);

    // Disable TCx
    TCx->COUNT16.CTRLA.bit.ENABLE = 0;
    syncTC_16(TCx);
    // Set Timer counter Mode to 16 bits, normal PWM
    TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MPWM;
    syncTC_16(TCx);
    // Set the initial value
    TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
    syncTC_16(TCx);
    // Enable TCx
    TCx->COUNT16.CTRLA.bit.ENABLE = 1;
    syncTC_16(TCx);
  } else {
    TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
    syncTC_16(TCx);
  }
  return;
  }
*/

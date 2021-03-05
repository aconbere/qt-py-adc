#include <sam.h>
// #include <Wire.h>

#define VOLT_SENSE PORT_PA02
#define CURRENT_SENSE PORT_PA03
#define MULTISAMPLE false

void Clock_Init(void) {
  // no prescaler (is 8 on reset)
  SYSCTRL->OSC8M.bit.PRESC = 0;

  // enable source
  SYSCTRL->OSC8M.bit.ENABLE = 1;

  // select GCLK_GEN[1]
  GCLK->GENDIV.bit.ID = 1;

  // no prescaler
  GCLK->GENDIV.bit.DIV = 0;

  // select GCLK_GEN[1]
  GCLK->GENCTRL.bit.ID = 1;

  // enable generator
  GCLK->GENCTRL.bit.GENEN = 1;

  // OSC8M source
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_SRC_OSC8M;

  // no prescaler
  PM->APBCSEL.bit.APBCDIV = 0;

  // Enable clocking for the ADC */
  PM->APBCMASK.bit.ADC_ = 1;
  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

uint32_t ADC_Read() {
  /* Start the ADC using a software trigger. */
  ADC->SWTRIG.bit.START = true;

  /* Wait for the result ready flag to be set. */
  while (ADC->INTFLAG.bit.RESRDY == 0);

  /* Clear the flag. */
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  /* Read the value. */
  return ADC->RESULT.reg;
}

void TC5_Init() {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_TC4_TC5) |
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;

  while (GCLK->STATUS.bit.SYNCBUSY);

   // Configure synchronous bus clock
  PM->APBCSEL.bit.APBCDIV = 0; // no prescaler
  PM->APBCMASK.bit.TC5_ = 1; // enable TC5 interface

  // Configure Count Mode (16-bit)
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 |
                            TC_CTRLA_WAVEGEN_MFRQ |
                            TC_CTRLA_PRESCALER_DIV1024 |
                            TC_CTRLA_ENABLE;


  // Configure Prescaler for divide by 2 (500kHz clock to COUNT)
  TC5->COUNT16.CTRLA.bit.PRESCALER = 1;

  // Configure TC5 Compare Mode for compare channel 0
  // "Match Frequency" operation
  TC5->COUNT16.CTRLA.bit.WAVEGEN = 1;

  // Initialize compare value for 100mS @ 500kHz
  TC5->COUNT16.CC[0].reg = 500000;

  // Enable TC5 compare mode interrupt generation
  // Enable match interrupts on compare channel 0 
  TC5->COUNT16.INTENSET.bit.MC0 = 1;

  // Enable TC5
  TC5->COUNT16.CTRLA.bit.ENABLE = 1;

  // Wait until TC5 is enabled
  while(TC5->COUNT16.STATUS.bit.SYNCBUSY == 1);
  NVIC_SetPriority(TC5_IRQn, 3);
  NVIC_EnableIRQ(TC5_IRQn);
}

void ADC_Init() {
  // PORT->Group[0] is "PORTA" if it was Group[1] it would be "PORTB"

  /* Set PA02 as an input pin. */
  PORT->Group[0].DIRCLR.reg = PORT_PA02;

  /* Enable the peripheral multiplexer for PA02. */
  PORT->Group[0].PINCFG[2].reg |= PORT_PINCFG_PMUXEN;

  /* PMUX is strange, the register only needs 4 bits
   * so they split the registers into odd and even segements
   * so a single PMUX register looks like
   *                PMUXE           PMUXO
   * PMUX[N / 2] [ 0, 0, 0, 0, ] [ 0, 0, 0, 0 ]
   *
   * such that for pin 3 we would have index 1 but reference PMUXO (for odd)
   * 
   * Set PA02 to function B which is analog input.
   */
  PORT->Group[0].PMUX[2 >> 1].bit.PMUXE = PORT_PMUX_PMUXO_B;

  // Generic Clock ADC
  // Generic Clock Generator 0 is source
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC )|
                      GCLK_CLKCTRL_GEN_GCLK0   |
                      GCLK_CLKCTRL_CLKEN;

  while (GCLK->STATUS.bit.SYNCBUSY);
  while (ADC->STATUS.bit.SYNCBUSY);

  // Divide Clock by 512.
  // 10 bits resolution as default
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |
                   ADC_CTRLB_RESSEL_12BIT;

  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(10);

  // 1 sample only (no oversampling nor averaging)
  // Adjusting result by 0
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |
                     ADC_AVGCTRL_ADJRES(0x0ul);

  while(ADC->STATUS.bit.SYNCBUSY);


  /* Configure the input parameters.

   * - GAIN_DIV2 means that the input voltage is halved. This is important
   *   because the voltage reference is 1/2 of VCCA. So if you want to
   *   measure 0-3.3v, you need to halve the input as well.

   * - MUXNEG_GND means that the ADC should compare the input value to GND.

   * - PORT_PA02 maps to D0/A0 on the qt py
   * - PA02 maps to AIN0
   * - ADC resides in peripheral function B
   * - Peripheral Function B is in PMUX(E)[0x01]

   * - MUXPOST_PIN3 means that the ADC should read from AIN3, or PA02.
   */
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 |
                       ADC_INPUTCTRL_MUXNEG_GND |
                       ADC_INPUTCTRL_MUXPOS_PIN0;


  /* Use the internal VCC reference. This is 1/2 of what's on VCCA.
   * since VCCA is typically 3.3v, this is 1.65v.
   */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  /* This is the default but sets the ADC into single ended mode
  */
  ADC->CTRLB.bit.DIFFMODE = 0;

  /* Enable interrupts */
  ADC->INTENSET.bit.RESRDY == 1;

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY);

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
  /* The first result should be thrown away, so let's just do that */
  ADC_Read();
  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 2);
}

void setup() {
  Serial.begin(9600);

  // Wire.begin(28);
  // Wire.onRequest(Request_Handler);
  // Wire.onReceive(Receive_Handler);

  __disable_irq();
  Clock_Init();
  ADC_Init();
  TC5_Init();
  __enable_irq();

  while (!Serial);
  Serial.println("QT Py Sensor");
}

void loop() { }

void TC5_Handler(void) {
  Serial.println("TC5_Handler");
  // start an ADC read
  ADC->SWTRIG.bit.START = true;

  // reset the timer interrupt
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}

void ADC_Handler(void) {
  Serial.print("V: ");
  Serial.print(ADC->RESULT.reg);
  Serial.print("\n\r");
  // reset the adc interrupt
  ADC->INTFLAG.bit.RESRDY = 1;
}

// void Request_Handler() {
//   Wire.write("hello");
// }
// 
// void Receive_Handler(int numBytes) {
//   while (Wire.available()) {
//     Wire.read();
//   }
// }

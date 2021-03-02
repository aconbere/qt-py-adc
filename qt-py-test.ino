#include <sam.h>
#include <Wire.h>

#define VOLT_SENSE PORT_PA02
#define CURRENT_SENSE PORT_PA03
#define MULTISAMPLE false

void clockInit(void) {
  SYSCTRL->OSC8M.bit.PRESC = 0;                      // no prescaler (is 8 on reset)
  SYSCTRL->OSC8M.bit.ENABLE = 1;                     // enable source

  GCLK->GENDIV.bit.ID = 1;                           // select GCLK_GEN[1]
  GCLK->GENDIV.bit.DIV = 0;                          // no prescaler

  GCLK->GENCTRL.bit.ID = 1;                          // select GCLK_GEN[1]
  GCLK->GENCTRL.bit.GENEN = 1;                       // enable generator
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_SRC_OSC8M;       // OSC8M source

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM0_CORE | // SERCOM0 peripheral channel
                      GCLK_CLKCTRL_GEN_GCLK1 |       // select source GCLK_GEN[1]
                      GCLK_CLKCTRL_CLKEN;

  PM->APBCSEL.bit.APBCDIV = 0;                       // no prescaler
  PM->APBCMASK.bit.ADC_ = 1;                         // Enable clocking for the ADC */
  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

uint32_t readAdc() {
  /* Start the ADC using a software trigger. */
  ADC->SWTRIG.bit.START = true;

  /* Wait for the result ready flag to be set. */
  while (ADC->INTFLAG.bit.RESRDY == 0);

  /* Clear the flag. */
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  /* Read the value. */
  return ADC->RESULT.reg;
}

void tc5Init() {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_TC4_TC5) |
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

   // Configure synchronous bus clock
  PM->APBCSEL.bit.APBCDIV = 0; // no prescaler
  PM->APBCMASK.bit.TC5_ = 1; // enable TC5 interface

  // Configure Count Mode (16-bit)
  TC5->COUNT16.CTRLA.bit.MODE = 0x0;

  // Configure Prescaler for divide by 2 (500kHz clock to COUNT)
  TC5->COUNT16.CTRLA.bit.PRESCALER = 0x1;

  // Configure TC5 Compare Mode for compare channel 0
  TC5->COUNT16.CTRLA.bit.WAVEGEN = 0x1; // "Match Frequency" operation

  // Initialize compare value for 100mS @ 500kHz
  TC5->COUNT16.CC[0].reg = 50000;

  // Enable TC5 compare mode interrupt generation
  TC5->COUNT16.INTENSET.bit.MC0 = 0x1; // Enable match interrupts on compare channel 0 

  // Enable TC5
  TC5->COUNT16.CTRLA.bit.ENABLE = 1;

  // Wait until TC5 is enabled
  while(TC5->COUNT16.STATUS.bit.SYNCBUSY == 1);
  NVIC_SetPriority(TC5_IRQn, 3);
  NVIC_EnableIRQ(TC5_IRQn);
}

void adcInit() {
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

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC )| // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0   | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN;

  while (GCLK->STATUS.bit.SYNCBUSY);
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |    // Divide Clock by 512.
                   ADC_CTRLB_RESSEL_12BIT;         // 10 bits resolution as default

  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(10);

  while(ADC->STATUS.bit.SYNCBUSY);

  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0


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

   * TODO - If I switch to an external reference I'll need to update
   * the gain to ADC_INPUTCTRL_GAIN_1X
   */
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 |
                       ADC_INPUTCTRL_MUXNEG_GND |
                       ADC_INPUTCTRL_MUXPOS_PIN0;


  /* Use the internal VCC reference. This is 1/2 of what's on VCCA.
   * since VCCA is typically 3.3v, this is 1.65v.

   * TODO - Consider switching to an external reference source
   * ADC_REFCTRL_REFSEL_VREFA since this would expand the range to
   * 0 - 3V from 0 - 1.65V
   */
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  /* This is the default but sets the ADC into single ended mode
  */
  ADC->CTRLB.bit.DIFFMODE = 0;

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY);

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
  /* The first result should be thrown away, so let's just do that */
  readAdc();
  NVIC_EnableIRQ(ADC_IRQn);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  clockInit();
  adcInit();
  tc5Init();

  while (!Serial);
  Serial.println("QT Py Sensor");
}

void loop() {
  // Serial.print("V: ");
  // Serial.print(readAdc());
  // Serial.print("\n\r");
  ADC->SWTRIG.bit.START = true;
  delay(1000);
}

void ADC_Handler(void) {
  Serial.print("V: ");
  Serial.print(ADC->RESULT.reg);
  Serial.print("\n\r");
}

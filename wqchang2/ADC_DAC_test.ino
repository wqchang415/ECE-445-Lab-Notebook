// Define ADC and DAC pins
#define ADC_PIN 34  // ADC input pin (e.g., GPIO34)
#define DAC_CH1 25   // DAC output pin (e.g., GPIO25)

void setup() {
  // Set up ADC (Optional: adjust if you want a different resolution)
  analogReadResolution(8);  // 12-bit resolution (0-4095)
}

void loop() {
  // Read the ADC value (from the ADC_PIN)
  int adcValue = analogRead(ADC_PIN); 

  // Map the ADC value to DAC range (0-255 for 8-bit DAC output)
  int dacValue = map(adcValue, 0, 255, 0, 255);

  // Output the mapped value to the DAC
  dacWrite(DAC_CH1, dacValue);
}

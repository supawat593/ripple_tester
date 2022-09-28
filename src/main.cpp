#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <chrono>
#include <driver/adc.h>

#define PIN_HALF_A 25
#define PIN_1_A 26
#define PIN_2_A 27
#define PIN_V 14

unsigned long Start;
unsigned short Count, CountN;
int lcdColumns = 16;
int lcdRows = 2;
String volt, ripple;
// float c1,c2,c3,current;
// int voltR = 0, min_voltR = 4095, max_voltR = 0, rippleR = 0;
// int voltN = 0, min_voltN = 4095, max_voltN = 0, rippleN = 0;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_HALF_A, INPUT);
  pinMode(PIN_1_A, INPUT);
  pinMode(PIN_2_A, INPUT);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // ADC1_CHANNEL_5-->33 nagative peak
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // ADC1_CHANNEL_7-->35 positive peak
  Start = millis();
  lcd.init();
  lcd.backlight();
}

void loop()
{
  // put your main code here, to run repeatedly:
  float ripple = 0.0;
  int voltR = 0, min_voltR = 4095, max_voltR = 0, rippleR = 0;
  int voltN = 0, min_voltN = 4095, max_voltN = 0, rippleN = 0;
  float c1 = 0.0, c2 = 0.0, c3 = 0.0, current = 0.0;
  float volts = 0;
  int adc = 0;
  float adc_f = 0.00, sum1 = 0.00, sum2 = 0.00;
  long adc_sum = 0, sumR = 0, sumN = 0;
  const uint16_t samples = 30000;
  const uint16_t samplesN = 30000;
  for (uint32_t i = 0; i < 10; i++)
  {
    adc = analogRead(PIN_V);
    adc_sum = adc_sum + adc;
  }
  adc_f = adc_sum / 10;
  //  for (uint32_t i = 0; i < samples; i++) {
  //    voltR = adc1_get_raw(ADC1_CHANNEL_7);
  //    if ((voltR < min_voltR) && (voltR != 0)) {
  //      min_voltR = voltR;
  //    } else {
  //      max_voltR = voltR;
  //    }
  //    ++Count;
  //  }
  //  for (uint32_t i = 0; i < samplesN; i++) {
  //    voltN = adc1_get_raw(ADC1_CHANNEL_5);
  //    if ((voltN < min_voltN) && (voltN != 0)) {
  //      min_voltN = voltN;
  //    } else {
  //      max_voltN = voltN;
  //    }
  //    ++CountN;
  //  }
  int vmin = 4095, vmax = 0;
  for (uint32_t i = 0; i < samples; i++)
  {
    voltR = adc1_get_raw(ADC1_CHANNEL_7);
    // voltR = voltR - 1200;
    // voltR = voltR - 1235;
    // voltR = voltR - 1620;
    // sumR = sumR + voltR;
    // voltR = voltR - 1620;
    if (voltR < vmin)
    {
      vmin = voltR;
    }
    if (voltR > vmax)
    {
      vmax = voltR;
    }
    sumR += voltR;
    ++Count;
  }
  sum1 = sumR / 30000;
  // sum1 /= 2;
  // sum1 -= 1200;
  sum1=vmax-2267;
  // sum1 /= 4;
  Serial.print("vmin = ");
  Serial.println(vmin);
  Serial.print("vmax = ");
  Serial.println(vmax);
  Serial.print("sum1 = ");
  Serial.println(sum1);

  for (uint32_t i = 0; i < samplesN; i++)
  {
    voltN = adc1_get_raw(ADC1_CHANNEL_5);
    voltN = voltN - 1235;
    // voltN = voltN - 1365;
    sumN = sumN + voltN;
    ++CountN;
  }
  sum2 = sumN / 30000;
  Serial.println(sum2);

  ripple = (sum1 + sum2) * 3300 / 4095;
  // ripple*=0.25;  //  re-scale
  if (digitalRead(PIN_HALF_A) == HIGH)
  {
    c1 = 0.5;
  }
  if (digitalRead(PIN_1_A) == HIGH)
  {
    c2 = 1.0;
  }
  if (digitalRead(PIN_2_A) == HIGH)
  {
    c3 = 2.0;
  }
  else
  {
    current = 0.0;
  }
  current = c1 + c2 + c3;
  // set cursor to first column, first row
  //  Serial.println("############");
  //  Serial.println(max_voltN);
  //  Serial.println(min_voltN);
  //  Serial.println(max_voltN-min_voltN);
  //  Serial.println("-------------");
  //  Serial.println(max_voltR);
  //  Serial.println(min_voltR);
  //  Serial.println(max_voltR-min_voltR);
  //  Serial.println("-------------");
  //  Serial.println(((max_voltR-min_voltR)+(max_voltN-min_voltN))*3300/4095);
  //  Serial.println("############");
  lcd.setCursor(0, 0);
  lcd.print(current);
  lcd.setCursor(3, 0);
  // print message
  lcd.print("A");
  lcd.setCursor(6, 0);
  lcd.print("Vin=");
  lcd.setCursor(10, 0);
  lcd.print(adc_f / 100); //-->100 calibrate
  // lcd.print(volt);
  lcd.setCursor(15, 0);
  lcd.print("V");
  // set cursor to first column, second row
  lcd.setCursor(0, 1);
  lcd.print("Ripple=");
  lcd.setCursor(7, 1);
  lcd.print(ripple / 3.8);
  lcd.setCursor(11, 1);
  lcd.print("mVp-p");
  delay(1000);
  // lcd.clear();  // for clear display LCD
}
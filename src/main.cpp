#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <driver/adc.h>
#include <uptime_formatter.h>

#define N_AVG 250
#define PIN_HALF_A 25
#define PIN_1_A 26
#define PIN_2_A 27
#define PIN_V 14

// unsigned short Count, CountN;
int lcdColumns = 16;
int lcdRows = 2;
String volt, ripple;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

QueueHandle_t qu_task;
TaskHandle_t xHandle = NULL, xHandle2 = NULL;

typedef struct ANALOG_READ
{
  uint8_t pin;
  uint16_t value;
} _analog_read;

// Function

float currentMeas()
{
  float c1 = 0.0, c2 = 0.0, c3 = 0.0;

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

  return c1 + c2 + c3;
}

void display(int pos, int neg, int level)
{
  float ripple = 0.0, current = 0.0, adc_f = 0.0;
  ripple = (pos + neg) * 3300 / 4095;
  adc_f = level / 1.0;
  current = currentMeas();

  Serial.print("ripple = ");
  Serial.println(ripple / 1.00);
  Serial.println("current : " + String(current));

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
  //   delay(1000);
  //   // lcd.clear();  // for clear display LCD
}

// Task Function

void vGetQueueTask(void *pvParameters)
{
  int getvalue;
  _analog_read data;
  uint8_t count1 = 0;
  uint8_t count2 = 0;
  uint32_t sum1 = 0;
  uint32_t sum2 = 0;

  while (true)
  {
    if (xQueueReceive(qu_task, (void *)&data, 0) == pdTRUE)
    {
      // Serial.println("pin:" + String(data.pin) + " value: " + String(data.value));
      // Serial.println("up " + uptime_formatter::getUptime());

      if (data.pin == 1)
      {
        sum1 += data.value;
        count1++;

        // if (count1 >= N_AVG)
        // {
        //   Serial.println("data1:" + String(sum1 / N_AVG));
        //   Serial.println("up " + uptime_formatter::getUptime());
        //   sum1 = 0;
        //   // sum2 = 0;
        //   count1 = 0;
        //   // count2 = 0;
        // }
      }
      if (data.pin == 2)
      {
        sum2 += data.value;
        count2++;

        // if (count2 >= N_AVG)
        // {
        //   Serial.println("data2: " + String(sum2 / N_AVG));
        //   Serial.println("up " + uptime_formatter::getUptime());
        //   // sum1 = 0;
        //   sum2 = 0;
        //   // count1 = 0;
        //   count2 = 0;
        // }
      }

      if ((count1 >= N_AVG) && (count2 >= N_AVG))
      {
        long adc_sum = 0;
        uint16_t pos_pk = 0, neg_pk = 0, adc_12v = 0, adc_12f = 0;

        for (uint32_t i = 0; i < 10; i++)
        {
          adc_12v = analogRead(PIN_V);
          adc_sum += adc_12v;
        }

        adc_12f = adc_sum / 10;
        pos_pk = sum1 / N_AVG;
        neg_pk = sum2 / N_AVG;

        Serial.println("data_12v: " + String(adc_12f));
        Serial.println("data1: " + String(pos_pk) + " " + "data2: " + String(neg_pk));
        Serial.println("up " + uptime_formatter::getUptime());

        display(pos_pk, neg_pk, adc_12f);

        adc_sum = 0;
        sum1 = 0;
        sum2 = 0;
        count1 = 0;
        count2 = 0;
      }
    }
    vTaskDelay(3 / portTICK_PERIOD_MS);
  }
}

void vPutQueueTask(void *pvParameters)
{
  _analog_read data;
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // ADC1_CHANNEL_7-->35 positive peak

  while (true)
  {
    data.pin = 1;
    data.value = adc1_get_raw(ADC1_CHANNEL_7);
    xQueueSend(qu_task, (void *)&data, 0);
    vTaskDelay(8 / portTICK_PERIOD_MS);
  }
  vTaskDelete(xHandle);
}

void vPutQueueTask2(void *pvParameters)
{
  _analog_read data;
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // ADC1_CHANNEL_5-->33 nagative peak

  while (true)
  {
    data.pin = 2;
    data.value = adc1_get_raw(ADC1_CHANNEL_5);
    xQueueSend(qu_task, (void *)&data, 0);
    vTaskDelay(8 / portTICK_PERIOD_MS);
  }
  vTaskDelete(xHandle2);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("up " + uptime_formatter::getUptime());

  pinMode(PIN_HALF_A, INPUT);
  pinMode(PIN_1_A, INPUT);
  pinMode(PIN_2_A, INPUT);

  adc1_config_width(ADC_WIDTH_BIT_12);
  // adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // ADC1_CHANNEL_5-->33 nagative peak
  // adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // ADC1_CHANNEL_7-->35 positive peak

  lcd.init();
  lcd.backlight();

  qu_task = xQueueCreate(2 * N_AVG, sizeof(int));
  xTaskCreate(vGetQueueTask, "vGetQueueTask", 1600, NULL, tskIDLE_PRIORITY - 1, NULL);
  xTaskCreate(vPutQueueTask, "vPutQueueTask", 1500, NULL, tskIDLE_PRIORITY - 2, &xHandle);
  xTaskCreate(vPutQueueTask2, "vPutQueueTask2", 1500, NULL, tskIDLE_PRIORITY - 2, &xHandle2);
}

void loop()
{
  // put your main code here, to run repeatedly:
}
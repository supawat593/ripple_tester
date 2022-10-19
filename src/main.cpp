#include <Arduino.h>
#include <driver/adc.h>
#include <LiquidCrystal_I2C.h>

// #define NREGAIN(x) 4e-5 * x *x + 0.1178 * x + 22.914
// #define PREGAIN(x) 3e-5 * x *x + 0.1389 * x + 19.365
#define NREGAIN(x) 8e-6 * x *x + 0.1004 * x + 5.6466 // y = 8E-06x2 + 0.1004x + 5.6466
#define PREGAIN(x) 4e-6 * x *x + 0.1089 * x + 4.3485 // y = 4E-06x2 + 0.1089x + 4.3485
#define NVREF_COMP(x) (int)(169 - 0.008 * x)
#define N_AVG 20

#define PIN_HALF_A 25
#define PIN_1_A 26
#define PIN_2_A 27
#define PIN_V 14

typedef struct ANALOG_READ
{
  uint8_t number;
  uint16_t value;
} _analog_read;

typedef struct PRINT_SCR
{
  float dCurrent;
  uint16_t pk_pos;
  uint16_t pk_neg;
  uint16_t level;
} _print_scr;

static SemaphoreHandle_t mutex;
SemaphoreHandle_t sem_one = NULL;
QueueHandle_t qu_task;
TaskHandle_t xHandle = NULL, xHandle2 = NULL, xHandle3 = NULL;

_print_scr data_scr = {0.0, 0, 0, 0};
int lcdColumns = 16;
int lcdRows = 2;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void TaskPrintSCR(void *pvParameters);
void vGetQueueTask(void *pvParameters);
void vPutQueueTask(void *pvParameters);
void vPutQueueTask2(void *pvParameters);
void vPutQueueTask3(void *pvParameters);

float read_ratingA()
{
  return 0.5 * ((digitalRead(PIN_HALF_A)) | (digitalRead(PIN_1_A) << 1) | (digitalRead(PIN_2_A) << 2));
}

void loop_switch(void *pvParameter)
{
  while (true)
  {
    static int dCurrent = 0;
    xSemaphoreTake(sem_one, portMAX_DELAY);

    Serial.print("click!!! : ");
    Serial.println(millis());

    dCurrent = (digitalRead(PIN_HALF_A)) | (digitalRead(PIN_1_A) << 1) | (digitalRead(PIN_2_A) << 2);

    // Serial.println("dCurrent " + String(dCurrent));
    // Serial.println("pinA " + String((dCurrent & 0b0001)));
    // Serial.println("pinB " + String((dCurrent & 0b0010) >> 1));
    // Serial.println("pinC " + String((dCurrent & 0b0100) >> 2));
    // Serial.println("amp_rating : " + String(0.5 * dCurrent));
    // vTaskDelay(50 / portTICK_PERIOD_MS);

    xSemaphoreTake(mutex, portMAX_DELAY);
    data_scr.dCurrent = 0.5 * dCurrent;
    xSemaphoreGive(mutex);
  }
  vTaskDelete(NULL);
}

void IRAM_ATTR onFalling()
{
  static unsigned long last_millis = 0;

  if (millis() - last_millis > 250)
  {
    xSemaphoreGive(sem_one);
    last_millis = millis();
  }
}

void IRAM_ATTR onFalling2()
{
  static unsigned long last_millis = 0;

  if (millis() - last_millis > 250)
  {
    xSemaphoreGive(sem_one);
    last_millis = millis();
  }
}

void IRAM_ATTR onFalling3()
{
  static unsigned long last_millis = 0;

  if (millis() - last_millis > 250)
  {
    xSemaphoreGive(sem_one);
    last_millis = millis();
  }
}

void display(_print_scr *data)
{
  float ripple = 0.0, current = 0.0, adc_f = 0.0;
  float neg_vp = 0.0, pos_vp = 0.0;

  // ripple = (data->pk_pos + data->pk_neg) * 3300 / 4095;
  // ripple /= 4.38; // cal back gain  G=4.7 (47k/10k)and  divider between 47k with 4.7k parallel 100k
  // ripple /= 8.62; // cal back gain  G1=4.7 (47k/10k)  G2=2 (10k/10k) and  divider between 47k with 4.7k parallel 100k
  // ripple = (float)REGAIN((float)ripple);

  neg_vp = (data->pk_neg) * 3300 / 4095;
  pos_vp = (data->pk_pos) * 3300 / 4095;
  Serial.println("vdet-> pos_vp: " + String(pos_vp) + " neg_vp: " + String(neg_vp));

  neg_vp = (float)NREGAIN((float)neg_vp);
  pos_vp = (float)PREGAIN((float)pos_vp);
  ripple = neg_vp + pos_vp;
  Serial.println("Vpk_test-> pos_vp: " + String(pos_vp) + " neg_vp: " + String(neg_vp));
  Serial.println("Vpk_test-> ripple mVp-p: " + String(ripple));

  adc_f = data->level;
  adc_f += (float)NVREF_COMP((int)adc_f);
  adc_f = adc_f * 3300 / 4095;
  adc_f *= 11.93; // cal back divider  divider between 47k with 4.7k parallel 100k

  current = data->dCurrent;

  Serial.print("ripple = ");
  Serial.println(ripple / 1.00);
  // Serial.println("current : " + String(current));

  lcd.setCursor(0, 0);
  lcd.print(current);
  lcd.setCursor(3, 0);
  // print message
  lcd.print("A");
  lcd.setCursor(6, 0);
  lcd.print("Vin=");
  lcd.setCursor(10, 0);
  // lcd.print(adc_f / 100); //-->100 calibrate
  lcd.print(adc_f / 1000); //-->1000 calibrate  mV to V
  // lcd.print(volt);
  lcd.setCursor(15, 0);
  lcd.print("V");
  // set cursor to first column, second row
  lcd.setCursor(0, 1);
  lcd.print("Ripple=");
  lcd.setCursor(7, 1);
  // lcd.print(ripple / 3.8);
  lcd.print(ripple);
  // lcd.print(ripple * 10.0);
  lcd.setCursor(11, 1);
  lcd.print("mVp-p");
  //   delay(1000);
  //   // lcd.clear();  // for clear display LCD
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_HALF_A, INPUT);
  pinMode(PIN_1_A, INPUT);
  pinMode(PIN_2_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_HALF_A), &onFalling, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_1_A), &onFalling2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_2_A), &onFalling3, CHANGE);

  data_scr.dCurrent = read_ratingA();

  adc1_config_width(ADC_WIDTH_12Bit);

  xTaskCreate(loop_switch, "loop_switch", 1500, NULL, tskIDLE_PRIORITY, NULL);
  sem_one = xSemaphoreCreateBinary();

  qu_task = xQueueCreate(20, sizeof(int));

  xTaskCreate(vGetQueueTask, "vGetQueueTask", 2000, NULL, tskIDLE_PRIORITY - 1, NULL);
  xTaskCreate(vPutQueueTask, "vPutQueueTask", 2000, NULL, tskIDLE_PRIORITY - 2, &xHandle);
  xTaskCreate(vPutQueueTask2, "vPutQueueTask2", 2000, NULL, tskIDLE_PRIORITY - 2, &xHandle2);
  xTaskCreate(vPutQueueTask3, "vPutQueueTask3", 2000, NULL, tskIDLE_PRIORITY - 2, &xHandle3);

  lcd.init();
  lcd.backlight();

  mutex = xSemaphoreCreateMutex();
  xTaskCreate(TaskPrintSCR, "TaskPrintSCR", 2000, NULL, tskIDLE_PRIORITY - 1, &xHandle3);
  xSemaphoreGive(mutex);
}

void loop()
{
  // put your main code here, to run repeatedly:
}

void TaskPrintSCR(void *pvParameters)
{
  while (true)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    display(&data_scr);
    Serial.println("---------------- uptime : " + String(millis()) + " ------------------------");
    Serial.println("print_scr : " + String(millis()));
    Serial.println("gpio : " + String(data_scr.dCurrent));
    Serial.println("level_12v : " + String(data_scr.level));
    Serial.println("pk_pos : " + String(data_scr.pk_pos));
    Serial.println("pk_neg : " + String(data_scr.pk_neg) + "\r\n");
    xSemaphoreGive(mutex);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void vGetQueueTask(void *pvParameters)
{

  _analog_read data;
  uint16_t count1 = 0;
  uint16_t count2 = 0;
  uint16_t count3 = 0;
  uint32_t sum_pos = 0;
  uint32_t sum_neg = 0;
  uint32_t sum_level = 0;

  while (true)
  {
    if (xQueueReceive(qu_task, (void *)&data, 0) == pdTRUE)
    {
      // Serial.println("pin:" + String(data.number) + " value: " + String(data.value));
      // Serial.println("up " + uptime_formatter::getUptime());
      // Serial.println("up " + millis());

      if (data.number == 1)
      {
        sum_pos += data.value;
        count1++;

        if (count1 >= N_AVG)
        {
          uint16_t pos_pk = 0, neg_pk = 0, adc_12v = 0, adc_12f = 0;

          pos_pk = (sum_pos) / (N_AVG);
          pos_pk += (uint16_t)NVREF_COMP((int)pos_pk);

          // Serial.println("---------------- uptime : " + String(millis()) + " ------------------------");
          // Serial.println("pkmin1: " + String(pkmin1) + " " + "pkmax1: " + String(pkmax1));
          // Serial.println("pkmin2: " + String(pkmin2) + " " + "pkmax2: " + String(pkmax2));
          // Serial.println("levelmin: " + String(levelmin) + " " + "levelmax: " + String(levelmax));
          // Serial.println("data_12v_level: " + String(adc_12v));
          // Serial.println("\r\nNew Data ------- uptime : " + String(millis()) + " ------------------------");
          // Serial.println("data1: " + String(pos_pk) + " " + "data2: " + String(neg_pk) + " " + "data_12v: " + String(adc_12v));
          // Serial.println("-------------------------------------------------------\r\n");

          xSemaphoreTake(mutex, portMAX_DELAY);
          data_scr.pk_pos = pos_pk;
          xSemaphoreGive(mutex);

          sum_pos = 0;
          count1 = 0;
        }
      }
      if (data.number == 2)
      {
        sum_neg += data.value;
        count2++;

        if (count2 >= N_AVG)
        {
          uint16_t pos_pk = 0, neg_pk = 0, adc_12v = 0, adc_12f = 0;

          neg_pk = (sum_neg) / (N_AVG);
          neg_pk += (uint16_t)NVREF_COMP((int)neg_pk);

          // Serial.println("---------------- uptime : " + String(millis()) + " ------------------------");
          // Serial.println("pkmin1: " + String(pkmin1) + " " + "pkmax1: " + String(pkmax1));
          // Serial.println("pkmin2: " + String(pkmin2) + " " + "pkmax2: " + String(pkmax2));
          // Serial.println("levelmin: " + String(levelmin) + " " + "levelmax: " + String(levelmax));
          // Serial.println("data_12v_level: " + String(adc_12v));
          // Serial.println("\r\nNew Data ------- uptime : " + String(millis()) + " ------------------------");
          // Serial.println("data1: " + String(pos_pk) + " " + "data2: " + String(neg_pk) + " " + "data_12v: " + String(adc_12v));
          // Serial.println("-------------------------------------------------------\r\n");

          xSemaphoreTake(mutex, portMAX_DELAY);
          data_scr.pk_neg = neg_pk;
          xSemaphoreGive(mutex);

          sum_neg = 0;
          count2 = 0;
        }
      }
      if (data.number == 3)
      {
        sum_level += data.value;
        count3++;

        if (count3 >= N_AVG)
        {
          uint16_t pos_pk = 0, neg_pk = 0, adc_12v = 0, adc_12f = 0;

          adc_12v = (sum_level) / (N_AVG);
          adc_12v += (uint16_t)NVREF_COMP((int)adc_12v);

          // Serial.println("---------------- uptime : " + String(millis()) + " ------------------------");
          // Serial.println("pkmin1: " + String(pkmin1) + " " + "pkmax1: " + String(pkmax1));
          // Serial.println("pkmin2: " + String(pkmin2) + " " + "pkmax2: " + String(pkmax2));
          // Serial.println("levelmin: " + String(levelmin) + " " + "levelmax: " + String(levelmax));
          // Serial.println("data_12v_level: " + String(adc_12v));
          // Serial.println("\r\nNew Data ------- uptime : " + String(millis()) + " ------------------------");
          // Serial.println("data1: " + String(pos_pk) + " " + "data2: " + String(neg_pk) + " " + "data_12v: " + String(adc_12v));
          // Serial.println("-------------------------------------------------------\r\n");

          xSemaphoreTake(mutex, portMAX_DELAY);
          data_scr.level = adc_12v;
          xSemaphoreGive(mutex);

          sum_level = 0;
          count3 = 0;
        }
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void vPutQueueTask(void *pvParameters)
{
  _analog_read data;
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // ADC1_CHANNEL_7-->35 positive peak

  while (true)
  {
    uint16_t read = 0;
    uint16_t max_value = 0;
    for (int i = 0; i < 20; i++)
    {
      read = adc1_get_raw(ADC1_CHANNEL_7);
      if (read > max_value)
      {
        max_value = read;
      }
      vTaskDelay(5 / portTICK_PERIOD_MS);
      // delay(10);
    }

    data.number = 1;
    // data.value = adc1_get_raw(ADC1_CHANNEL_7);
    data.value = max_value;
    // Serial.println("raw_pos : " + String(data.value));
    xQueueSend(qu_task, (void *)&data, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(xHandle);
}

void vPutQueueTask2(void *pvParameters)
{
  _analog_read data;
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // ADC1_CHANNEL_5-->33 negative peak

  while (true)
  {
    uint16_t read = 0;
    uint16_t max_value = 0;
    for (int i = 0; i < 20; i++)
    {
      read = adc1_get_raw(ADC1_CHANNEL_5);
      if (read > max_value)
      {
        max_value = read;
      }
      vTaskDelay(5 / portTICK_PERIOD_MS);
      // delay(10);
    }

    // Serial.println("Negative peak -------- uptime : " + String(millis()) + " -----> " + String(max_value));

    data.number = 2;
    // data.value = adc1_get_raw(ADC1_CHANNEL_5);
    data.value = max_value;
    // Serial.println("raw_neg : " + String(data.value));
    xQueueSend(qu_task, (void *)&data, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(xHandle);
}

void vPutQueueTask3(void *pvParameters)
{
  _analog_read data;

  while (true)
  {
    data.number = 3;
    data.value = analogRead(PIN_V);
    // Serial.println("raw_12v : " + String(data.value));
    xQueueSend(qu_task, (void *)&data, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(xHandle);
}
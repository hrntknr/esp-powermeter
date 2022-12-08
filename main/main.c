#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_tls.h"
#include "mqtt_client.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"

#include "config.h"

const char *TAG = "main";

#define ADC_ATTEN ADC_ATTEN_DB_11 // 0-2600mV
#define ADC_CH_1 ADC_CHANNEL_0
#define ADC_CH_2 ADC_CHANNEL_1
#define ADC_UNIT ADC_UNIT_1
#define ADC_SAMPLE_FREQ_HZ 50 * 100
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_BUF_LEN 1024

#define R1 100
#define CT_RATIO 1000
#define POWER_FACTOR 1

extern const uint8_t root_ca_pem_start[] asm("_binary_root_ca_pem_start");
extern const uint8_t root_ca_pem_end[] asm("_binary_root_ca_pem_end");

static TaskHandle_t collectorHandle = NULL;
static uint8_t adc_buf[ADC_BUF_LEN];
static uint32_t sample_buf_1[ADC_SAMPLE_FREQ_HZ / 2];
static uint32_t sample_buf_2[ADC_SAMPLE_FREQ_HZ / 2];
static uint32_t sample_buf_index_1 = 0;
static uint32_t sample_buf_index_2 = 0;

static bool mqtt_connected = false;
static bool wifi_connected = false;
static esp_netif_t *sta_netif = NULL;
static esp_mqtt_client_handle_t client = NULL;
static TaskHandle_t mqtt_init_handle = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
  {
    esp_netif_create_ip6_linklocal(sta_netif);
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGI(TAG, "Disconnected from AP, trying to reconnect...");
    esp_wifi_connect();
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    wifi_connected = true;
    if (mqtt_init_handle != NULL)
    {
      xTaskNotifyGive(mqtt_init_handle);
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_GOT_IP6)
  {
    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    ESP_LOGI(TAG, "got ip6:" IPV6STR, IPV62STR(event->ip6_info.ip));
    wifi_connected = true;
    if (mqtt_init_handle != NULL)
    {
      xTaskNotifyGive(mqtt_init_handle);
    }
  }
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_id == MQTT_EVENT_CONNECTED)
  {
    ESP_LOGI(TAG, "connected to mqtt broker");
    mqtt_connected = true;
  }
  else if (event_id == MQTT_EVENT_DISCONNECTED)
  {
    ESP_LOGI(TAG, "disconnected from mqtt broker");
    mqtt_connected = false;
  }
  else if (event_id == MQTT_EVENT_BEFORE_CONNECT)
  {
    ESP_LOGI(TAG, "connecting to mqtt broker");
    mqtt_connected = false;
  }
  else if (event_id == MQTT_EVENT_ERROR)
  {
    ESP_LOGI(TAG, "mqtt error");
    mqtt_connected = false;
  }
}

esp_err_t wifi_init()
{
  sta_netif = esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = WIFI_SSID,
          .password = WIFI_PASSWORD,
          .threshold.authmode = WIFI_AUTHMODE,
          .listen_interval = 3,
      },
  };
  ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode failed");
  ESP_RETURN_ON_ERROR(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config), TAG, "esp_wifi_set_config failed");
  ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL), TAG, "esp_event_handler_instance_register failed");
  ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL), TAG, "esp_event_handler_instance_register failed");
  ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");
  ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(8), TAG, "esp_wifi_set_max_tx_power failed");
  ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_MAX_MODEM), TAG, "esp_wifi_set_ps failed");
  return ESP_OK;
}

esp_err_t mqtt_init()
{
  mqtt_init_handle = xTaskGetCurrentTaskHandle();

  ESP_RETURN_ON_ERROR(esp_tls_init_global_ca_store(), TAG, "esp_tls_init_global_ca_store failed");
  ESP_RETURN_ON_ERROR(esp_tls_set_global_ca_store(root_ca_pem_start, root_ca_pem_end - root_ca_pem_start), TAG, "esp_tls_set_global_ca_store failed");

  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = MQTT_BROKER_URL,
      .broker.verification.use_global_ca_store = true,
      .credentials.username = MQTT_USERNAME,
      .credentials.authentication.password = MQTT_PASSWORD,
  };
  client = esp_mqtt_client_init(&mqtt_cfg);
  ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, &mqtt_event_handler, NULL), TAG, "esp_event_handler_instance_register failed");

  if (!wifi_connected)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  return esp_mqtt_client_start(client);
}

esp_err_t adc_init(adc_continuous_handle_t *handle, adc_cali_handle_t *cali_handle)
{
  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = ADC_BUF_LEN,
  };
  ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&adc_config, handle), TAG, "adc_continuous_new_handle failed");

  adc_digi_pattern_config_t adc_pattern[2];
  adc_pattern[0].atten = ADC_ATTEN;
  adc_pattern[0].channel = ADC_CH_1;
  adc_pattern[0].unit = ADC_UNIT;
  adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  adc_pattern[1].atten = ADC_ATTEN;
  adc_pattern[1].channel = ADC_CH_2;
  adc_pattern[1].unit = ADC_UNIT;
  adc_pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  adc_continuous_config_t dig_cfg = {
      .pattern_num = 2,
      .adc_pattern = adc_pattern,
      .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
      .conv_mode = ADC_CONV_MODE,
      .format = ADC_OUTPUT_TYPE,
  };
  ESP_RETURN_ON_ERROR(adc_continuous_config(*handle, &dig_cfg), TAG, "adc_continuous_config failed");

  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT,
      .atten = ADC_ATTEN,
      .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
  };
  ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_config, cali_handle), TAG, "adc_cali_create_scheme_curve_fitting failed");

  ESP_RETURN_ON_ERROR(adc_continuous_start(*handle), TAG, "adc_continuous_start failed");

  return ESP_OK;
}

void collector(void *pvParameters)
{
  uint32_t buf_sum, buf_avg, mvavg, w1, w2;
  while (1)
  {
    buf_sum = 0;
    for (int i = 0; i < ADC_SAMPLE_FREQ_HZ; i++)
    {
      buf_sum += sample_buf_1[i];
    }
    buf_avg = buf_sum / (ADC_SAMPLE_FREQ_HZ);
    buf_sum = 0;
    for (int i = 0; i < ADC_SAMPLE_FREQ_HZ; i++)
    {
      buf_sum += abs((int)sample_buf_1[i] - (int)buf_avg);
    }
    mvavg = buf_sum / (ADC_SAMPLE_FREQ_HZ);
    w1 = (mvavg * CT_RATIO * 100 / 1000 / R1) * POWER_FACTOR;

    buf_sum = 0;
    for (int i = 0; i < ADC_SAMPLE_FREQ_HZ; i++)
    {
      buf_sum += sample_buf_2[i];
    }
    buf_avg = buf_sum / (ADC_SAMPLE_FREQ_HZ);
    buf_sum = 0;
    for (int i = 0; i < ADC_SAMPLE_FREQ_HZ; i++)
    {
      buf_sum += abs((int)sample_buf_2[i] - (int)buf_avg);
    }
    mvavg = buf_sum / (ADC_SAMPLE_FREQ_HZ);
    w2 = (mvavg * CT_RATIO * 100 / 1000 / R1) * POWER_FACTOR;

    ESP_LOGI(TAG, "w1: %ld, w2: %ld", w1, w2);
    if (mqtt_connected)
    {
      char power_str[16] = {};
      sprintf(power_str, "%ld", w1 + w2);
      ESP_ERROR_CHECK(esp_mqtt_client_publish(client, MQTT_TOPIC_POWER_SOURCE, power_str, 0, 0, 0));
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(wifi_init());
  ESP_ERROR_CHECK(mqtt_init());

  adc_continuous_handle_t handle = NULL;
  adc_cali_handle_t cali_handle = NULL;
  ESP_ERROR_CHECK(adc_init(&handle, &cali_handle));

  xTaskCreate(collector, "collector", 8192, NULL, 1, &collectorHandle);

  esp_err_t ret;
  uint32_t ret_num;
  int voltage;
  while (1)
  {
    vTaskDelay(1);
    ret = adc_continuous_read(handle, adc_buf, ADC_BUF_LEN, &ret_num, 0);
    if (ret == ESP_OK)
    {
      for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
      {
        adc_digi_output_data_t *p = (void *)&adc_buf[i];
        adc_cali_raw_to_voltage(cali_handle, p->type2.data, &voltage);
        if (p->type2.channel == ADC_CH_1)
        {
          sample_buf_1[sample_buf_index_1++] = (uint32_t)voltage;
          if (sample_buf_index_1 >= ADC_SAMPLE_FREQ_HZ)
          {
            sample_buf_index_1 = 0;
          }
        }
        else if (p->type2.channel == ADC_CH_2)
        {
          sample_buf_2[sample_buf_index_2++] = (uint32_t)voltage;
          if (sample_buf_index_2 >= ADC_SAMPLE_FREQ_HZ)
          {
            sample_buf_index_2 = 0;
          }
        }
        else
        {
          ESP_LOGE(TAG, "Unknown channel: %d", p->type2.channel);
        }
      }
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
      continue;
    }
    else
    {
      ESP_LOGE(TAG, "adc_continuous_read failed: %d", ret);
      break;
    }
  }
}

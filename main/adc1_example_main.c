
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include <esp_http_server.h>

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#include <sys/param.h>
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "esp_tls.h"
#include "esp_crt_bundle.h"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
 
/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation
 
/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

static const char *web_urls = "http://iot-vfbcooifca-uc.a.run.app/alerts";
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


int  MQGetPercentage(float rs_ro_ratio, float *pcurve);
int  MQGetGasPercentage(float rs_ro_ratio, int gas_id);
float MQRead();
float MQCalibration();
float MQResistanceCalculation(int raw_adc);
static int AnalogRead();

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void delay(uint32_t time)
{
    vTaskDelay(pdMS_TO_TICKS(time));
}


static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;


static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

/* FreeRTOS event group to signal when we are connected*/

static EventGroupHandle_t s_wifi_event_group;

static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

const int CONNECTED_BIT = BIT0;
static int s_retry_num = 0;


static const char *TAG = "ESP32_Server";

static RTC_DATA_ATTR char GOT_IP = false;
static RTC_DATA_ATTR char __SSID[32];
static RTC_DATA_ATTR char __PWD[64];

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }

    ESP_LOGI(TAG, "wifi_event_handler wifi_event_handler wifi_event_handler");
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        if (GOT_IP == false)
        {
            ESP_LOGI(TAG, "Sorry wrong PSW");
            memset(&__PWD[0], 0, sizeof(__PWD));
            memset(&__SSID[0], 0, sizeof(__SSID));
            esp_sleep_enable_timer_wakeup(100000);
            esp_deep_sleep_start();
        }
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP ");
        ESP_LOGI(TAG, "Login Success");
        GOT_IP = true;

        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t servePage_get_handler(httpd_req_t *req)
{
    httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html>");

    httpd_resp_sendstr_chunk(req, "<head>");
    httpd_resp_sendstr_chunk(req, "<style>");
    httpd_resp_sendstr_chunk(req, "form {display: grid;padding: 1em; background: #f9f9f9; border: 1px solid #c1c1c1; margin: 2rem auto 0 auto; max-width: 400px; padding: 1em;}}");
    httpd_resp_sendstr_chunk(req, "form input {background: #fff;border: 1px solid #9c9c9c;}");
    httpd_resp_sendstr_chunk(req, "form button {background: lightgrey; padding: 0.7em;width: 100%; border: 0;");
    httpd_resp_sendstr_chunk(req, "label {padding: 0.5em 0.5em 0.5em 0;}");
    httpd_resp_sendstr_chunk(req, "input {padding: 0.7em;margin-bottom: 0.5rem;}");
    httpd_resp_sendstr_chunk(req, "input:focus {outline: 10px solid gold;}");
    httpd_resp_sendstr_chunk(req, "@media (min-width: 300px) {form {grid-template-columns: 200px 1fr; grid-gap: 16px;} label { text-align: right; grid-column: 1 / 2; } input, button { grid-column: 2 / 3; }}");
    httpd_resp_sendstr_chunk(req, "</style>");
    httpd_resp_sendstr_chunk(req, "</head>");

    httpd_resp_sendstr_chunk(req, "<body>");
    httpd_resp_sendstr_chunk(req, "<form class=\"form1\" id=\"loginForm\" action=\"\">");

    httpd_resp_sendstr_chunk(req, "<label for=\"SSID\">WiFi Name</label>");
    httpd_resp_sendstr_chunk(req, "<input id=\"ssid\" type=\"text\" name=\"ssid\" maxlength=\"64\" minlength=\"4\">");

    httpd_resp_sendstr_chunk(req, "<label for=\"Password\">Password</label>");
    httpd_resp_sendstr_chunk(req, "<input id=\"pwd\" type=\"password\" name=\"pwd\" maxlength=\"64\" minlength=\"4\">");

    httpd_resp_sendstr_chunk(req, "<button>Submit</button>");
    httpd_resp_sendstr_chunk(req, "</form>");

    httpd_resp_sendstr_chunk(req, "<script>");
    httpd_resp_sendstr_chunk(req, "document.getElementById(\"loginForm\").addEventListener(\"submit\", (e) => {e.preventDefault(); const formData = new FormData(e.target); const data = Array.from(formData.entries()).reduce((memo, pair) => ({...memo, [pair[0]]: pair[1],  }), {}); var xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"http://192.168.4.1/connection\", true); xhr.setRequestHeader('Content-Type', 'application/json'); xhr.send(JSON.stringify(data)); document.getElementById(\"output\").innerHTML = JSON.stringify(data);});");
    httpd_resp_sendstr_chunk(req, "</script>");

    httpd_resp_sendstr_chunk(req, "</body></html>");

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t servePage = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = servePage_get_handler,
    .user_ctx = NULL};

void wifi_init_softap();
static esp_err_t psw_ssid_get_handler(httpd_req_t *req)
{
    char buf[128];
    int ret, remaining = req->content_len;

    while (remaining > 0)
    {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                                  MIN(remaining, sizeof(buf)))) <= 0)
        {
            if (ret == 0)
            {
                ESP_LOGI(TAG, "No content received please try again ...");
            }
            else if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {

                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "===================================="); 

        cJSON *root = cJSON_Parse(buf);

        sprintf(__SSID, "%s", cJSON_GetObjectItem(root, "ssid")->valuestring);
        sprintf(__PWD, "%s", cJSON_GetObjectItem(root, "pwd")->valuestring);

        remaining -= ret;

        nvs_handle_t my_handle;
    
       esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
        if (err != ESP_OK) {
            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        } else {
            printf("Done\n");
            err = nvs_set_str(my_handle, "BUFF", buf);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
            err = nvs_commit(my_handle);
            printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
        }
    }

    // End response
     httpd_resp_send_chunk(req, NULL, 0);
    abort();
    return ESP_OK;
}

static const httpd_uri_t psw_ssid = {
    .uri = "/connection",
    .method = HTTP_POST,
    .handler = psw_ssid_get_handler,
    .user_ctx = "TEST"};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &servePage);
        httpd_register_uri_handler(server, &psw_ssid);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void wifi_init_softap()
{
    size_t required_size = 0;

    nvs_handle_t my_handle;
    esp_err_t  err = nvs_open("storage", NVS_READWRITE, &my_handle);
        if (err != ESP_OK) {
            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        } else {
            printf("Done\n");
            nvs_get_str(my_handle, "BUFF", NULL, &required_size);
            char* BUFF = malloc(required_size);
            nvs_get_str(my_handle, "BUFF", BUFF, &required_size);
            printf("%d", required_size);
            if(required_size > 0)
            {
                switch (err) {
                    case ESP_OK:
                        printf("Done\n");
                        cJSON *root = cJSON_Parse(BUFF);
                        if(cJSON_GetObjectItem(root, "ssid"))
                       {
                            sprintf(__SSID, "%s", cJSON_GetObjectItem(root, "ssid")->valuestring);
                            sprintf(__PWD, "%s", cJSON_GetObjectItem(root, "pwd")->valuestring);
                       }
                        break;
                    case ESP_ERR_NVS_NOT_FOUND:
                        printf("The value is not initialized yet!\n");
                        break;
                    default :
                        printf("Error (%s) reading!\n", esp_err_to_name(err));
                }
            }
        }
    if (strlen(__PWD) && strlen(__SSID) != 0)
    {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "isolated",
            .password = "noor@4321",
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);


    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
    }
    else
    {
        s_wifi_event_group = xEventGroupCreate();

       ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_ap();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            NULL));

        wifi_config_t wifi_config = {
            .ap = {
                .ssid = "ESP32",
                .ssid_len = strlen("ESP32"),
                .password = "12345678",
                .max_connection = 5,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            },
        };
        // if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        //     wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        // }

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "wifi_init_softap finished. ");
    }

    start_webserver();
}

static int AnalogRead()
{
    int adc_reading = 0;
            if (unit == ADC_UNIT_1) {
                adc_reading = adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading = raw;
                }
        // printf("Raw: %d\n", adc_reading);
        adc_reading = map(adc_reading, 0, 4095, 0, 1023);
        return adc_reading;
}

  /****************** MQResistanceCalculation **************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration **************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration()
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(AnalogRead());
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}
/*****************************  MQRead *******************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead()
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(AnalogRead());
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage ********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 
/*****************************  MQGetPercentage ********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

//====-------------------------------------------

static void ADC_READ(void *pvParameters)
{
     check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    while (1) {
        int adc_reading = 0;
        //Multisampling
        adc_reading = AnalogRead();
        printf("Raw: %d      LPG = %d ppm\n", adc_reading, MQGetGasPercentage(MQRead()/Ro,GAS_LPG));
        printf("CO: %d ppm  SMOKE = %d ppm\n", MQGetGasPercentage(MQRead()/Ro,GAS_CO), MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
        vTaskDelay(pdMS_TO_TICKS(1000));
        //------------------HTTP request
       /* if(GOT_IP) {
            esp_tls_cfg_t cfg = {
                    .crt_bundle_attach = esp_crt_bundle_attach,
                };

                struct esp_tls *tls = esp_tls_conn_http_new(web_urls, &cfg);

                if (tls != NULL) {
                    ESP_LOGI(TAG, "Connection established to %s", web_urls);
                } else {
                    ESP_LOGE(TAG, "Could not connect to %s", web_urls);
                }

                esp_tls_conn_delete(tls);
        } */
    }

}
//============================================================
//====================APP_MAIN
//==========================================================
void app_main()
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

     wifi_init_softap();

    xTaskCreate(&ADC_READ, "ADC_READ", 8192, NULL, 5, NULL); 
    
}


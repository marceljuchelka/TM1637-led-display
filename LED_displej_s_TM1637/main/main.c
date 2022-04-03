/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_task_wdt.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ir_rx.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <netdb.h>
#include "main.h"

#include "sdkconfig.h"






#if witty == 0
	#define senzor_web_teplota	"&sensor[marcel_teplota_prace]="
	#define senzor_web_vlhkost	"&sensor[marcel_vlhkost_prace]="
	#define senzor_web_jas		"&sensor[marcel_jas_prace]="
	#define senzor_web_restart	"&sensor[marcel_restart]="
//	#define pin_red_led			15
//	#define pin_green_led		12
//	#define pin_blue_led		13

#else
	#define senzor_web_teplota	"&sensor[marcel_teplota_doma]="
	#define senzor_web_vlhkost	"&sensor[marcel_vlhkost_doma]="
	#define senzor_web_restart	"&sensor[marcel_restart]="
#endif



/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "wifi station";
static int s_retry_num = 0;

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "http://meter.v108b.com"
#define WEB_PORT 80
#define WEB_URL "http://meter.v108b.com/sensor/receive/?module=marcel"
//#define WEB_URL "http://meter.v108b.com/sensor/receive/?module=marcel&sensor[marcel_teplota]=22.0"

static const char *TAG1 = "example";
//static const char *HODNOTA11 = "hodnota11";

//static const char *REQUEST = "GET " WEB_URL " HTTP/1.0\r\n"
//    "Host: "WEB_SERVER"\r\n"
//    "User-Agent: esp-idf/1.0 esp32\r\n"
//    "\r\n";

//static const char	*http_web_url 	=	"GET " WEB_URL " HTTP/1.0\r\n"
//static const char	*http_host 		= 	"Host: "WEB_SERVER"\r\n";
//static const char	*http_user 		= 	"User-Agent: esp-idf/1.0 esp32\r\n r\n";

#define IR_RX_IO_NUM 0
#define IR_RX_BUF_LEN 128

static esp_err_t ir_rx_nec_code_check(ir_rx_nec_data_t nec_code)
{

    if ((nec_code.addr1 != ((~nec_code.addr2) & 0xff))) {
        return ESP_FAIL;
    }

    if ((nec_code.cmd1 != ((~nec_code.cmd2) & 0xff))) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void ir_rx_task(void *arg)
{
    ir_rx_nec_data_t ir_data;
    ir_rx_config_t ir_rx_config = {
        .io_num = IR_RX_IO_NUM,
        .buf_len = IR_RX_BUF_LEN
    };
    ir_rx_init(&ir_rx_config);

    while (1) {
        ir_data.val = 0;
        ir_rx_recv_data(&ir_data, 1, portMAX_DELAY);
        ESP_LOGI(TAG, "addr1: 0x%x, addr2: 0x%x, cmd1: 0x%x, cmd2: 0x%x", ir_data.addr1, ir_data.addr2, ir_data.cmd1, ir_data.cmd2);

        if (ESP_OK == ir_rx_nec_code_check(ir_data)) {
            ESP_LOGI(TAG, "ir rx nec data:  0x%x", ir_data.cmd1);
        } else {
            ESP_LOGI(TAG, "Non-standard nec infrared protocol");
        }
    }

    vTaskDelete(NULL);
}

/* priprava SMTP  */
static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    char *TAG = "SNTP";

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

esp_err_t tisk_casu(){
	char *TAG = "TISK CASU";

	time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    time(&now);
    localtime_r(&now, &timeinfo);

    // Je cas nastaven? (1970 - 1900).
        if (timeinfo.tm_year < (2016 - 1900)) {
            ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
            obtain_time();
        }

    /*nastaveni lokalniho casu na Cz */
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
     tzset();
     time(&now);
     localtime_r(&now, &timeinfo);
     strftime(strftime_buf, sizeof(strftime_buf), "%d.%m.%Y %H:%M", &timeinfo);
//     ESP_LOGI(TAG, "Datum a cas v CR je: %s", strftime_buf);
     printf(strftime_buf);
     ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());

	return 0;
}

static void sntp_example_task(void *arg)
{
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);
    localtime_r(&now, &timeinfo);

    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
    }

    // Set timezone to Eastern Standard Time and print local time
//    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
     tzset();

     // update 'now' variable with current time
     time(&now);
     localtime_r(&now, &timeinfo);

     if (timeinfo.tm_year < (2016 - 1900)) {
         ESP_LOGE(TAG, "The current date/time error");
     } else {
         strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
         ESP_LOGI(TAG, "Datum/time in CR is: %s", strftime_buf);
     }

     ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
     vTaskDelay(1000 / portTICK_RATE_MS);
}



static void http_get_task(uint8_t velicina,float* hodnota)
{
	char buf[90];
	char request[200];
	char *REQUEST = &request[0];
	ESP_LOGI(TAG1,"velicina %d hodnota %2.1f",velicina,*hodnota);

	if(velicina == teplota_wifi) sprintf(buf,"%s%s%2.1f",WEB_URL,senzor_web_teplota,*hodnota);
	if(velicina == vlhkost_wifi) sprintf(buf,"%s%s%2.1f",WEB_URL,senzor_web_vlhkost,*hodnota);
	if(velicina == restart_wifi) sprintf(buf,"%s%s%2.1f",WEB_URL,senzor_web_restart,*hodnota);
//	if(velicina == jas) sprintf(buf,"%s%s%2.1f",WEB_URL,senzor_web_jas,*hodnota);
//	printf("buf = %s\n",buf);
	sprintf(request,"GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: esp-idf/1.0 esp32\r\n"
			"\r\n", buf,WEB_SERVER);

//	printf(REQUEST);


	const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

//    while(0) {
        int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);
        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
//            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG1, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG1, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
//            continue;
        }
        ESP_LOGI(TAG1, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG1, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
//            continue;
        }

        ESP_LOGI(TAG1, "... connected");
        freeaddrinfo(res);

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG1, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
//            continue;
        }
        ESP_LOGI(TAG1, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG1, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
//            continue;
        }
        ESP_LOGI(TAG1, "... set socket receiving timeout success");
        close(s);

        /* Read HTTP response */
//        do {
//            bzero(recv_buf, sizeof(recv_buf));
//            r = read(s, recv_buf, sizeof(recv_buf)-1);
//            for(int i = 0; i < r; i++) {
////                putchar(recv_buf[i]);
//            }
//        } while(r > 0);
//
//        ESP_LOGI(TAG1, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
//        close(s);
//        for(int countdown = 1; countdown >= 0; countdown--) {
//            ESP_LOGI(TAG1, "%d... ", countdown);
//            vTaskDelay(1000 / portTICK_PERIOD_MS);
//        }
//        ESP_LOGI(TAG1, "Starting again!");
//    }
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
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


uint16_t test_num(uint8_t byte_hi, uint8_t byte_lo){
	uint16_t vysledek = (byte_lo)|(byte_hi<<8);
	printf("byte_hi = %X  byle_lo = %X  vysledek = %X nebo %d \n", byte_hi, byte_lo,vysledek,vysledek);
	return vysledek;
}





void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    /* Setting a password implies station will connect to all security modes including WEP/WPA.
        * However these modes are deprecated and not advisable to be used. Incase your Access point
        * doesn't support WPA2, these mode can be enabled by commenting below line */

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

void tisk_teplota(void *pvParameters){
	char *TAG = "tisk_teplota";
	for(;;){
	printf("tisk teplota 1\n");
//    ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void tisk_vlhkost(void *pvParameters){
	char *TAG = "tisk_vlhkost";
	for(;;){
	printf("tisk vlhkost 2\n");
    ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void tisk_hodnot(void *pvParameters){
	char *ktisku;
	ktisku = (char*)pvParameters;
	char *TAG = "tisk_hodnoty";
	for(;;){
	printf("tisk hodnoty 3 %s\n",ktisku);
//    ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}





void tisk_restart(void *pvParameters){
	char *TAG = "RESTART";
	ESP_LOGI(TAG, "odeslani restartu");
}

void blik_led(void *pvParameters) {
	uint16_t cas = 3;
	static const char *TAG = "LED BLIK";
	for (;;) {
//		ESP_LOGI(TAG, "led blik");
		gpio_set_level(led_mb_num, 0);
		vTaskDelay(cas / portTICK_PERIOD_MS);
//		os_delay_us(1000);
		gpio_set_level(led_mb_num, 1);
		esp_task_wdt_reset();
		vTaskDelay(6 / portTICK_PERIOD_MS);
	}
}

uint8_t sbuff[4] = { 0x00, 0x00, 0x00, 0x00 };

//-----------------------------------------------------------------------------
void serial_begin(void)
    {
    SERIAL_DELAY;
    DIO_L;
    SERIAL_DELAY;
    }

//-----------------------------------------------------------------------------
void serial_end(void)
    {
    CLK_L;
    SERIAL_DELAY;
    DIO_L;
    SERIAL_DELAY;

    CLK_H;
    SERIAL_DELAY;
    DIO_H;
    SERIAL_DELAY;
    }

//-----------------------------------------------------------------------------
void serial_cycle(uint8_t data)
    {
    CLK_L;
    SERIAL_DELAY;

    if(data) { DIO_H; } else { DIO_L; }
    SERIAL_DELAY;

    CLK_H;
    SERIAL_DELAY;
    SERIAL_DELAY;
    }

//-----------------------------------------------------------------------------
uint8_t serial_write(uint8_t data)
    {
    uint8_t ack=HIGH;

    for(uint8_t mask=1; mask; mask<<=1)  serial_cycle(data & mask);

    serial_cycle(HIGH);

    if(DIO_IS_CLR) ack=LOW;

    return ack;
    }


//-----------------------------------------------------------------------------
void led_update(void)
    {
    serial_begin();
    serial_write(DATA_COMMAND | AUTO_ADDRESS | WRITE_DATA);
    serial_end();

    serial_begin();
    serial_write(ADDRESS_COMMAND | DEFAULT_ADDRESS);

    for(uint8_t k=0; k<4; k++) serial_write(sbuff[k]);

    serial_end();

//    serial_begin();
//    serial_write(DISPLAY_COMMAND | DISPLAY_ON | disp_bright2);
//    serial_end();
    }

void led_night_set (){
    serial_begin();
    serial_write(DISPLAY_COMMAND | DISPLAY_ON | disp_bright0);
    serial_end();
}

void led_day_set (){
    serial_begin();
    serial_write(DISPLAY_COMMAND | DISPLAY_ON | disp_bright7);
    serial_end();
}

//-----------------------------------------------------------------------------
void led_char(uint8_t pos, uint8_t code)
    {
    uint8_t tmp = chset1[10];  //for unsuppotted chars

    if(code>=48 && code<=57) tmp=chset1[code-48];

    for(uint8_t k=0; k<21; k++)
            {
            if(code==chcode[k]) tmp=chset2[k];
            }

    if(pos<4) sbuff[pos]=(tmp|(sbuff[pos]&0b10000000));
    }

//-----------------------------------------------------------------------------
void led_print(uint8_t pos, char *str)
    {
    for(;((*str) && (pos<4));) led_char(pos++,*str++);

    led_update();
    }

//-----------------------------------------------------------------------------
void led_dots(uint8_t on)
    {
//    if(on) SET_BIT(sbuff[1],7);
//    else CLR_BIT(sbuff[1],7);
	if(on) sbuff[1]|= (1<<7);
	else sbuff[1]&= ~(1<<7);

    led_update();
    }


void cas_to_led(void *pvParameters){
	static const char *TAG = "cas to led";
	time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year < (2016 - 1900)) {
                obtain_time();
    }
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    if(timeinfo.tm_hour>6 && (timeinfo.tm_hour< 21)) led_day_set();
    else led_night_set();
    strftime(strftime_buf, sizeof(strftime_buf), "%H%M", &timeinfo);
    printf(strftime_buf);
    ESP_LOGI(TAG,strftime_buf);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    for(;;);

//	led_print(0, strftime_buf);
//	if(timeinfo.tm_sec &1) {
//		led_dots(1);
//	}
//	else {
//		led_dots(0);
//	}
}

void num_to_led(uint16_t num ){
	const char *TAG = "numtoled";
	char buf[10];
	sprintf(buf,"%d\n",num);
//	printf(buf);
	led_print(0, buf);
//	ESP_LOGI(TAG,"konec\n");
}
void app_main()
{
//	uint16_t adc_data = 0;
//	adc_config_t adc_conf;
//	adc_conf.mode = ADC_READ_TOUT_MODE;
//	adc_conf.clk_div = 8;
//	if((adc_init(&adc_conf))!= ESP_OK) ESP_LOGI(TAG,"chyba");
//	gpio_config_t conf;
//	conf.pin_bit_mask = (1<<TM_1637_CLK)|(1<<TM_1637_DIO);
//	conf.pull_up_en = 1;
//	conf.mode = GPIO_MODE_INPUT;
//	gpio_config(&conf);
//	TM1637_SERIAL_INIT;
//	/* priprava wifi */
//	ESP_ERROR_CHECK(nvs_flash_init());
//	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
//	wifi_init_sta();
	vTaskDelay(5000/portTICK_PERIOD_MS);
//


	//	conf.pin_bit_mask = (1<<led_mb_num);
//	conf.mode = GPIO_MODE_DEF_OUTPUT;
//	gpio_config(&conf);
//	gpio_set_direction(led_mb_num, GPIO_MODE_OUTPUT);


//    vTaskDelay(2000/portTICK_PERIOD_MS);
//    cas_to_led(0);
//    led_day_set();
//    vTaskDelay(2000/portTICK_PERIOD_MS);
//    xTaskCreate(cas_to_led, "Tisk_Casu", 4096, NULL, 0, NULL);
	xTaskCreate(ir_rx_task, "ir_rx_task", 2048, NULL, 5, NULL);
//	for(;;){

//    while(1){
//    	adc_read(&adc_data);
//		led_print(0, "1111");
//    	cas_to_led(0);
//		vTaskDelay(1000/portTICK_PERIOD_MS);
//		ESP_LOGI(TAG,"hodnota GPIO 0 = %d\n", adc_data);
//		gpio_set_level(led_mb_num, 1);

//    	num_to_led(adc_data);
//	}
//	xTaskCreate(tisk_teplota, "tiskteplota", 2000, NULL, 1, NULL);
//	xTaskCreate(tisk_vlhkost, "tiskvlhkost", 2000, NULL, 1, NULL);
//	xTaskCreate(tisk_hodnot, "tiskhodnot", 2000,(void*) HODNOTA11, 1, NULL);
//	xTaskCreate(blik_led, "blikled", 2000, NULL, 1, NULL);
//	}
}




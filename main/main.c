#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "esp_timer.h"

/* Interruption Flag */
#define ESP_INTR_FLAG_DEFAULT 0
int64_t DEBOUNCE_DELAY = 50000;

/* Gpio definitions*/
#define BLINK_GPIO 1
#define GPIO_INPUT_PIN_SEL 39
adc1_channel_t ADC_channel = ADC1_CHANNEL_3 ;

/* Led and Adc task frequency */
const TickType_t xLed_Frequency = 1000 / portTICK_PERIOD_MS;
const TickType_t xAdc_Frequency = 1000 / portTICK_PERIOD_MS;

/* State Indicator Mailbox */
QueueHandle_t xState_Mailbox;

/* Logs tag declaration */
static const char *Init_Tag = "Initialization";
static const char *State1_Tag = "State1";
static const char *State2_Tag = "State2";
static const char *Error_Tag = "Error";

/* Initialization Functions */
static void initialize_led(void)
{
	esp_err_t error_indicator;
	/* set gpio in default state and which mode */
	gpio_reset_pin(BLINK_GPIO);

	error_indicator = gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

	if (error_indicator == ESP_OK ) {
		ESP_LOGI(Init_Tag, "Led initialization was successful");
	} else
	{
		ESP_LOGE(Error_Tag, "Error initializing the led GPIO, the error code is: %d", error_indicator);
	}
}

static void initialize_adc(void)
{
	esp_err_t error_indicator;
	/* set adc width and attenuation */
	error_indicator = adc1_config_width(ADC_WIDTH_BIT_12);
	error_indicator = error_indicator | adc1_config_channel_atten(ADC_channel, ADC_ATTEN_DB_2_5 );

	if (error_indicator == ESP_OK ) {
		ESP_LOGI(Init_Tag, "ADC initialization was successful");
	} else
	{
		ESP_LOGE(Error_Tag, "Error initializing the ADC GPIO");
	}
}

static void initialize_mailbox(void)
{
	/* Create mailbox and fill it with the initial state */
	int zero=0 ;
	xState_Mailbox = xQueueCreate( 1, sizeof( int ) );
	xQueueOverwrite(xState_Mailbox, &zero);
	 if (xState_Mailbox == NULL ) {
		 ESP_LOGE(Error_Tag, "Error creating the mailbox");
	 	} else
	 	{
	 		ESP_LOGI(Init_Tag, "Mailbox initialization was successful");
	 	}
}

/* Interruption handler */
static void IRAM_ATTR isr_handler(void* arg)
{
	int counter;
	static int64_t  last_interruption_time = 0;
	int64_t  interruption_time;
	interruption_time = esp_timer_get_time() ;
	if ((interruption_time - last_interruption_time) > DEBOUNCE_DELAY){
	xQueuePeekFromISR(xState_Mailbox, &counter); /* Reads state  */
	counter += 1 ;
	if( counter >= 3 )
	{
		counter = 0 ;
	}
	xQueueOverwriteFromISR(xState_Mailbox, &counter, NULL);
	last_interruption_time = interruption_time;
	}
}

static void initialize_interrupt_gpio(void)
{
	esp_err_t error_indicator;
	gpio_config_t io_conf = {};
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    error_indicator = gpio_set_intr_type(GPIO_INPUT_PIN_SEL, GPIO_INTR_NEGEDGE);

    //install gpio isr service
    error_indicator = error_indicator | gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    error_indicator = error_indicator | gpio_isr_handler_add(GPIO_INPUT_PIN_SEL, isr_handler, NULL);

	if (error_indicator == ESP_OK ) {
		ESP_LOGI(Init_Tag, "Gpio Interrupt initialization was successful");
	} else
	{
		ESP_LOGE(Error_Tag, "Error initializing the Gpio for interrupt");
	}
}

/* Tasks */
void vTask1( void * pvParameters )
{
	int state; /* system state */
	static uint8_t led_mode = 0; /* led state */
	while(1)
	{
	xQueuePeek(xState_Mailbox, &state, 0); /* Reads state and decides if executing */
	if ((state == 0) | (state ==1) ) {
	ESP_LOGI(State1_Tag, "Turning the LED %s!", led_mode == true ? "ON" : "OFF");
	gpio_set_level(BLINK_GPIO, led_mode);
	led_mode = !led_mode;
	}
	vTaskDelay(xLed_Frequency); /* Task Frequency*/
	}
}

void vTask2( void * pvParameters )
{
	int state; /* system state */
	int adc_value; /* ADC raw value */
	float voltage; /* Voltage read by the ADC */
	while(1)
	{
		xQueuePeek(xState_Mailbox, &state, 0); /* Reads state and decides if executing */
		if ((state == 0) | (state ==2) ) {
		adc_value = adc1_get_raw(ADC_channel);
		voltage = adc_value * 1250 / 4095 ;  /* Interprets the adc value */
		ESP_LOGI(State2_Tag, "Reading the ADC value:  %d   The equivalent voltage in milivolts is: %4.2f", adc_value, voltage);
		}
	vTaskDelay(xAdc_Frequency); /* Task Frequency*/
	}
}

/* Main */
void app_main(void)
{
	/* Initialization */
	initialize_interrupt_gpio();
	initialize_mailbox();
	initialize_led();
	initialize_adc();
	/* Tasks Creation */
	xTaskCreatePinnedToCore( &vTask1, "Led_Blinking", 5000, NULL, 1, NULL , 0);
	xTaskCreatePinnedToCore( &vTask2, "Adc_Reading", 5000, NULL, 1, NULL , 0);
}

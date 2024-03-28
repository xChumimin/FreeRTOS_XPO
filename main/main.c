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

/* Gpio definitions*/
#define BLINK_GPIO 1
#define BUTTON_GPIO 39
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
static const char *State3_Tag = "State3";
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

static void initialize_button(void)
{
	esp_err_t error_indicator;
	/* set gpio in default state and which mode */
	 gpio_reset_pin(BUTTON_GPIO);
	 error_indicator =  gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
	 if (error_indicator == ESP_OK ) {
	 		ESP_LOGI(Init_Tag, "Button initialization was successful");
	 	} else
	 	{
	 		ESP_LOGE(Error_Tag, "Error initializing the Button GPIO, the error code is: %d", error_indicator);
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

void vTask3( void * pvParameters )
{
	int last_button_state = 0 ;
	int button_state = 0 ;
	int counter = 0 ; /* Counter of button presses, it is the state of the system */

	while(1)
		{
		button_state = gpio_get_level(BUTTON_GPIO); /* If a button press exist the counter rises */
		if((button_state == 1) && (last_button_state == 0))
		{
			counter += 1 ;
			if( counter >= 3 )
			{
				counter = 0 ;
			}
			xQueueOverwrite(xState_Mailbox, &counter);
			ESP_LOGI(State3_Tag, "Button Pressed ! Counter:  %d",counter );
		}
		last_button_state = button_state ;
		vTaskDelay(10 / portTICK_PERIOD_MS); /* Task Frequency*/
		}
}

/* Main */
void app_main(void)
{
	/* Initialization */
	initialize_mailbox();
	initialize_led();
	initialize_adc();
	initialize_button();
	/* Tasks Creation */
	xTaskCreatePinnedToCore( &vTask1, "Led_Blinking", 5000, NULL, 1, NULL , 0);
	xTaskCreatePinnedToCore( &vTask2, "Adc_Reading", 5000, NULL, 1, NULL , 0);
	xTaskCreatePinnedToCore( &vTask3, "Button", 5000, NULL, 1, NULL , 0);
}

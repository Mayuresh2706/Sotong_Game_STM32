#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "math.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"

#include "../../Drivers/OLED/ssd1306.h"
#include "../../Drivers/OLED/ssd1306_fonts.h"


#include "oled_assets.h" //contains different bitmaps and bitmap arrays for animation
#include "oled_animations.h" //contains the functionalities for display management

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"
#include "wifi.h"

#define inter_press_delay 1000 //if the delay between two button presses is less than this, it is registered as a double press
#define LOG_BUFFER_SIZE 4096 //max size of the log buffer to store all the messages to be transmitted via uart

volatile int flag = 0; //used for checking single/double press
int single_press_detected = 0;
int double_press_detected = 0;
volatile uint32_t T1;
UART_HandleTypeDef huart1; //UART handler
I2C_HandleTypeDef hi2c1; //I2C handler
int which_game = 0; //0-> Green light, Red light || 1-> Catch and Run
int game_0_in_progress = 0; //1 if Green light, Red light in progress, otherwise 0
int game_1_in_progress = 0; //1 if Catch and Run in progress, otherwise 0
int game_started = 0;
uint32_t code_start_time;

int game_done = 0;// used to detect when player has entered well-lit area in Green Light Red Light
int game_done_printed = 0;//checking whether game done condition printed
int use_WiFi = 0;// to check if WiFi is enabled

//WIFI SETUP
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000
const char* WiFi_SSID = "Mayuresh's A16";
const char* WiFi_password = "12345678";
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;
const uint16_t SOURCE_PORT = 1234;
uint8_t ipaddr[4] = {10, 14, 122, 238};//My Laptop's IP
const uint16_t DEST_PORT = 2028;

char log_buffer[LOG_BUFFER_SIZE] = {0}; //buffer to log all the messages
int log_index = 0;

//some variable initializations for Green Light, Red Light
int green_light_in_progress;
int red_light_in_progress;
uint32_t present_time;
int which_light;
uint32_t print_timer;
uint32_t switch_light_timer;
uint32_t LED_toggle_timer;
int green_light_first_time;
int accelero_flag = 0;// EXTI for accelerometer

//some variable initializations for Catch and Run
uint32_t env_read_timer;
uint32_t magneto_thresh_exceeded_time;
int LED_toggle_delay;
int magneto_thresh_exceeded;
int catch_and_run_animation_layout_playing = 0;
volatile int magneto_data_ready = 1; //if the magneto data is ready (updated using interrupt)
float magneto[3] = {0};

#define MAX_BUZZER_DURATION 5000 //to prevent the buzzer from being ON for more than 5 sec
#define MAX_PATTERN_LEN 16 //max length of a pending pattern
#define BLIP_ON_TIME 50 //time (in ms) of each buzzer blip during countdown
#define DEFAULT_TONE_FREQ 1000 //Default frequency for the buzzer is 1kHz

TIM_HandleTypeDef htim3;
#define BUZZER_TIMER htim3
#define BUZZER_CHANNEL TIM_CHANNEL_4
#define SYSCLK_FREQ 120000000UL

typedef struct {
	int playing; //if a pattern is playing or not
	int current_index; //current index of the pattern
	int length; //length of the pattern
	uint32_t start_time; //time at which the current index of the pattern was started
	unsigned int duration; //duration of the current index of the pattern
	const unsigned int* pattern; //the pattern array
	const unsigned int* frequencies; //frequency for the current index of the pattern
									// 0 for OFF
	int state; //1 = buzzer ON; 0 = buzzer_OFF
	int switch_game_behaviour; //specifies if a pattern can be stopped when games are switched
							   //0-> can be stopped by switch_games() | 1-> can not be stopped by switch_games()

	// allow 1 pending pattern
	unsigned int pending_pattern[MAX_PATTERN_LEN];
	unsigned int pending_frequencies[MAX_PATTERN_LEN];
	int pending_len; //length of the pending pattern
	int has_pending; //if there is a pending pattern
	int pending_switch_game_behaviour; //specifies if the pending pattern can be stopped when games are switched
									   //0-> can be stopped by switch_games() | 1-> can not be stopped by switch_games()

} BuzzerState;

BuzzerState buzzer = {0}; //to store the state of the buzzer

//some patterns for the buzzer
const unsigned int BUZZ_GAME_START[] = {120, 50, 120, 50, 120, 50, 150, 50, 200, 50, 300, 600};
const unsigned int FREQ_GAME_START[] = {523, 0, 659, 0, 784, 0, 1047, 0, 1319, 0, 1568, 0};  // C-E-G-C-E-G major arpeggio
#define LEN_BUZZ_GAME_START (sizeof(BUZZ_GAME_START)/sizeof(unsigned int))

const unsigned int BUZZ_POWERUP[] = {80, 50, 80, 50, 150, 600};
const unsigned int FREQ_POWERUP[] = {800, 0, 1000, 0, 1200, 0};
#define LEN_BUZZ_POWERUP (sizeof(BUZZ_POWERUP)/sizeof(unsigned int))

const unsigned int BUZZ_SUCCESS[] = {100, 50, 100, 50, 100, 150, 200, 800};
const unsigned int FREQ_SUCCESS[] = {1000, 0, 1200, 0, 1500, 0, 2000, 0};
#define LEN_BUZZ_SUCCESS (sizeof(BUZZ_SUCCESS)/sizeof(unsigned int))

const unsigned int BUZZ_GAMEOVER[] = {300, 100, 300, 100, 1000, 800};
const unsigned int FREQ_GAMEOVER[] = {800, 0, 600, 0, 400, 0};
#define LEN_BUZZ_GAMEOVER (sizeof(BUZZ_GAMEOVER)/sizeof(unsigned int))

const unsigned int BUZZ_ERROR[] = {200, 100, 200, 100, 300, 800};
const unsigned int FREQ_ERROR[] = {500, 0, 500, 0, 300, 0};
#define LEN_BUZZ_ERROR (sizeof(BUZZ_ERROR)/sizeof(unsigned int))

const unsigned int BUZZ_DANGER_FAST[] = {100, 100, 100, 100, 100, 100};
const unsigned int FREQ_DANGER_FAST[] = {1500, 0, 1500, 0, 1500, 0};
#define LEN_BUZZ_DANGER_FAST (sizeof(BUZZ_DANGER_FAST)/sizeof(unsigned int))

const unsigned int BUZZ_DANGER_ESCALATE[] = {300, 400, 200, 300, 100, 200};
const unsigned int FREQ_DANGER_ESCALATE[] = {800, 0, 1000, 0, 1200, 0};
#define LEN_BUZZ_DANGER_ESCALATE (sizeof(BUZZ_DANGER_ESCALATE)/sizeof(unsigned int))

const unsigned int BUZZ_RED_LIGHT[] = {300, 400, 300, 400};
const unsigned int FREQ_RED_LIGHT[] = {700, 0, 700, 0};
#define LEN_BUZZ_RED_LIGHT (sizeof(BUZZ_RED_LIGHT)/sizeof(unsigned int))

const unsigned int BUZZ_GREEN_LIGHT[] = {180, 80, 250, 100, 350};
const unsigned int FREQ_GREEN_LIGHT[] = {1200, 0, 1400, 0, 0};
#define LEN_BUZZ_GREEN_LIGHT (sizeof(BUZZ_GREEN_LIGHT)/sizeof(unsigned int))

const unsigned int BUZZ_CATCH_AND_RUN[] = {100, 80, 100, 80, 200, 500};
const unsigned int FREQ_CATCH_AND_RUN[] = {1000, 0, 1000, 0, 1200, 0};
#define LEN_BUZZ_CATCH_AND_RUN (sizeof(BUZZ_CATCH_AND_RUN)/sizeof(unsigned int))

// Player wins - happy victory melody
const unsigned int BUZZ_PLAYER_WIN[] = {150, 50, 150, 50, 150, 50, 200, 100, 150, 50, 150, 50, 200, 100, 250, 50, 250, 50, 400, 800};
const unsigned int FREQ_PLAYER_WIN[] = {523, 0, 659, 0, 784, 0, 1047, 0, 784, 0, 1047, 0, 1319, 0, 1047, 0, 1319, 0, 1568, 0};  // C-E-G-C-G-C-E-C-E-G celebration
#define LEN_BUZZ_PLAYER_WIN (sizeof(BUZZ_PLAYER_WIN)/sizeof(unsigned int))


void SystemClock_Config(void); //auto-generated by CubeMX
void PeriphCommonClock_Config(void); //auto-generated by CubeMX
static void MX_I2C1_Init(void); //I2C1 Initializer
static void MAGNETO_DRDY_EXTI_Init(void); //setup data ready EXTI Interrupt for Magneto

static void UART1_Init(void); //uart initializer function
static void Buzzer_Init(void); //buzzer initializer function
void game(void); //function to handle all the game functionality
void switch_games(void); //handles switching between games
void green_light_red_light(void);
void catch_and_run(void);

static void Accelero_EXTI_Init(void);// setup EXTI Interrupt for Accelerometer
static void MX_ADC1_Init(void);// set up ADC Watchdog for Light Sensor Interrupt
static void WiFi_init(void);// set up ports for WiFi communication
void check_enable_wifi(void);//waits 3 sec for a button press to come to enable WiFi

// --------- UTILITY FUNCTIONS -----------
void log_printf(const char *fmt, ...); //logs formatted text into buffer
void send_message(void); //sends the logged messages via UART and resets the buffer
void send_message_WiFi(void);//sends the logged messages via WiFi
void resetBuffer(void);//resets buffer to 0
void buzzerOn(void); //turns on the buzzer
void buzzerOff(void); //turns off the buzzer
void buzzerStartTone(uint32_t freqHz); //start the buzzer with a provided tone
void buzzerStopTone(void); //stop the playing tone
void buzzerToggle(void); //toggles the buzzer
void buzzerPlayPattern(const unsigned int* pattern, const unsigned int* frequencies, int length, int pend, int switch_game_behaviour); //execute a particular pattern on the buzzer
void buzzerPlayDuration(unsigned int duration); //turns on the delay for a specified time [NON-BLOCKING]
void start_buzzer_countdown(int seconds); //buzzer starts a countdown for a specified number of seconds
void buzzer_update(void); //updates the buzzer state
void readAcc(float accelerometer[3]); //reads accelerometer and performs unit conversion
void readMagneto(float magneto[3]); //reads the magnetometer
void read_temp_pres_hum(float env_readings[3]); //reads temperature, humidity, and pressure and puts them in an array
void printAcc(float accelerometer[3]); //print the accelerometer readings
void printGyro(float gyro[3]); //print the gyroscope readings
void read_and_print_Acc(void); //reads and prints the accelerometer reading
void read_and_print_Gyro(void); //reads and prints the gyroscope reading
void temp_pres_hum_print_game0(void); //read and print temperature, pressure, and humidity for Green Light, Red Light
void temp_pres_hum_print_game1(float env_readings[3]); //print temperature, pressure, and humidity based on thresholds for Catch and Run
float vec_to_mag(float vector[3]); //calculates the magnitude of a vector
int LED_Delay(float magneto[3]); //function to calculate the LED toggle delay based on Magnetometer reading (in Catch and Run)

// SPI Init for WiFi
SPI_HandleTypeDef hspi3;

void SPI3_IRQHandler(void){
  	HAL_SPI_IRQHandler(&hspi3);
}

// ADC Interrupt Handler Enabling
ADC_HandleTypeDef hadc1;
void ADC1_2_IRQHandler(void){
    HAL_ADC_IRQHandler(&hadc1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_EXTI13_Pin){
		int Tpb = HAL_GetTick();
		if (flag == 0){
			T1 = Tpb;
			flag = 1;
		}
		else if (flag == 1){
			flag = 2;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_8){
    	//for magnetometer data ready interrupt
    	magneto_data_ready = 1;
    }

	else if(GPIO_Pin == GPIO_PIN_1){
		SPI_WIFI_ISR();
	}

	else if(GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin){
		if ((which_game == 0) && (red_light_in_progress == 1)){
			accelero_flag = 1;
		}
	}
}


//Callback for crossing light threshold
volatile uint8_t triggered = 0; // Ensures interrupt only called once
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {
	if (triggered){
		return ;
	}
	else if ((which_game == 0) && (green_light_in_progress == 1)){
		game_done = 1;
		triggered = 1;
	}
}



int main(void)
{
	HAL_Init();
	SystemClock_Config();
	PeriphCommonClock_Config();

	code_start_time = HAL_GetTick();
	MX_I2C1_Init();
	UART1_Init();
	BSP_LED_Init(LED2);
	Buzzer_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_TSENSOR_Init();
	BSP_MAGNETO_Init();
	MAGNETO_DRDY_EXTI_Init();
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
	Accelero_EXTI_Init();
	MX_ADC1_Init();

	check_enable_wifi(); //waits 3 sec. If button pressed within 3 sec, WiFi enabled
	if (use_WiFi){
		WiFi_init();
	}
	//WiFi_init();
	code_start_time = HAL_GetTick();
	ssd1306_Init();
	oled_anim_init();

	buzzerPlayPattern(BUZZ_GAME_START, FREQ_GAME_START, LEN_BUZZ_GAME_START, 3, 0);
	oled_start_layout(&sotong_game_layout, 1);

	while (1)
	{
		if (flag==1){
			if (HAL_GetTick() - T1 >= inter_press_delay){
				single_press_detected = 1;
				flag = 0;
			}
		}
		else if (flag==2){
			//DOUBLE PRESS CONFIRMED
			double_press_detected = 1;
			oled_stop_current_layout(); //terminate the current layout on the oled screen
			switch_games();
			flag = 0;
		}
		if (((HAL_GetTick() - code_start_time) >= 3000) && (!game_done)){
			//start the game 3 sec after the code starts
			//this is only for the first time. So there's no "blocking" of the normal flow
			//basically for the "SOTONG GAME" oled animation to be visible
			game();
		}
		buzzer_update();
		oled_anim_update();

		if ((game_done) && (!game_done_printed)){
			//to print Player Wins once when game is done (light interrupt)
			log_printf("\r\n---Player Wins!---\r\n\r\n");
			buzzerPlayPattern(BUZZ_PLAYER_WIN, FREQ_PLAYER_WIN, LEN_BUZZ_PLAYER_WIN, 3, 0);
			oled_start_layout(&player_wins_layout, 1);
			game_done_printed = 1;
		}
		if (use_WiFi){
			//send message buffer over wifi (if using wifi)d
			send_message_WiFi();
		}
		send_message(); //send all the messages via UART. Only one UART transmit for each game iteration. Clean and efficient!
		resetBuffer(); //reset the message
		single_press_detected = 0;
		double_press_detected = 0;
	}
}

void game(void){
	//this function handles all the game functionality
	if (which_game==0){
		green_light_red_light();
	}
	else{
		catch_and_run();
	}
}

void switch_games(void){
	which_game = ~which_game; //change the game being played

	BSP_LED_Off(LED2); //turn off the LED
	//deactivate the running game
	if (game_0_in_progress==1){
		game_0_in_progress = 0;
	}
	if (game_1_in_progress==1){
		game_1_in_progress = 0;
	}

	//handle buzzer switch_game_behaviour
	//this is because some buzzer patterns are supposed to terminate at switch_game and some are not
	if (buzzer.playing == 1 && buzzer.switch_game_behaviour == 0){
		//stop the pattern
		buzzerOff();
		buzzer.playing = 0;
		buzzer.state = 0;
	}
	if (buzzer.has_pending == 1 && buzzer.pending_switch_game_behaviour == 0){
		//remove the pending pattern from the queue
		buzzer.has_pending = 0;
	}
}

void green_light_red_light(void){
	if (game_0_in_progress==0){
		//initialization code for Green light, Red light
		game_0_in_progress = 1;
		log_printf("\r\n-----Entering Green Light, Red Light as Player----\r\n\r\n");
		if ((!game_started) || double_press_detected){
			//to run the red_light_green_light only once at the start (in terminating mode)
			//when coming from catch and run, red light green light is handled by catch and run game over layout
			oled_start_layout(&red_light_green_light_layout, 1);
			game_started = 1;
		}
		green_light_in_progress = 0;
		red_light_in_progress = 0;
		which_light = 0; //0-> Green light || 1-> Red light
		green_light_first_time = 1;
	}

	if (which_light==0){ //green light
		if (green_light_in_progress==0){
			//initialization code for Green light
			green_light_in_progress = 1;

			BSP_LED_On(LED2); //turn on the LED for Green light
			log_printf("---Green Light!---\r\n");
			buzzerPlayPattern(BUZZ_GREEN_LIGHT, FREQ_GREEN_LIGHT, LEN_BUZZ_GREEN_LIGHT, 1, 0);
			print_timer = HAL_GetTick();
			switch_light_timer = HAL_GetTick();
			//HAL_ADC_Start_IT(&hadc1); //enable light interrupt
			start_buzzer_countdown(10); //start a buzzer countdown for 10 sec

			if (green_light_first_time){
				oled_start_layout(&green_light_layout_initial, 0); //start in pending mode if first green light
			}
			else if (!green_light_first_time){
				oled_start_layout(&green_light_layout, 1);
			}
		}
		HAL_ADC_Start_IT(&hadc1); //enable light interrupt
		present_time = HAL_GetTick();

		if (present_time-print_timer >= 2000){
			temp_pres_hum_print_game0();
			print_timer = HAL_GetTick();
		}

		if (present_time-switch_light_timer >= 10000){
			//code to switch to Red light
			green_light_in_progress = 0;
			which_light = 1; //to start Red light
			green_light_first_time = 0;
			BSP_LED_Off(LED2);
			HAL_ADC_Stop_IT(&hadc1);// stop light interrupt
			switch_light_timer = HAL_GetTick();
		}
	}

	else if (which_light==1){ //red light
		if (red_light_in_progress==0){
			//initialization code for Red light
			red_light_in_progress = 1;
			accelero_flag = 0;
			log_printf("\r\n---Red Light!---\r\n");
			buzzerPlayPattern(BUZZ_RED_LIGHT, FREQ_RED_LIGHT, LEN_BUZZ_RED_LIGHT, 3, 0);
			print_timer = HAL_GetTick();
			switch_light_timer = HAL_GetTick();
			LED_toggle_timer = HAL_GetTick();

			oled_start_layout(&red_light_layout, 1);
		}

		present_time = HAL_GetTick();

		if (present_time-LED_toggle_timer >= 500){
			//toggle the LED
			BSP_LED_Toggle(LED2);
			LED_toggle_timer = HAL_GetTick();
		}

		if (present_time-print_timer >= 2000){
			//print accelerometer and gyroscope readings
			log_printf("\r\n");
			read_and_print_Acc();
			read_and_print_Gyro();
			log_printf("\r\n");
			print_timer = HAL_GetTick();
		}

		if (present_time-switch_light_timer >= 10000){
			//code to switch to green light
			red_light_in_progress = 0;
			which_light = 0; //to start Green light
			switch_light_timer = HAL_GetTick();
		}

		if (accelero_flag){ //acclero flag is set using interrupts
			//threshold exceeded. Switch to Catch and Run
			log_printf("Game Over!\r\n");
			accelero_flag = 0;
			buzzerPlayPattern(BUZZ_ERROR, FREQ_ERROR, LEN_BUZZ_ERROR, 3, 1);
			oled_start_layout(&red_light_green_light_game_over_layout, 1);
			red_light_in_progress = 1;
			switch_games();
		}

	}
}
void catch_and_run(void){
	if (game_1_in_progress == 0){
		//initialization code for Catch and Run
		game_1_in_progress = 1;
		log_printf("\r\n-----Entering Catch and Run as Player-----\r\n\r\n");

		oled_start_layout(&catch_and_run_layout, 0);
		buzzerPlayPattern(BUZZ_CATCH_AND_RUN, FREQ_CATCH_AND_RUN, LEN_BUZZ_CATCH_AND_RUN, 1, 0);
		LED_toggle_delay = 0;
		magneto_thresh_exceeded = 0;
		catch_and_run_animation_layout_playing = 0;
		env_read_timer = HAL_GetTick();
		LED_toggle_timer = HAL_GetTick();
	}
	present_time = HAL_GetTick();

	if ((!catch_and_run_animation_layout_playing) && (!oled_is_layout_active())){
		//if the catch and run animations layout is already not playing and no other layout is being played, start playing catch and run animation layout
		catch_and_run_animation_layout_playing = 1;
		oled_start_layout(&catch_and_run_animation_layout, 0);
	}

	if (magneto_data_ready){
		//Read the magnetometer only when new data is available
		//magneto_data_ready is updated based on DRDY interrupt
		readMagneto(magneto);
		magneto_data_ready = 0;
	}

	if ((vec_to_mag(magneto) >= 7500.0) && (magneto_thresh_exceeded==0)){
		//Magnetometer threshold exceeded
		magneto_thresh_exceeded = 1;
		log_printf("\r\n------Enforcer nearby! Be careful.------\r\n\r\n");
		start_buzzer_countdown(3); //start buzzer countdown for 3 sec
		magneto_thresh_exceeded_time = HAL_GetTick();
		catch_and_run_animation_layout_playing = 0;
		oled_start_layout(&enforcer_nearby_layout, 1);
	}

	if (magneto_thresh_exceeded == 1){
		int time_passed = present_time - magneto_thresh_exceeded_time;
		if ((time_passed < 3000) && (single_press_detected==1)){
			log_printf("\r\n---Player escaped, good job!---\r\n\r\n");
			magneto_thresh_exceeded = 0;
			oled_start_layout(&escaped_layout, 1);
			buzzerPlayPattern(BUZZ_SUCCESS, FREQ_SUCCESS, LEN_BUZZ_SUCCESS, 3, 1);
		}
		else if (time_passed >= 3000){
			log_printf("\r\n---Game Over!---\r\n\r\n");
			magneto_thresh_exceeded = 0;
			oled_start_layout(&catch_and_run_game_over_layout, 1);
			buzzerPlayPattern(BUZZ_GAMEOVER, FREQ_GAMEOVER, LEN_BUZZ_GAMEOVER, 3, 1);
			switch_games();
		}
	}

	if (present_time-LED_toggle_timer >= LED_toggle_delay){
		BSP_LED_Toggle(LED2);
		buzzerPlayDuration(40); //small 40 ms blips
		LED_toggle_delay = LED_Delay(magneto); //delay for LED toggle based on Magnetometer reading
		LED_toggle_timer = HAL_GetTick();
	}

	if (present_time-env_read_timer >= 1000){
		//read temperature, humidity, and pressure every 1 second
		float env_readings[3] = {0};
		read_temp_pres_hum(env_readings);
		//print when thresholds exceeded
		temp_pres_hum_print_game1(env_readings);
		env_read_timer = HAL_GetTick();
	}

}

void check_enable_wifi(void){
	//waits for max 3 sec to check if WiFi is to be enabled
	//if button pressed, wifi is enabled
	while (HAL_GetTick() - code_start_time < 3000){
		if (flag == 1){
			use_WiFi = 1;
			break;
		}
	}
}

static void UART1_Init(void)
{
	//intializes UART
	/* Pin configuration for UART. */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	 __HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
	  while(1);
	}
}

static void Buzzer_Init(void){
	//initializes the buzzer connected at PB1
    __HAL_RCC_GPIOB_CLK_ENABLE(); //for GPIO Port B
    __HAL_RCC_TIM3_CLK_ENABLE(); //for timer 3

    //Initialize PB1 GPIO Pin
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //Initialize timer
    TIM_OC_InitTypeDef sConfigOC = {0};

    BUZZER_TIMER.Instance = TIM3;
    BUZZER_TIMER.Init.Prescaler = 0;
    BUZZER_TIMER.Init.CounterMode = TIM_COUNTERMODE_UP;
    BUZZER_TIMER.Init.Period = 65535;
    BUZZER_TIMER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    BUZZER_TIMER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&BUZZER_TIMER) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&BUZZER_TIMER, &sConfigOC, BUZZER_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
}

static void MAGNETO_DRDY_EXTI_Init(void){
	//DRDY pin of the sensor is connected to PC8
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Accelero_EXTI_Init(){
	SENSOR_IO_Init();
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	 __HAL_RCC_GPIOD_CLK_ENABLE();
	 GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	 HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0E, 0x00);
	 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


	 // Writes 0b10000000 to TAP_CFG register enabling interrupts
	 SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80);

	 // Sets a threshold for the wake-up interrupt to be enabled
	 SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WAKE_UP_THS, 0x08);

	 //Sets wake up duration (optional)

	 SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0b00100000);

}

static void MX_ADC1_Init(void)
{

    ADC_ChannelConfTypeDef sConfig = {0};
    ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC_CLK_ENABLE();

    // GPIO for light sensor (adjust pin as needed)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;

    HAL_ADC_Init(&hadc1);

    // Calibrate ADC
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // Configure channel
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Configure Analog Watchdog
    AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
    AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
    AnalogWDGConfig.Channel = ADC_CHANNEL_14;
    AnalogWDGConfig.ITMode = ENABLE;
    AnalogWDGConfig.HighThreshold = 4000;  // Upper threshold
    AnalogWDGConfig.LowThreshold = 0;

    HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig);

    // Enable ADC interrupt
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0x0D, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

static void WiFi_init(void){
	WIFI_Status_t WiFi_Stat;
	WiFi_Stat = WIFI_Init();
	WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security);
	WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT);
}

static void MX_I2C1_Init(void)
{
	/* Enable clocks */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	/* Configure GPIOC for I2C1: PB8 = SCL, PB9 = SDA */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;        // Alternate function, open-drain
	GPIO_InitStruct.Pull = GPIO_PULLUP;            // I2C requires pull-ups
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;     // AF4 = I2C3
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x307075B1;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	*/
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	*/
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

// --------- UTILITY FUNCTIONS ------------
int LED_Delay(float magneto[3]){
	//function to calculate the LED toggle delay based on Magnetometer reading (in Catch and Run)
	//MAX_DELAY=1000 || MIN_DELAY=50
	float mag = vec_to_mag(magneto);
	int delay = (int)((-0.164)*mag + 1360.8); //line between points (2200, 1000) and (8000, 50)
	if (delay<50.0){
		delay = 50.0;
	}
	else if (delay>1000.0){
		delay = 1000.0;
	}
	return delay;
}

int check_acc_gyro_thresh_conditions(float accelerometer[3], float gyro[3]){
	//check for accelerometer and gyroscope threshold conditions
	//!!!!!this function should be REMOVED after implementing interrupts!!!!!
	//1 if threshold exceeded, 0 otherwise;
	if ((vec_to_mag(accelerometer) > 12.0) & (vec_to_mag(gyro) > 1250.0)){ //REFINE THE THRESHOLDS
		return 1;
	}
	return 0;

}

void readAcc(float accelerometer[3]){
	//reads the accelerometer and performs unit conversion
	int16_t accel_raw[3];
	BSP_ACCELERO_AccGetXYZ(accel_raw);
	accelerometer[0] = accel_raw[0]*(9.8/1000.0f);
	accelerometer[1] = accel_raw[1]*(9.8/1000.0f);
	accelerometer[2] = accel_raw[2]*(9.8/1000.0f);
}

void readMagneto(float magneto[3]){
	//reads the magnetometer
	int16_t magneto_raw[3];
	BSP_MAGNETO_GetXYZ(magneto_raw);
	magneto[0] = (float)magneto_raw[0];
	magneto[1] = (float)magneto_raw[1];
	magneto[2] = (float)magneto_raw[2];
}

void printAcc(float accelerometer[3]){
	//print the accelerometer readings
	log_printf("Accelerometer:: X : %.2f ms-2, Y : %.2f ms-2, Z : %.2f ms-2 \r\n", accelerometer[0], accelerometer[1],accelerometer[2]);

}

void printGyro(float gyro[3]){
	//print the gyroscope readings
	log_printf("Gyroscope:: X : %.2f mdps, Y : %.2f mdps, Z : %.2f \r\n mdps", gyro[0], gyro[1],gyro[2]);
}

void read_and_print_Acc(void){
	//reads and prints the accelerometer reading
	float accelerometer[3];
	readAcc(accelerometer);
	printAcc(accelerometer);
}

void read_and_print_Gyro(void){
	//reads and prints the gyroscope reading
	float gyro[3];
	BSP_GYRO_GetXYZ(gyro);
	printGyro(gyro);
}

void read_temp_pres_hum(float env_readings[3]){
	//reads temperature, humidity, and pressure and puts them in an array
	//array order: {temperature (in C), humidity (in %), pressure (in hPa)}
	env_readings[0] = BSP_TSENSOR_ReadTemp();
	env_readings[1] = BSP_HSENSOR_ReadHumidity();
	env_readings[2] = BSP_PSENSOR_ReadPressure();
}

void temp_pres_hum_print_game0(void){
	//read and print temperature, pressure, and humidity for Green Light, Red Light
	float env_readings[3] = {0};
	read_temp_pres_hum(env_readings);
	log_printf("Temperature: %.2fC | Humidity: %.2f%% | Pressure: %.2fhPa\r\n", env_readings[0], env_readings[1], env_readings[2]);
}

void temp_pres_hum_print_game1(float env_readings[3]){
	//print temperature, pressure, and humidity based on thresholds for Catch and Run

	//!!!!! REFINE THE THRESHOLDS !!!!!!;
	if (env_readings[0] >= 35.5){
		log_printf("\nTemperature spike detected! T:%.2fC. Dangerous environment!\r\n\r\n", env_readings[0]);
		catch_and_run_animation_layout_playing = 0;
		oled_start_layout(&dangerous_environment_layout, 1);
		buzzerPlayPattern(BUZZ_DANGER_FAST, FREQ_DANGER_FAST, LEN_BUZZ_DANGER_FAST, 3, 0);
	}

	if (env_readings[1] >= 75.0){
		log_printf("\nHumidity spike detected! H:%.2f%%. Dangerous environment!\r\n\r\n", env_readings[1]);
		catch_and_run_animation_layout_playing = 0;
		oled_start_layout(&dangerous_environment_layout, 1);
		buzzerPlayPattern(BUZZ_DANGER_FAST, FREQ_DANGER_FAST, LEN_BUZZ_DANGER_FAST, 3, 0);
	}

	if (env_readings[2] >= 1050.0){
		log_printf("\nPressure spike detected! P:%.2fhPa. Dangerous environment!\r\n\r\n", env_readings[2]);
		catch_and_run_animation_layout_playing = 0;
		oled_start_layout(&dangerous_environment_layout, 1);
		buzzerPlayPattern(BUZZ_DANGER_FAST, FREQ_DANGER_FAST, LEN_BUZZ_DANGER_FAST, 3, 0);
	}
}

float vec_to_mag(float vector[3]){
	//calculates the magnitude of a vector
	float mag = sqrtf((pow(vector[0],2.0) + pow(vector[1],2.0) + pow(vector[2],2.0)));
	return mag;
}

void log_printf(const char *fmt, ...){
	//logs formatted text into buffer
	va_list args;
	va_start(args, fmt);
	log_index += vsnprintf((log_buffer+log_index), LOG_BUFFER_SIZE - log_index, fmt, args);
	va_end(args);
}

void send_message(void){
	//sends the logged messages via UART and resets the buffer
	HAL_UART_Transmit(&huart1, (uint8_t*)log_buffer, strlen(log_buffer), 0xFFFF);
}

void send_message_WiFi(void){
	uint16_t Datalen;
	WIFI_SendData(1,(uint8_t*)log_buffer , strlen(log_buffer), &Datalen, WIFI_WRITE_TIMEOUT);
}

void resetBuffer(void){
//resets Buffer to 0
	log_index = 0;
	log_buffer[0] = '\0';
}

//------BUZZER FUNCTIONS------
void buzzerOn(void){
	//turns on the buzzer with default tone
	buzzerStartTone(DEFAULT_TONE_FREQ);
}

void buzzerOff(void){
	//turns off the buzzer
	buzzerStopTone();
}

void buzzerStartTone(uint32_t freqHz){
	//start the buzzer with a provided tone
	if (freqHz == 0){
		//stop the buzzer when freqHz = 0
		buzzerStopTone();
		return;
	}

	//PWM_Freq = SYSCLK_FREQ/((PSC+1)*(ARR+1))
	//tick_freq = SYSCLK_FREQ/(PSC+1)
	//PWM_Freq = tick_freq/(ARR+1)

	uint32_t tick_freq = 1000000UL;   // desired 1 MHz tick frequency

	uint32_t psc = (SYSCLK_FREQ/tick_freq) - 1;
	uint32_t arr = (tick_freq/freqHz)-1;
	uint32_t ccr = arr/2; //CCR for 50% duty cycle

    BUZZER_TIMER.Instance->PSC = psc;
    BUZZER_TIMER.Instance->ARR = arr;
    BUZZER_TIMER.Instance->CCR4 = ccr;

    HAL_TIM_PWM_Start(&BUZZER_TIMER, BUZZER_CHANNEL);
}

void buzzerStopTone(void){
	//stop the buzzer
    BUZZER_TIMER.Instance->CCR4 = 0; //set duty cycle to 0
    HAL_TIM_PWM_Stop(&BUZZER_TIMER, BUZZER_CHANNEL);
}

void buzzerToggle(){
	//toggles the buzzer
	if (buzzer.playing){
		// if a pattern is playing, ignore the toggle request
		return;
	}
	if (buzzer.state){
		buzzerOff();
		buzzer.state = 0;
	}
	else{
		buzzerOn();
		buzzer.state = 1;
	}
}

void buzzerPlayPattern(const unsigned int* pattern, const unsigned int* frequencies,
						int length, int pend, int switch_game_behaviour){
	//plays a pattern on the buzzer
	// [NON-BLOCKING]

	// pattern -> array where alternate entries specify ON ond OFF timing of the buzzer (starting from ON)
	// length -> length of the pattern array
	// allows 1 pattern to be pended, that is, one pattern can be pended if another pattern is already playing
	// if multiple patterns pended while some other pattern is playing, only the last one is actually pended
	// pend -> 1 = pend on busy |
	// 		   2 = do not pend (new pattern discarded if another pattern already playing)
	//		   3 = immediately stop current pattern and start the new one
	// switch_game_behaviour -> 0 = can be stopped by switch_games() | 1 = can not be stopped by switch_games()
	if (buzzer.playing){
		if (pend == 1){
			// pend mode
			memcpy(buzzer.pending_pattern, pattern, length * sizeof(unsigned int));
			memcpy(buzzer.pending_frequencies, frequencies, length * sizeof(unsigned int));
			buzzer.pending_len = length;
			buzzer.has_pending = 1;
			buzzer.pending_switch_game_behaviour = switch_game_behaviour;
			return;
		}
		else if (pend == 2){
			//discard the new request (do nothing)
			return;
		}
		else if (pend == 3){
			//interrupt the already playing pattern and play the new one
			buzzerOff();
			buzzer.playing = 0;
			buzzer.state = 0;
			buzzer.has_pending = 0;
		}
		else{
			//default to discard mode
			return;
		}
	}

	buzzer.playing = 1;
	buzzer.pattern = pattern;
	buzzer.frequencies = frequencies;
	buzzer.length = length;
	buzzer.current_index = 0;
	buzzer.switch_game_behaviour = switch_game_behaviour;
	buzzer.start_time = HAL_GetTick();
	buzzer.duration = pattern[0];

	//start first tone
	uint32_t freq = frequencies[0];

	if (freq>0){
		buzzer.state = 1;
		buzzerStartTone(freq);
	}
	else {
		buzzer.state = 0;
		buzzerOff();
	}
}

void buzzerPlayDuration(unsigned int duration){
	// turns on the buzzer with default tone for a specified amount of time [NON-BLOCKING]
	// playing for a duration is just playing a pattern of length 1
	static unsigned int singlePattern[1];
	static unsigned int singleFreq[1];
	singlePattern[0] = duration;
	singleFreq[0] = DEFAULT_TONE_FREQ;
	buzzerPlayPattern(singlePattern, singleFreq, 1, 2, 0); //in non pending mode.
	//if a pattern is playing, a play duration request is ignored
	//play duration can be stopped by switch_games()
}

void buzzer_update(void){
	//updates the buzzer state
	if (!buzzer.playing){
		if (buzzer.has_pending){
			//start the pending pattern
			buzzerPlayPattern(buzzer.pending_pattern, buzzer.pending_frequencies,
							  buzzer.pending_len, 2, buzzer.pending_switch_game_behaviour);
			buzzer.has_pending = 0; //pending pattern cleared
		}
		// don't do anything if the buzzer not playing (and no pending pattern)
		return;
	}

	uint32_t now = HAL_GetTick();
	if ((now - buzzer.start_time >= buzzer.pattern[buzzer.current_index]) || (now - buzzer.start_time >= MAX_BUZZER_DURATION)){
		// current index of the pattern done executing; move to the next one
		buzzer.current_index++;

		if (buzzer.current_index >= buzzer.length){
			// pattern has been completely executed
			buzzerOff();
			buzzer.playing = 0;
			buzzer.state = 0;
			return;
		}

		buzzer.start_time = now;
		buzzer.duration = buzzer.pattern[buzzer.current_index];

		uint32_t freq = buzzer.frequencies[buzzer.current_index];
		if (freq > 0) {
			buzzer.state = 1;
		    buzzerStartTone(freq);
		}
		else {
		    buzzer.state = 0;
		    buzzerOff();
		}
	}
}

void start_buzzer_countdown(int seconds){
	//buzzer starts a countdown for the specified number of seconds
	//beeps faster as 0 approaches
	if (seconds > 30){
		//max countdown = 30 sec
		seconds = 30;
	}

	static unsigned int pattern[300];
	static unsigned int frequencies[300];
	int idx = 0;

	for (int i=0; i<seconds; i++){
		int remaining = seconds - i;

		int blips_per_sec = 1; //1 blip per second until 5 seconds left
		unsigned int tone_freq = 800;  // Low tone for normal countdown

		if (remaining <= 5 && remaining >3){
			//2 blips per second between 5 and 3
			blips_per_sec = 2;
			tone_freq = 1000;   // Medium tone
		}
		else if (remaining <= 3 && remaining > 2){
			//3 blips per second between 3 and 2
			blips_per_sec = 3;
			tone_freq = 1200;   // Higher tone
		}
		else if (remaining <= 2 && remaining > 1){
			//4 blips per second between 2 and 1
			blips_per_sec = 4;
			tone_freq = 1500;   // Even higher tone
		}
		else if (remaining <= 1){
			//5 blips per second between 1 and 0
			blips_per_sec = 5;
			tone_freq = 2000;   // Highest urgency tone
		}
		unsigned int cycle_time = 1000/blips_per_sec;
		unsigned int off_time = cycle_time - BLIP_ON_TIME;

		for (int b=0; b<blips_per_sec; b++){
			pattern[idx] = BLIP_ON_TIME;
			frequencies[idx] = tone_freq; // Tone during blip
			idx++;
			if (!(i == seconds - 1 && b == blips_per_sec - 1)){
				//add an off-time except after the very last blip
				pattern[idx] = off_time;
				frequencies[idx] = 0;  // Silence during off-time
				idx++;
			}
		}
	}

	buzzerPlayPattern(pattern, frequencies, idx, 1, 0); //start countdown in pend mode and can be interrupted by switch_games()
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

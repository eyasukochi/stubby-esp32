#include <esp_log.h>
#include <string>
#include "sdkconfig.h"

#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>



#include "sdkconfig.h"

#define I2C_EXAMPLE_MASTER_SCL_IO   GPIO_NUM_4    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO   GPIO_NUM_5    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_FREQ_HZ  100000     /*!< I2C master clock frequency */
#define I2C_EXAMPLE_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


#define I2C_ADDRESS     0x40    /*!< lave address for PCA9685 */
#define I2C_ADDRESS_2   0x41

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 120 //Maximum angle in degree upto which servo can rotate

static char tag[] = "STUBBY";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);


extern "C" {
	#include "pca9685.h"
	void app_main(void);
}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD(tag, ">> PCA9685");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;

    i2c_port_t i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));
}

esp_err_t set_any_pwm(uint8_t board, uint8_t num, uint16_t on, uint16_t off)
{
	if (board==0) {
		set_pca9685_adress(I2C_ADDRESS);
	} else {
		set_pca9685_adress(I2C_ADDRESS_2);
	}
	return setPWM(num, on, off);
}

void init_zero()
{

	set_pca9685_adress(I2C_ADDRESS);
	resetPCA9685();
	setFrequencyPCA9685(60);  // 1000 Hz
	turnAllOff();
}

void init_one()
{

	set_pca9685_adress(I2C_ADDRESS_2);
	resetPCA9685();
	setFrequencyPCA9685(60);  // 1000 Hz
	turnAllOff();
}

void task_PCA9685(void *ignore)
{
    printf("Executing on core %d\n", xPortGetCoreID());

    i2c_example_master_init();
    init_zero();
    init_one();

    printf("Finished setup, entering loop now\n");

    while(1)
    {
        printf("Now attempting test\n");

        // Just operate on pin 2 (lower leg focus for now)

        // FOR REFERENCE, THESE NUMBERS CAME FROM:  https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino

        // lowers: FIRST:  2, 10, 14, SECOND: 2, 10, 14

		uint16_t index;//, count;
		for (index = 150; index <= 500 ; index += 1) {

			set_any_pwm(0, 2, 0, index);
			set_any_pwm(0, 10, 0, index);
			set_any_pwm(0, 14, 0, index);
			set_any_pwm(1, 2, 0, index);
			set_any_pwm(1, 10, 0, index);
			set_any_pwm(1, 14, 0, index);

			vTaskDelay(5);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
		}

        vTaskDelay(1000 / portTICK_RATE_MS);

    }

    vTaskDelete(NULL);
}

void app_main(void)
{
	xTaskCreate(task_PCA9685, "task_PCA9685", 1024 * 2, (void* ) 0, 10, NULL);

}


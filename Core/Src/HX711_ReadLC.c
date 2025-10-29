#include "hx711.h"
#include "hx711Config.h"
#if (_HX711_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define hx711_delay(x)    osDelay(x)
#else
#define hx711_delay(x)    HAL_Delay(x)
#endif

//#############################################################################################
void hx711_delay_us(void)
{
  uint32_t delay = _HX711_DELAY_US_LOOP;
  while (delay > 0)
  {
    delay--;
  }
}
//#############################################################################################
void hx711_lock(hx711_t *hx711)
{
  while (hx711->lock)
    hx711_delay(1);
  hx711->lock = 1;
}
//#############################################################################################
void hx711_unlock(hx711_t *hx711)
{
  hx711->lock = 0;
}
//#############################################################################################
void hx711_init(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin)
{
  hx711_lock(hx711);
  hx711->clk_gpio = clk_gpio;
  hx711->clk_pin = clk_pin;
  hx711->dat_gpio = dat_gpio;
  hx711->dat_pin = dat_pin;

  GPIO_InitTypeDef  gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = clk_pin;
  HAL_GPIO_Init(clk_gpio, &gpio);
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = dat_pin;
  HAL_GPIO_Init(dat_gpio, &gpio);
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
  hx711_delay(10);
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
  hx711_delay(10);
  hx711_value(hx711);
  hx711_value(hx711);
  hx711_unlock(hx711);
}
//#############################################################################################
int32_t hx711_value(hx711_t *hx711)
{
  uint32_t data = 0;
  uint32_t  startTime = HAL_GetTick();
  while(HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_SET)
  {
    hx711_delay(1);
    if(HAL_GetTick() - startTime > 150)
      return 0;
  }
  for(int8_t i=0; i<24 ; i++)
  {
    HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
    hx711_delay_us();
    HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
    hx711_delay_us();
    data = data << 1;
    if(HAL_GPIO_ReadPin(hx711->dat_gpio, hx711->dat_pin) == GPIO_PIN_SET)
      data ++;
  }
  data = data ^ 0x800000;
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
  hx711_delay_us();
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
  hx711_delay_us();
  return data;
}
//#############################################################################################
int32_t hx711_value_ave(hx711_t *hx711, uint16_t sample)
{
  hx711_lock(hx711);
  int64_t  ave = 0;
  for(uint16_t i=0 ; i<sample ; i++)
  {
    ave += hx711_value(hx711);
    hx711_delay(5);
  }
  answer = (int32_t)(ave / sample);
  hx711_unlock(hx711);
  return answer;
}
//#############################################################################################
void hx711_tare(hx711_t *hx711, uint16_t sample)
{
  hx711_lock(hx711);
  int64_t  ave = 0;
  for(uint16_t i=0 ; i<sample ; i++)
  {
    ave += hx711_value(hx711);
    hx711_delay(5);
  }
  hx711->offset = (int32_t)(ave / sample);
  hx711_unlock(hx711);
}
//#############################################################################################
void hx711_calibration(hx711_t *hx711, int32_t noload_raw, int32_t load_raw, float scale)
{
  hx711_lock(hx711);
  hx711->offset = noload_raw;
  hx711->coef = (load_raw - noload_raw) / scale;
  hx711_unlock(hx711);
}
//#############################################################################################
float hx711_weight(hx711_t *hx711, uint16_t sample)
{
  hx711_lock(hx711);
  int64_t  ave = 0;
  for(uint16_t i=0 ; i<sample ; i++)
  {
    ave += hx711_value(hx711);
    hx711_delay(5);
  }
  int32_t data = (int32_t)(ave / sample);
  float answer =  (data - hx711->offset) / hx711->coef;
  hx711_unlock(hx711);
  return answer;
}
//#############################################################################################
void hx711_coef_set(hx711_t *hx711, float coef)
{
  hx711->coef = coef;
}
//#############################################################################################
float hx711_coef_get(hx711_t *hx711)
{
  return hx711->coef;
}

/* -----------------------------------------------------------------------------
* function : hx711_collect_weight_samples()
* INs      : hx           Pointer to the HX711 load cell structure
 *				 data_array   Pointer to array of weight_sample_t to store data
 *				 num_points   Number of samples to collect
 *     		 interval_ms  Time delay between each sample in milliseconds
* OUTs     :
* action   : Collect weight samples and timestamps from HX711 into an array
* authors  : Karla Lira (kl) - kliragon@calpoly.edu
* version  : 0.1
* date     : 251016
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 251016 kl
* -------------------------------------------------------------------------- */
void hx711_collect_weight_samples(hx711_t *hx, weight_sample_t *data_array, uint16_t num_points, uint16_t interval_ms)
{
    for (uint16_t i = 0; i < num_points; i++)
    {
        data_array[i].timestamp_ms = HAL_GetTick();	  // Record current timestamp in milliseconds
        data_array[i].weight_g = hx711_weight(hx, 20);  // 20-sample average
        hx711_delay(interval_ms);
    }
}
//#############################################################################################
void hx711_power_down(hx711_t *hx711)
{
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_SET);
  hx711_delay(1);
}
//#############################################################################################
void hx711_power_up(hx711_t *hx711)
{
  HAL_GPIO_WritePin(hx711->clk_gpio, hx711->clk_pin, GPIO_PIN_RESET);
}
//#############################################################################################


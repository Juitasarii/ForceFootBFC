/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * Author: Juita Sari
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* HX711 Load Cell Definitions
   
   HX711 is a digital pressure sensor interface IC
   Each loadcell requires 2 GPIO pins: SCK (clock output) and DT (data input)
   
   Pin Assignment based on available pins (STM32F401CEUx):
   ► Already used: PA9(UART_RX), PA10(UART_TX), PA13(SYS_JTMS), PH0-PH1(OSC), PB12(DIR)
   
   Selected Configuration for 4 Loadcells:
   Loadcell 0: SCK=PA2, DT=PA1
   Loadcell 1: SCK=PA4, DT=PA3
   Loadcell 2: SCK=PA6, DT=PA5
   Loadcell 3: SCK=PB0, DT=PA7
   
   GPIO Output (SCK): PA2, PA4, PA6, PB0
   GPIO Input (DT):  PA1, PA3, PA5, PA7
*/

#define HX711_NUM_SENSORS     4U

typedef struct {
  GPIO_TypeDef* gpio_sck;
  uint16_t pin_sck;
  GPIO_TypeDef* gpio_dt;
  uint16_t pin_dt;
} HX711_PinConfig;

static const HX711_PinConfig hx711_pins[HX711_NUM_SENSORS] = {
  { GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_1 },  /* Loadcell 0: PA2(SCK), PA1(DT) */
  { GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_3 },  /* Loadcell 1: PA4(SCK), PA3(DT) */
  { GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5 },  /* Loadcell 2: PA6(SCK), PA5(DT) */
  { GPIOB, GPIO_PIN_0, GPIOA, GPIO_PIN_7 },  /* Loadcell 3: PB0(SCK), PA7(DT) */
};

/* HX711 offset calibration values (zero reading) */
static int32_t hx711_offset[HX711_NUM_SENSORS] = {0, 0, 0, 0};

#define DXL_UART_HANDLE huart1
#define DXL_XH540_MODEL_NUMBER 1150U
#define DXL_XH540_FIRMWARE_VERSION 45U
#define DXL_PACKET_MAX_SIZE   256U
#define DXL_CONTROL_TABLE_SIZE 256U
#define DXL_INST_PING         0x01U
#define DXL_INST_READ         0x02U
#define DXL_INST_WRITE        0x03U
#define DXL_INST_REBOOT       0x08U
#define DXL_INST_STATUS       0x55U
#define DXL_BROADCAST_ID      0xFEU
#define DXL_LED_ACTIVITY_MS   60U
#define DBG_RX_Pin            GPIO_PIN_13
#define DBG_RX_GPIO_Port      GPIOB
#define DBG_CRC_Pin           GPIO_PIN_14
#define DBG_CRC_GPIO_Port     GPIOB
#define DBG_PKT_Pin           GPIO_PIN_15
#define DBG_PKT_GPIO_Port     GPIOB

typedef struct
{
  UART_HandleTypeDef *huart;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;

  uint8_t id;
  uint8_t rx_byte;
  uint16_t rx_size;
  uint8_t rx_buffer[DXL_PACKET_MAX_SIZE];

  uint8_t tx_pending;
  uint16_t tx_size;
  uint8_t tx_buffer[DXL_PACKET_MAX_SIZE];
  uint8_t control_table[DXL_CONTROL_TABLE_SIZE];
  int32_t present_position;
  int32_t present_velocity;
  uint16_t present_current;
  uint16_t present_pwm;
  uint16_t present_voltage;
  uint8_t present_temperature;
  uint32_t last_update_tick;
} DxlScanResponder;

static DxlScanResponder g_dxl;
static volatile uint32_t g_dxl_last_rx_tick;
static volatile uint8_t g_dxl_rx_seen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void HX711_Init_Pins(void);
static int32_t HX711_ReadValue(uint8_t sensor_id);
static void HX711_Calibrate_Offset(void);
static void DxlScanResponder_SetDirectionRx(DxlScanResponder *ctx);
static void DxlScanResponder_SetDirectionTx(DxlScanResponder *ctx);
static void DxlScanResponder_ResetParser(DxlScanResponder *ctx);
static uint16_t DxlScanResponder_UpdateCrc(uint16_t crc,
                                           const uint8_t *data,
                                           uint16_t length);
static uint16_t DxlScanResponder_RemoveStuffing(const uint8_t *src,
                                                uint16_t src_len,
                                                uint8_t *dst,
                                                uint16_t dst_max);
static uint16_t DxlScanResponder_ApplyStuffing(const uint8_t *src,
                                               uint16_t src_len,
                                               uint8_t *dst,
                                               uint16_t dst_max);
static void DxlScanResponder_InitControlTable(DxlScanResponder *ctx);
static void DxlScanResponder_UpdateTelemetry(DxlScanResponder *ctx);
static void DxlScanResponder_ProcessBuffer(DxlScanResponder *ctx);
static uint8_t DxlScanResponder_ShouldRespond(const DxlScanResponder *ctx,
                                              uint8_t packet_id,
                                              uint8_t instruction);
static uint8_t DxlScanResponder_ReadRegister(const DxlScanResponder *ctx, uint16_t address);
static void DxlScanResponder_WriteRegister(DxlScanResponder *ctx, uint16_t address, uint8_t value);
static uint16_t DxlScanResponder_ReadWord(const DxlScanResponder *ctx, uint16_t address);
static uint32_t DxlScanResponder_ReadDword(const DxlScanResponder *ctx, uint16_t address);
static void DxlScanResponder_WriteWord(DxlScanResponder *ctx, uint16_t address, uint16_t value);
static void DxlScanResponder_WriteDword(DxlScanResponder *ctx, uint16_t address, uint32_t value);
static uint8_t DxlScanResponder_IsWritableAddress(uint16_t address);
static uint8_t DxlScanResponder_WriteBlock(DxlScanResponder *ctx,
                                           uint16_t address,
                                           const uint8_t *data,
                                           uint16_t length);
static void DxlScanResponder_HandlePacket(DxlScanResponder *ctx,
                                          uint8_t packet_id,
                                          uint8_t instruction,
                                          const uint8_t *params,
                                          uint16_t param_count);
static void DxlScanResponder_QueueStatus(DxlScanResponder *ctx,
                                         uint8_t packet_id,
                                         uint8_t error,
                                         const uint8_t *params,
                                         uint16_t param_count);
static void DxlScanResponder_RestartReception(DxlScanResponder *ctx);
static HAL_StatusTypeDef DxlScanResponder_Start(DxlScanResponder *ctx);
static void DxlScanResponder_ProcessByte(DxlScanResponder *ctx, uint8_t byte);
static void ForceFoot_MarkBusActivity(void);
static void ForceFoot_UpdateActivityLed(void);
static void Debug_SetTxPendingIndicator(uint8_t active);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
   74HC125N FORWARD DECLARATIONS
   ============================================================================ */
static void HC125N_Init(void);
static void HC125N_Enable(void);
static void HC125N_Disable(void);
static uint8_t HC125N_IsEnabled(void);

/* ============================================================================
   74HC125N 3-STATE BUFFER CONTROL FUNCTIONS
   ============================================================================
   IC 74HC125N: 3-State Buffer dengan 4 channel
   
   Konfigurasi untuk U2D2 TTL Communication:
   - PA9 (UART1 TX) ────────► HC125N Pin 2 (1A input)
   - HC125N Pin 3 (1Y) ─────► U2D2 DATA line (with 4.7K pull-up to 3.3V)
   - PA10 (UART1 RX) ──────► U2D2 DATA line (direct)
   - PB12 (GPIO Output) ────► HC125N Pin 1 (1OE - Output Enable)
   
   Operasi:
   - OE = 0 (LOW):  HC125N output aktif → TX data ke U2D2
   - OE = 1 (HIGH): HC125N output tri-state → hanya bisa RX dari U2D2
   
   Note: PA9 tetap UART TX, PA10 tetap UART RX (jangan ubah ke GPIO!)
        Hanya PB12 yang dikontrol sebagai GPIO untuk OE signal.
*/

typedef struct {
  GPIO_TypeDef* oe_port;
  uint16_t oe_pin;
  uint8_t enabled;
} HC125N_Config;

static HC125N_Config hc125n = {0};

/* Initialize 74HC125N - hanya setup PB12 untuk Output Enable control */
static void HC125N_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Enable GPIO clock untuk PB12 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure PB12 sebagai GPIO Output untuk HC125N OE control */
  GPIO_InitStruct.Pin = HC125N_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HC125N_OE_GPIO_Port, &GPIO_InitStruct);
  
  /* Initialize structure */
  hc125n.oe_port = HC125N_OE_GPIO_Port;
  hc125n.oe_pin = HC125N_OE_Pin;
  hc125n.enabled = 0U;
  
  /* Default state: Disable buffer output (OE=HIGH)
     Ini memastikan RX line bebas saat startup */
  HC125N_Disable();
}

/* Enable 74HC125N output (OE=LOW)
   Mengaktifkan TX path: PA9 UART TX ─► HC125N ─► U2D2 */
static void HC125N_Enable(void)
{
  HAL_GPIO_WritePin(hc125n.oe_port, hc125n.oe_pin, GPIO_PIN_RESET);
  hc125n.enabled = 1U;
}

/* Disable 74HC125N output (OE=HIGH) - tri-state
   Menonaktifkan TX buffer, hanya RX yang aktif: U2D2 ─► PA10 UART RX */
static void HC125N_Disable(void)
{
  HAL_GPIO_WritePin(hc125n.oe_port, hc125n.oe_pin, GPIO_PIN_SET);
  hc125n.enabled = 0U;
}

/* Check if buffer output is enabled */
static uint8_t HC125N_IsEnabled(void)
{
  return hc125n.enabled;
}

/* ============================================================================
   END 74HC125N BUFFER FUNCTIONS
   ============================================================================
*/

#define DXL_REG_MODEL_NUMBER           0U
#define DXL_REG_MODEL_INFORMATION      2U
#define DXL_REG_FIRMWARE_VERSION       6U
#define DXL_REG_ID                     7U
#define DXL_REG_BAUDRATE               8U
#define DXL_REG_RETURN_DELAY           9U
#define DXL_REG_DRIVE_MODE             10U
#define DXL_REG_OPERATING_MODE         11U
#define DXL_REG_SECONDARY_ID           12U
#define DXL_REG_PROTOCOL_TYPE          13U
#define DXL_REG_HOMING_OFFSET          20U
#define DXL_REG_MOVING_THRESHOLD       24U
#define DXL_REG_TEMPERATURE_LIMIT      31U
#define DXL_REG_MAX_VOLTAGE_LIMIT      32U
#define DXL_REG_MIN_VOLTAGE_LIMIT      34U
#define DXL_REG_PWM_LIMIT              36U
#define DXL_REG_CURRENT_LIMIT          38U
#define DXL_REG_ACCEL_LIMIT            40U
#define DXL_REG_VELOCITY_LIMIT         44U
#define DXL_REG_MAX_POSITION_LIMIT     48U
#define DXL_REG_MIN_POSITION_LIMIT     52U
#define DXL_REG_SHUTDOWN               63U
#define DXL_REG_TORQUE_ENABLE          64U
#define DXL_REG_LED                    65U
#define DXL_REG_STATUS_RETURN_LEVEL    68U
#define DXL_REG_REGISTERED_INSTRUCTION 69U
#define DXL_REG_HARDWARE_ERROR_STATUS  70U
#define DXL_REG_VELOCITY_I_GAIN        76U
#define DXL_REG_VELOCITY_P_GAIN        78U
#define DXL_REG_POSITION_D_GAIN        80U
#define DXL_REG_POSITION_I_GAIN        82U
#define DXL_REG_POSITION_P_GAIN        84U
#define DXL_REG_FEEDFORWARD_2ND_GAIN   88U
#define DXL_REG_FEEDFORWARD_1ST_GAIN   90U
#define DXL_REG_BUS_WATCHDOG           98U
#define DXL_REG_GOAL_PWM               100U
#define DXL_REG_GOAL_CURRENT           102U
#define DXL_REG_GOAL_VELOCITY          104U
#define DXL_REG_PROFILE_ACCELERATION   108U
#define DXL_REG_PROFILE_VELOCITY       112U
#define DXL_REG_GOAL_POSITION          116U
#define DXL_REG_REALTIME_TICK          120U
#define DXL_REG_MOVING                 122U
#define DXL_REG_MOVING_STATUS          123U
#define DXL_REG_PRESENT_PWM            124U
#define DXL_REG_PRESENT_CURRENT        126U
#define DXL_REG_PRESENT_VELOCITY       128U
#define DXL_REG_PRESENT_POSITION       132U
#define DXL_REG_VELOCITY_TRAJECTORY    136U
#define DXL_REG_POSITION_TRAJECTORY    140U
#define DXL_REG_PRESENT_INPUT_VOLTAGE  144U
#define DXL_REG_PRESENT_TEMPERATURE    146U

/* Sensor data area - similar to ForceFoot GitHub
   4 sensors x 3 bytes each = 12 bytes starting at register 160 */
#define DXL_SENSOR_DATA_START          160U
#define DXL_NUM_SENSORS                4U
#define DXL_SENSOR_BYTES_PER_SENSOR    3U

/* Simulated sensor value storage
   In real application, read from actual ADC/sensor hardware */
typedef struct
{
  int32_t sensor_values[DXL_NUM_SENSORS];
} SensorData;

static SensorData g_sensor_data = {0};

/* Helper function to put 3-byte sensor value into buffer
   Similar to putValue() in ForceFoot GitHub */
static void DxlScanResponder_PutSensorValue(uint8_t *dst, int32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
}

/* Update sensor data to control table
   This is called before each READ request */
static void DxlScanResponder_UpdateSensorData(DxlScanResponder *ctx)
{
  uint16_t i;
  uint8_t temp_buf[3];

  /* Put all 4 sensor values into control table starting at DXL_SENSOR_DATA_START */
  for (i = 0U; i < DXL_NUM_SENSORS; i++) {
    DxlScanResponder_PutSensorValue(temp_buf, g_sensor_data.sensor_values[i]);
    ctx->control_table[DXL_SENSOR_DATA_START + (i * DXL_SENSOR_BYTES_PER_SENSOR) + 0U] = temp_buf[0];
    ctx->control_table[DXL_SENSOR_DATA_START + (i * DXL_SENSOR_BYTES_PER_SENSOR) + 1U] = temp_buf[1];
    ctx->control_table[DXL_SENSOR_DATA_START + (i * DXL_SENSOR_BYTES_PER_SENSOR) + 2U] = temp_buf[2];
  }
}

/* Simulated sensor read function
   In real application, read from actual ADC or sensor hardware */
static void DxlScanResponder_ReadSensors(void)
{
  /* READ FROM HX711 LOADCELL SENSORS
     
     This function is called every time Dynamixel host sends READ command.
     It reads all 4 HX711 loadcells and updates the control table.
  */
  
  uint16_t i;
  int32_t raw_value;
  
  /* Read all 4 HX711 loadcells */
  for (i = 0U; i < DXL_NUM_SENSORS; i++) {
    
    /* Read 24-bit value from HX711 */
    raw_value = HX711_ReadValue(i);
    
    /* Subtract offset (calibration) to get actual weight */
    raw_value -= hx711_offset[i];
    
    /* Clamp to 24-bit signed range for Dynamixel protocol */
    if (raw_value > 8388607) {
      raw_value = 8388607;
    } else if (raw_value < -8388608) {
      raw_value = -8388608;
    }
    
    /* Store in sensor data */
    g_sensor_data.sensor_values[i] = raw_value;
  }
}

/* ============================================================================
   HX711 HARDWARE INTERFACE FUNCTIONS
   ============================================================================ */

/* Initialize GPIO pins for all 4 HX711 loadcells
   SCK pins configured as GPIO Output
   DT pins configured as GPIO Input
*/
static void HX711_Init_Pins(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t i;
  
  /* Enable GPIO clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure all SCK pins (outputs) and DT pins (inputs) for each loadcell */
  for (i = 0U; i < HX711_NUM_SENSORS; i++) {
    /* Configure SCK pin as GPIO Output */
    GPIO_InitStruct.Pin = hx711_pins[i].pin_sck;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hx711_pins[i].gpio_sck, &GPIO_InitStruct);
    
    /* Initialize SCK to LOW (inactive state) */
    HAL_GPIO_WritePin(hx711_pins[i].gpio_sck, hx711_pins[i].pin_sck, GPIO_PIN_RESET);
    
    /* Configure DT pin as GPIO Input */
    GPIO_InitStruct.Pin = hx711_pins[i].pin_dt;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hx711_pins[i].gpio_dt, &GPIO_InitStruct);
  }
}

/* Read 24-bit value from HX711 loadcell
   HX711 Protocol:
   - Waits for DT pin to go LOW (data ready)
   - Clocks out 24 bits of data via SCK pulses
   - Final SCK pulse selects gain (128x by staying low)
   - Returns signed 24-bit value
*/
static int32_t HX711_ReadValue(uint8_t sensor_id)
{
  int32_t result = 0;
  uint8_t bit;
  uint32_t timeout_count;
  
  if (sensor_id >= HX711_NUM_SENSORS) {
    return 0;
  }
  
  /* Wait for DT to go LOW (data ready) - timeout at ~1ms */
  timeout_count = 0;
  while (HAL_GPIO_ReadPin(hx711_pins[sensor_id].gpio_dt, hx711_pins[sensor_id].pin_dt) == GPIO_PIN_SET) {
    timeout_count++;
    if (timeout_count > 10000U) {
      return 0;  /* Timeout - sensor not responding */
    }
  }
  
  /* Read 24 bits of data, MSB first */
  for (bit = 0U; bit < 24U; bit++) {
    
    /* Clock HIGH - data setup time */
    HAL_GPIO_WritePin(hx711_pins[sensor_id].gpio_sck, hx711_pins[sensor_id].pin_sck, GPIO_PIN_SET);
    
    /* Small delay for data line to stabilize */
    for (volatile uint32_t delay = 0U; delay < 10U; delay++) {
    }
    
    /* Read bit from DT */
    result = result << 1;
    if (HAL_GPIO_ReadPin(hx711_pins[sensor_id].gpio_dt, hx711_pins[sensor_id].pin_dt) == GPIO_PIN_SET) {
      result |= 1;
    }
    
    /* Clock LOW */
    HAL_GPIO_WritePin(hx711_pins[sensor_id].gpio_sck, hx711_pins[sensor_id].pin_sck, GPIO_PIN_RESET);
    
    /* Delay before next clock pulse */
    for (volatile uint32_t delay = 0U; delay < 10U; delay++) {
    }
  }
  
  /* 25th clock pulse - gain select (keep low for 128x gain) */
  HAL_GPIO_WritePin(hx711_pins[sensor_id].gpio_sck, hx711_pins[sensor_id].pin_sck, GPIO_PIN_SET);
  for (volatile uint32_t delay = 0U; delay < 10U; delay++) {
  }
  HAL_GPIO_WritePin(hx711_pins[sensor_id].gpio_sck, hx711_pins[sensor_id].pin_sck, GPIO_PIN_RESET);
  
  /* Convert to 24-bit signed integer (sign extend from bit 23) */
  if (result & 0x800000U) {
    result |= 0xFF000000U;  /* Sign extend */
  }
  
  return result;
}

/* Calibrate HX711 offset (tare/zero) by averaging multiple readings
   Called at startup or when user requests recalibration
*/
static void HX711_Calibrate_Offset(void)
{
  uint8_t sensor_id;
  uint8_t sample;
  int32_t sum;
  
  /* For each loadcell, average 10 readings to get zero offset */
  for (sensor_id = 0U; sensor_id < HX711_NUM_SENSORS; sensor_id++) {
    sum = 0;
    
    /* Take 10 samples */
    for (sample = 0U; sample < 10U; sample++) {
      sum += HX711_ReadValue(sensor_id);
      
      /* Small delay between samples */
      HAL_Delay(10U);
    }
    
    /* Store average as offset */
    hx711_offset[sensor_id] = sum / 10;
  }
}

/* ============================================================================
   END HX711 FUNCTIONS
   ============================================================================ */


void ForceFoot_DxlScan_Init(void)
{
  uint16_t i;

  memset(&g_dxl, 0, sizeof(g_dxl));
  g_dxl.huart = &DXL_UART_HANDLE;
  g_dxl.dir_port = DXL_DIR_GPIO_Port;
  g_dxl.dir_pin = DXL_DIR_Pin;
  g_dxl.id = 21U;
  g_dxl_last_rx_tick = 0U;
  g_dxl_rx_seen = 0U;

  /* Initialize 74HC125N buffer control pins (PB12, PA9, PA8) */
  HC125N_Init();
  
  /* Initialize HX711 GPIO pins */
  HX711_Init_Pins();
  
  /* Calibrate HX711 offset (tare/zero) */
  HX711_Calibrate_Offset();

  /* Initialize sensor values */
  for (i = 0U; i < DXL_NUM_SENSORS; i++) {
    g_sensor_data.sensor_values[i] = 0;
  }

  DxlScanResponder_InitControlTable(&g_dxl);
  DxlScanResponder_SetDirectionRx(&g_dxl);
  DxlScanResponder_ResetParser(&g_dxl);
  LED_Off();
  Debug_SetTxPendingIndicator(0U);
  DxlScanResponder_Start(&g_dxl);
}

void ForceFoot_DxlScan_Task(void)
{
  ForceFoot_UpdateActivityLed();

  if (g_dxl.tx_pending == 0U) {
    return;
  }

  /* Enable 74HC125N output sebelum transmit */
  DxlScanResponder_SetDirectionTx(&g_dxl);
  
  HAL_UART_Transmit(g_dxl.huart, g_dxl.tx_buffer, g_dxl.tx_size, 10U);
  while (__HAL_UART_GET_FLAG(g_dxl.huart, UART_FLAG_TC) == RESET) {
  }
  
  /* Disable 74HC125N output setelah transmit selesai */
  DxlScanResponder_SetDirectionRx(&g_dxl);
  
  g_dxl.tx_pending = 0U;
  Debug_SetTxPendingIndicator(0U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != g_dxl.huart) {
    return;
  }

  ForceFoot_MarkBusActivity();
  HAL_GPIO_TogglePin(DBG_RX_GPIO_Port, DBG_RX_Pin);
  DxlScanResponder_ProcessByte(&g_dxl, g_dxl.rx_byte);
  DxlScanResponder_RestartReception(&g_dxl);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != g_dxl.huart) {
    return;
  }

  __HAL_UART_CLEAR_PEFLAG(g_dxl.huart);
  __HAL_UART_CLEAR_FEFLAG(g_dxl.huart);
  __HAL_UART_CLEAR_NEFLAG(g_dxl.huart);
  __HAL_UART_CLEAR_OREFLAG(g_dxl.huart);
  DxlScanResponder_ResetParser(&g_dxl);
  DxlScanResponder_RestartReception(&g_dxl);
}

static HAL_StatusTypeDef DxlScanResponder_Start(DxlScanResponder *ctx)
{
  DxlScanResponder_SetDirectionRx(ctx);
  return HAL_UART_Receive_IT(ctx->huart, &ctx->rx_byte, 1U);
}

static void ForceFoot_MarkBusActivity(void)
{
  g_dxl_last_rx_tick = HAL_GetTick();
  g_dxl_rx_seen = 1U;
}

static void ForceFoot_UpdateActivityLed(void)
{
  uint32_t elapsed;

  if (g_dxl_rx_seen == 0U) {
    LED_Off();
    return;
  }

  elapsed = HAL_GetTick() - g_dxl_last_rx_tick;
  if (elapsed < DXL_LED_ACTIVITY_MS) {
    LED_On();
  } else {
    LED_Off();
  }
}

static void Debug_SetTxPendingIndicator(uint8_t active)
{
  if (active != 0U) {
    LED_On();
  } else {
    LED_Off();
  }
}

static void DxlScanResponder_InitControlTable(DxlScanResponder *ctx)
{
  uint16_t i;

  /* SIMPLIFIED like ForceFoot GitHub: Just basic register setup */
  memset(ctx->control_table, 0, sizeof(ctx->control_table));

  /* Basic ID register */
  ctx->control_table[DXL_REG_ID] = ctx->id;
  
  /* Model info - simulating virtual sensor device */
  DxlScanResponder_WriteWord(ctx, DXL_REG_MODEL_NUMBER, 5004U);  /* Virtual sensor model */
  ctx->control_table[DXL_REG_FIRMWARE_VERSION] = 1U;
  
  /* Protocol 2.0 settings */
  ctx->control_table[DXL_REG_PROTOCOL_TYPE] = 2U;
  ctx->control_table[DXL_REG_STATUS_RETURN_LEVEL] = 2U;
  
  /* Initialize sensor data area with zeros */
  for (i = 0U; i < (DXL_NUM_SENSORS * DXL_SENSOR_BYTES_PER_SENSOR); i++) {
    ctx->control_table[DXL_SENSOR_DATA_START + i] = 0U;
  }
}

/* Simplified telemetry - only updates sensor area now */
static void DxlScanResponder_UpdateTelemetry(DxlScanResponder *ctx)
{
  /* This function is now minimal - sensor update is done in READ handler */
  (void)ctx;  /* Unused parameter */
}

static void DxlScanResponder_ProcessByte(DxlScanResponder *ctx, uint8_t byte)
{
  if (ctx->rx_size >= DXL_PACKET_MAX_SIZE) {
    DxlScanResponder_ResetParser(ctx);
  }

  ctx->rx_buffer[ctx->rx_size++] = byte;
  DxlScanResponder_ProcessBuffer(ctx);
}

static void DxlScanResponder_SetDirectionRx(DxlScanResponder *ctx)
{
  /* For 74HC125N: Disable buffer output (OE=HIGH) ketika receive
     Ini memungkinkan pembaca lain untuk menulis di bus */
  HC125N_Disable();
  
  /* Original code: HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, GPIO_PIN_SET); */
  HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, GPIO_PIN_SET);
}

static void DxlScanResponder_SetDirectionTx(DxlScanResponder *ctx)
{
  /* For 74HC125N: Enable buffer output (OE=LOW) ketika transmit
     Ini mengaktifkan output driver untuk mengirim data ke bus */
  HC125N_Enable();
  
  /* Original code: HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, GPIO_PIN_RESET); */
  HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, GPIO_PIN_RESET);
}

static void DxlScanResponder_ResetParser(DxlScanResponder *ctx)
{
  ctx->rx_size = 0U;
}

static uint16_t DxlScanResponder_UpdateCrc(uint16_t crc,
                                           const uint8_t *data,
                                           uint16_t length)
{
  static const uint16_t crc_table[256] = {
      0x0000U, 0x8005U, 0x800FU, 0x000AU, 0x801BU, 0x001EU, 0x0014U, 0x8011U,
      0x8033U, 0x0036U, 0x003CU, 0x8039U, 0x0028U, 0x802DU, 0x8027U, 0x0022U,
      0x8063U, 0x0066U, 0x006CU, 0x8069U, 0x0078U, 0x807DU, 0x8077U, 0x0072U,
      0x0050U, 0x8055U, 0x805FU, 0x005AU, 0x804BU, 0x004EU, 0x0044U, 0x8041U,
      0x80C3U, 0x00C6U, 0x00CCU, 0x80C9U, 0x00D8U, 0x80DDU, 0x80D7U, 0x00D2U,
      0x00F0U, 0x80F5U, 0x80FFU, 0x00FAU, 0x80EBU, 0x00EEU, 0x00E4U, 0x80E1U,
      0x00A0U, 0x80A5U, 0x80AFU, 0x00AAU, 0x80BBU, 0x00BEU, 0x00B4U, 0x80B1U,
      0x8093U, 0x0096U, 0x009CU, 0x8099U, 0x0088U, 0x808DU, 0x8087U, 0x0082U,
      0x8183U, 0x0186U, 0x018CU, 0x8189U, 0x0198U, 0x819DU, 0x8197U, 0x0192U,
      0x01B0U, 0x81B5U, 0x81BFU, 0x01BAU, 0x81ABU, 0x01AEU, 0x01A4U, 0x81A1U,
      0x01E0U, 0x81E5U, 0x81EFU, 0x01EAU, 0x81FBU, 0x01FEU, 0x01F4U, 0x81F1U,
      0x81D3U, 0x01D6U, 0x01DCU, 0x81D9U, 0x01C8U, 0x81CDU, 0x81C7U, 0x01C2U,
      0x0140U, 0x8145U, 0x814FU, 0x014AU, 0x815BU, 0x015EU, 0x0154U, 0x8151U,
      0x8173U, 0x0176U, 0x017CU, 0x8179U, 0x0168U, 0x816DU, 0x8167U, 0x0162U,
      0x8123U, 0x0126U, 0x012CU, 0x8129U, 0x0138U, 0x813DU, 0x8137U, 0x0132U,
      0x0110U, 0x8115U, 0x811FU, 0x011AU, 0x810BU, 0x010EU, 0x0104U, 0x8101U,
      0x8303U, 0x0306U, 0x030CU, 0x8309U, 0x0318U, 0x831DU, 0x8317U, 0x0312U,
      0x0330U, 0x8335U, 0x833FU, 0x033AU, 0x832BU, 0x032EU, 0x0324U, 0x8321U,
      0x0360U, 0x8365U, 0x836FU, 0x036AU, 0x837BU, 0x037EU, 0x0374U, 0x8371U,
      0x8353U, 0x0356U, 0x035CU, 0x8359U, 0x0348U, 0x834DU, 0x8347U, 0x0342U,
      0x03C0U, 0x83C5U, 0x83CFU, 0x03CAU, 0x83DBU, 0x03DEU, 0x03D4U, 0x83D1U,
      0x83F3U, 0x03F6U, 0x03FCU, 0x83F9U, 0x03E8U, 0x83EDU, 0x83E7U, 0x03E2U,
      0x83A3U, 0x03A6U, 0x03ACU, 0x83A9U, 0x03B8U, 0x83BDU, 0x83B7U, 0x03B2U,
      0x0390U, 0x8395U, 0x839FU, 0x039AU, 0x838BU, 0x038EU, 0x0384U, 0x8381U,
      0x0280U, 0x8285U, 0x828FU, 0x028AU, 0x829BU, 0x029EU, 0x0294U, 0x8291U,
      0x82B3U, 0x02B6U, 0x02BCU, 0x82B9U, 0x02A8U, 0x82ADU, 0x82A7U, 0x02A2U,
      0x82E3U, 0x02E6U, 0x02ECU, 0x82E9U, 0x02F8U, 0x82FDU, 0x82F7U, 0x02F2U,
      0x02D0U, 0x82D5U, 0x82DFU, 0x02DAU, 0x82CBU, 0x02CEU, 0x02C4U, 0x82C1U,
      0x8243U, 0x0246U, 0x024CU, 0x8249U, 0x0258U, 0x825DU, 0x8257U, 0x0252U,
      0x0270U, 0x8275U, 0x827FU, 0x027AU, 0x826BU, 0x026EU, 0x0264U, 0x8261U,
      0x0220U, 0x8225U, 0x822FU, 0x022AU, 0x823BU, 0x023EU, 0x0234U, 0x8231U,
      0x8213U, 0x0216U, 0x021CU, 0x8219U, 0x0208U, 0x820DU, 0x8207U, 0x0202U
  };
  uint16_t i;
  uint16_t table_index;

  for (i = 0U; i < length; i++) {
    table_index = (uint16_t)(((crc >> 8U) ^ data[i]) & 0x00FFU);
    crc = (uint16_t)((crc << 8U) ^ crc_table[table_index]);
  }

  return crc;
}

static uint16_t DxlScanResponder_RemoveStuffing(const uint8_t *src,
                                                uint16_t src_len,
                                                uint8_t *dst,
                                                uint16_t dst_max)
{
  uint16_t src_index = 0U;
  uint16_t dst_index = 0U;

  while (src_index < src_len) {
    if ((src_index + 3U < src_len) &&
        (src[src_index] == 0xFFU) &&
        (src[src_index + 1U] == 0xFFU) &&
        (src[src_index + 2U] == 0xFDU) &&
        (src[src_index + 3U] == 0xFDU)) {
      if ((dst_index + 3U) > dst_max) {
        return 0U;
      }
      dst[dst_index++] = 0xFFU;
      dst[dst_index++] = 0xFFU;
      dst[dst_index++] = 0xFDU;
      src_index = (uint16_t)(src_index + 4U);
      continue;
    }

    if (dst_index >= dst_max) {
      return 0U;
    }
    dst[dst_index++] = src[src_index++];
  }

  return dst_index;
}

static uint16_t DxlScanResponder_ApplyStuffing(const uint8_t *src,
                                               uint16_t src_len,
                                               uint8_t *dst,
                                               uint16_t dst_max)
{
  uint16_t src_index;
  uint16_t dst_index = 0U;

  for (src_index = 0U; src_index < src_len; src_index++) {
    if (dst_index >= dst_max) {
      return 0U;
    }

    dst[dst_index++] = src[src_index];

    if ((dst_index >= 3U) &&
        (dst[dst_index - 3U] == 0xFFU) &&
        (dst[dst_index - 2U] == 0xFFU) &&
        (dst[dst_index - 1U] == 0xFDU)) {
      if (dst_index >= dst_max) {
        return 0U;
      }
      dst[dst_index++] = 0xFDU;
    }
  }

  return dst_index;
}

static void DxlScanResponder_ProcessBuffer(DxlScanResponder *ctx)
{
  uint16_t packet_length;
  uint16_t total_length;
  uint16_t crc_expected;
  uint16_t crc_received;
  uint8_t packet_id;
  uint8_t instruction;
  uint8_t decoded[DXL_PACKET_MAX_SIZE];
  uint16_t decoded_size;
  const uint8_t *params;
  uint16_t param_count;

  while (ctx->rx_size >= 4U) {
    if ((ctx->rx_buffer[0] == 0xFFU) &&
        (ctx->rx_buffer[1] == 0xFFU) &&
        (ctx->rx_buffer[2] == 0xFDU) &&
        (ctx->rx_buffer[3] == 0x00U)) {
      break;
    }

    memmove(ctx->rx_buffer, &ctx->rx_buffer[1], (size_t)(ctx->rx_size - 1U));
    ctx->rx_size--;
  }

  if (ctx->rx_size < 7U) {
    return;
  }

  packet_length = (uint16_t)ctx->rx_buffer[5] | (uint16_t)((uint16_t)ctx->rx_buffer[6] << 8U);
  if ((packet_length < 3U) || (packet_length > (DXL_PACKET_MAX_SIZE - 7U))) {
    memmove(ctx->rx_buffer, &ctx->rx_buffer[1], (size_t)(ctx->rx_size - 1U));
    ctx->rx_size--;
    return;
  }

  total_length = (uint16_t)(packet_length + 7U);
  if (ctx->rx_size < total_length) {
    return;
  }

  crc_expected = DxlScanResponder_UpdateCrc(0U, ctx->rx_buffer, (uint16_t)(total_length - 2U));
  crc_received = (uint16_t)ctx->rx_buffer[total_length - 2U]
               | (uint16_t)((uint16_t)ctx->rx_buffer[total_length - 1U] << 8U);

  if (crc_expected != crc_received) {
    memmove(ctx->rx_buffer, &ctx->rx_buffer[1], (size_t)(ctx->rx_size - 1U));
    ctx->rx_size--;
    return;
  }

  HAL_GPIO_TogglePin(DBG_CRC_GPIO_Port, DBG_CRC_Pin);

  packet_id = ctx->rx_buffer[4];
  decoded_size = DxlScanResponder_RemoveStuffing(&ctx->rx_buffer[7],
                                                 (uint16_t)(packet_length - 2U),
                                                 decoded,
                                                 DXL_PACKET_MAX_SIZE);
  if (decoded_size == 0U) {
    DxlScanResponder_ResetParser(ctx);
    return;
  }

  instruction = decoded[0];
  params = &decoded[1];
  param_count = (uint16_t)(decoded_size - 1U);

  DxlScanResponder_HandlePacket(ctx, packet_id, instruction, params, param_count);

  if (ctx->rx_size > total_length) {
    memmove(ctx->rx_buffer, &ctx->rx_buffer[total_length], (size_t)(ctx->rx_size - total_length));
  }
  ctx->rx_size = (uint16_t)(ctx->rx_size - total_length);
}

static uint8_t DxlScanResponder_ReadRegister(const DxlScanResponder *ctx, uint16_t address)
{
  if (address >= DXL_CONTROL_TABLE_SIZE) {
    return 0U;
  }

  return ctx->control_table[address];
}

static uint16_t DxlScanResponder_ReadWord(const DxlScanResponder *ctx, uint16_t address)
{
  return (uint16_t)DxlScanResponder_ReadRegister(ctx, address)
       | (uint16_t)((uint16_t)DxlScanResponder_ReadRegister(ctx, (uint16_t)(address + 1U)) << 8U);
}

static uint32_t DxlScanResponder_ReadDword(const DxlScanResponder *ctx, uint16_t address)
{
  return (uint32_t)DxlScanResponder_ReadRegister(ctx, address)
       | ((uint32_t)DxlScanResponder_ReadRegister(ctx, (uint16_t)(address + 1U)) << 8U)
       | ((uint32_t)DxlScanResponder_ReadRegister(ctx, (uint16_t)(address + 2U)) << 16U)
       | ((uint32_t)DxlScanResponder_ReadRegister(ctx, (uint16_t)(address + 3U)) << 24U);
}

static void DxlScanResponder_WriteWord(DxlScanResponder *ctx, uint16_t address, uint16_t value)
{
  if (address >= (DXL_CONTROL_TABLE_SIZE - 1U)) {
    return;
  }

  ctx->control_table[address] = (uint8_t)(value & 0xFFU);
  ctx->control_table[address + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void DxlScanResponder_WriteDword(DxlScanResponder *ctx, uint16_t address, uint32_t value)
{
  if (address >= (DXL_CONTROL_TABLE_SIZE - 3U)) {
    return;
  }

  ctx->control_table[address] = (uint8_t)(value & 0xFFU);
  ctx->control_table[address + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
  ctx->control_table[address + 2U] = (uint8_t)((value >> 16U) & 0xFFU);
  ctx->control_table[address + 3U] = (uint8_t)((value >> 24U) & 0xFFU);
}

/* SIMPLIFIED: Like ForceFoot GitHub, WRITE is practically ignored.
   Only allow writing to ID and basic config, which will be silently ignored anyway */
static uint8_t DxlScanResponder_IsWritableAddress(uint16_t address)
{
  /* By design, like ForceFoot GitHub:
     - ID write is "allowed" but we don't change anything
     - Everything else is read-only
     This makes WRITE instruction work but have no effect (similar to GitHub) */
  if (address == DXL_REG_ID) {
    return 1U;  /* "Allow" but ignored */
  }
  
  return 0U;  /* Everything else is read-only */
}

static void DxlScanResponder_WriteRegister(DxlScanResponder *ctx, uint16_t address, uint8_t value)
{
  if (address >= DXL_CONTROL_TABLE_SIZE) {
    return;
  }

  if (DxlScanResponder_IsWritableAddress(address) == 0U) {
    return;
  }

  if ((ctx->control_table[DXL_REG_TORQUE_ENABLE] != 0U) && (address < 64U)) {
    return;
  }

  ctx->control_table[address] = value;
  if (address == DXL_REG_ID) {
    ctx->id = value;
  }
}

static uint8_t DxlScanResponder_WriteBlock(DxlScanResponder *ctx,
                                           uint16_t address,
                                           const uint8_t *data,
                                           uint16_t length)
{
  uint16_t i;

  if ((length == 0U) || ((uint32_t)address + length > DXL_CONTROL_TABLE_SIZE)) {
    return 0x07U;
  }

  for (i = 0U; i < length; i++) {
    if (DxlScanResponder_IsWritableAddress((uint16_t)(address + i)) == 0U) {
      return 0x07U;
    }
    if ((ctx->control_table[DXL_REG_TORQUE_ENABLE] != 0U) && ((address + i) < 64U)) {
      return 0x07U;
    }
  }

  for (i = 0U; i < length; i++) {
    DxlScanResponder_WriteRegister(ctx, (uint16_t)(address + i), data[i]);
  }

  return 0U;
}

static uint8_t DxlScanResponder_ShouldRespond(const DxlScanResponder *ctx,
                                              uint8_t packet_id,
                                              uint8_t instruction)
{
  uint8_t level = ctx->control_table[DXL_REG_STATUS_RETURN_LEVEL];

  if (packet_id == DXL_BROADCAST_ID) {
    return (instruction == DXL_INST_PING) ? 1U : 0U;
  }

  if ((instruction == DXL_INST_PING) || (instruction == DXL_INST_READ)) {
    return 1U;
  }

  if (level == 2U) {
    return 1U;
  }

  if ((level == 1U) && (instruction != DXL_INST_WRITE) && (instruction != DXL_INST_REBOOT)) {
    return 1U;
  }

  return 0U;
}

static void DxlScanResponder_HandlePacket(DxlScanResponder *ctx,
                                          uint8_t packet_id,
                                          uint8_t instruction,
                                          const uint8_t *params,
                                          uint16_t param_count)
{
  uint16_t address;
  uint16_t read_length;
  uint16_t i;
  uint8_t reply[DXL_PACKET_MAX_SIZE];

  if ((packet_id != ctx->id) && (packet_id != DXL_BROADCAST_ID)) {
    return;
  }

  HAL_GPIO_TogglePin(DBG_PKT_GPIO_Port, DBG_PKT_Pin);
  DxlScanResponder_UpdateTelemetry(ctx);

  switch (instruction) {
    case DXL_INST_PING:
      reply[0] = (uint8_t)(DXL_XH540_MODEL_NUMBER & 0xFFU);
      reply[1] = (uint8_t)((DXL_XH540_MODEL_NUMBER >> 8U) & 0xFFU);
      reply[2] = DXL_XH540_FIRMWARE_VERSION;
      if (DxlScanResponder_ShouldRespond(ctx, packet_id, instruction) != 0U) {
        DxlScanResponder_QueueStatus(ctx, packet_id, 0U, reply, 3U);
      }
      break;

    case DXL_INST_READ:
      if ((packet_id == DXL_BROADCAST_ID) || (param_count != 4U)) {
        if (packet_id != DXL_BROADCAST_ID) {
          DxlScanResponder_QueueStatus(ctx, packet_id, 0x03U, NULL, 0U);
        }
        return;
      }

      /* LIKE FORCEFOOT GITHUB: Read current sensor values and update control table */
      DxlScanResponder_ReadSensors();
      DxlScanResponder_UpdateSensorData(ctx);

      address = (uint16_t)params[0] | (uint16_t)((uint16_t)params[1] << 8U);
      read_length = (uint16_t)params[2] | (uint16_t)((uint16_t)params[3] << 8U);
      if ((read_length == 0U) || (read_length > (DXL_PACKET_MAX_SIZE - 11U)) ||
          ((uint32_t)address + read_length > DXL_CONTROL_TABLE_SIZE)) {
        DxlScanResponder_QueueStatus(ctx, packet_id, 0x04U, NULL, 0U);
        return;
      }

      for (i = 0U; i < read_length; i++) {
        reply[i] = DxlScanResponder_ReadRegister(ctx, (uint16_t)(address + i));
      }

      if (DxlScanResponder_ShouldRespond(ctx, packet_id, instruction) != 0U) {
        DxlScanResponder_QueueStatus(ctx, packet_id, 0U, reply, read_length);
      }
      break;

    case DXL_INST_WRITE:
      /* LIKE FORCEFOOT GITHUB: WRITE is practically ignored
         Acknowledge the write but don't do anything */
      if (packet_id != DXL_BROADCAST_ID) {
        if (DxlScanResponder_ShouldRespond(ctx, packet_id, instruction) != 0U) {
          DxlScanResponder_QueueStatus(ctx, packet_id, 0U, NULL, 0U);
        }
      }
      break;

    case DXL_INST_REBOOT:
      if (DxlScanResponder_ShouldRespond(ctx, packet_id, instruction) != 0U) {
        DxlScanResponder_QueueStatus(ctx, packet_id, 0U, NULL, 0U);
      }
      break;

    default:
      if (packet_id != DXL_BROADCAST_ID) {
        DxlScanResponder_QueueStatus(ctx, packet_id, 0x02U, NULL, 0U);
      }
      break;
  }
}

static void DxlScanResponder_QueueStatus(DxlScanResponder *ctx,
                                         uint8_t packet_id,
                                         uint8_t error,
                                         const uint8_t *params,
                                         uint16_t param_count)
{
  uint16_t i;
  uint16_t stuffed_length;
  uint16_t packet_length;
  uint16_t packet_size;
  uint16_t crc;
  uint8_t stuffed[DXL_PACKET_MAX_SIZE];

  stuffed[0] = DXL_INST_STATUS;
  stuffed[1] = error;
  for (i = 0U; i < param_count; i++) {
    stuffed[2U + i] = (params != NULL) ? params[i] : 0U;
  }

  stuffed_length = DxlScanResponder_ApplyStuffing(stuffed,
                                                  (uint16_t)(param_count + 2U),
                                                  &ctx->tx_buffer[7],
                                                  (uint16_t)(DXL_PACKET_MAX_SIZE - 9U));
  if (stuffed_length == 0U) {
    return;
  }

  packet_length = (uint16_t)(stuffed_length + 2U);
  packet_size = (uint16_t)(packet_length + 7U);

  if (packet_size > DXL_PACKET_MAX_SIZE) {
    return;
  }

  ctx->tx_buffer[0] = 0xFFU;
  ctx->tx_buffer[1] = 0xFFU;
  ctx->tx_buffer[2] = 0xFDU;
  ctx->tx_buffer[3] = 0x00U;
  ctx->tx_buffer[4] = ctx->id;
  ctx->tx_buffer[5] = (uint8_t)(packet_length & 0xFFU);
  ctx->tx_buffer[6] = (uint8_t)((packet_length >> 8U) & 0xFFU);

  crc = DxlScanResponder_UpdateCrc(0U, ctx->tx_buffer, (uint16_t)(packet_size - 2U));
  ctx->tx_buffer[packet_size - 2U] = (uint8_t)(crc & 0xFFU);
  ctx->tx_buffer[packet_size - 1U] = (uint8_t)((crc >> 8U) & 0xFFU);
  ctx->tx_size = packet_size;
  ctx->tx_pending = 1U;
  Debug_SetTxPendingIndicator(1U);
}

static void DxlScanResponder_RestartReception(DxlScanResponder *ctx)
{
  HAL_UART_Receive_IT(ctx->huart, &ctx->rx_byte, 1U);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_Init();
  HC125N_Init();        /* Initialize 74HC125N buffer */
  HX711_Init_Pins();
  HX711_Calibrate_Offset();
  ForceFoot_DxlScan_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ForceFoot_DxlScan_Task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

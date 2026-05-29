# 74HC125N 3-State Buffer Configuration

## Ringkasan Modifikasi
Kode telah dimodifikasi untuk mengintegrasikan IC **74HC125N** sebagai 3-state buffer untuk komunikasi U2D2 dengan proteksi bus.

---

## Skema Pin 74HC125N

```
    ┌─────────────────────────────┐
    │     74HC125N IC             │
    │  3-State Buffer x4          │
    ├─────────────────────────────┤
    │  PIN  │  SIGNAL   │  STM32  │
    ├───────┼───────────┼─────────┤
    │  1    │ 1OE       │  PB12   │  ◄── Output Enable (active LOW)
    │  2    │ 1Y        │  PA9    │  ◄── Data OUTPUT ke U2D2
    │  3    │ 1A        │  PA8    │  ◄── Data INPUT dari Sensor
    │  8    │ GND       │  GND    │
    │  16   │ VCC (+5V) │  +5V    │
    └─────────────────────────────┘
    
    Note: Channels 2-4 (pins 4-7, 9-12, 13-15) tidak digunakan
```

---

## Konfigurasi Hardware Lengkap

### 1. Pin STM32F401CEU (Dynamixel Protocol)
- **PA9** (UART1 RX → 74HC125N Output): Data ke U2D2
- **PA10** (UART1 TX): TX line untuk komunikasi
- **PA8**: Data input dengan pull-up 4.7K
- **PB12**: Output Enable untuk 74HC125N

### 2. Resistor Pull-Up (Eksternal pada PCB)
```
  U2D2 DATA ────┬────────────────┐
                │                │
             [4.7K]             PA9 (STM32)
                │                │
              3.3V              ◄──
              (VCC)             74HC125N Pin 2
```

### 3. Status Register Encoding (74HC125N)
| OE (PB12) | 1Y (PA9)     | Kondisi        |
|-----------|--------------|----------------|
| **0 (LOW)**   | 1A level   | **OUTPUT AKTIF** |
| **1 (HIGH)**  | High-Z     | **TRI-STATE**   |

---

## Perubahan Kode Software

### Fungsi-Fungsi Baru di main.c

```c
/* Initialize 74HC125N Control */
static void HC125N_Init(void)
  └─ Konfigurasi PB12 (OE), PA9 (output), PA8 (input)

/* Enable Output (OE=0) - Transmit Mode */
static void HC125N_Enable(void)

/* Disable Output (OE=1) - Receive Mode */
static void HC125N_Disable(void)

/* Write data ke buffer output */
static void HC125N_WriteData(uint8_t value)

/* Read data dari buffer input */
static uint8_t HC125N_ReadData(void)
```

### Modifikasi Existing Functions

#### DxlScanResponder_SetDirectionRx()
```c
// Disable buffer (OE=HIGH) untuk menerima data dari U2D2
HC125N_Disable();  // ◄── BARU
HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, GPIO_PIN_SET);
```

#### DxlScanResponder_SetDirectionTx()
```c
// Enable buffer (OE=LOW) untuk mengirim data ke U2D2
HC125N_Enable();   // ◄── BARU
HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, GPIO_PIN_RESET);
```

#### ForceFoot_DxlScan_Task()
```c
HC125N_Enable();   // ◄── Enable buffer sebelum TX
HAL_UART_Transmit(g_dxl.huart, ...);
HC125N_Disable();  // ◄── Disable buffer setelah TX
```

---

## Flow Komunikasi

```
┌─────────────────────────────────────────────┐
│          STM32F401CEU                       │
│  ┌────────────────────────────────────────┐ │
│  │  UART1:                                 │ │
│  │  TX: PA10 ──────────────────────────┐   │ │
│  │  RX: From U2D2 (PA9 dari 74HC125N)  │   │ │
│  │                                      │   │ │
│  │  GPIO Control:                       │   │ │
│  │  PB12 ◄─ 74HC125N Output Enable      │   │ │
│  │  PA8  ◄─ 74HC125N Data Input         │   │ │
│  └────────────────────────────────────────┘ │
│           │                                  │
│           │                                  │
│           ▼                                  │
│      ┌─────────────┐                        │
│      │ 74HC125N    │                        │
│      │  Buffer     │                        │
│      │             │                        │
│      │ 1A ──► 1Y   │                        │
│      │ OE controls │                        │
│      └─────────────┘                        │
│           │                                  │
│           ▼ (PA9)                           │
└─────────────────────────────────────────────┘
           │
           ▼ (Pull-up 4.7K ke 3.3V)
    ┌──────────────┐
    │   U2D2       │
    │  (Module)    │
    └──────────────┘
```

---

## Operasi Mode

### MODE RX (Menerima dari U2D2)
```
1. HC125N_Disable() → PB12 = HIGH
2. Output 74HC125N: TRI-STATE (high impedance)
3. U2D2 dapat menulis ke PA9 (pull-up aktif)
4. STM32 UART RX membaca data
```

### MODE TX (Mengirim ke U2D2)
```
1. HC125N_Enable() → PB12 = LOW
2. Output 74HC125N: AKTIF (mengikuti PA8 input)
3. PA8 (dari sensor) diteruskan ke PA9 (ke U2D2)
4. Data terkirim melalui buffer dengan proteksi
```

---

## Pinout Reference (STM32F401CEU)

```
┌───────────────────────────────────────┐
│  STM32F401CEU LQFP48 Pinout           │
├───────────────────────────────────────┤
│ Port A:                               │
│  PA8  = GPIO Input (HC125N 1A)        │
│  PA9  = GPIO Open-Drain (HC125N 1Y)   │
│  PA10 = UART1 TX                      │
│                                       │
│ Port B:                               │
│  PB12 = GPIO Output (HC125N OE)       │
│                                       │
│ Port C:                               │
│  PC13 = LED Indicator                 │
└───────────────────────────────────────┘
```

---

## Troubleshooting

### Problem: Data tidak terkirim ke U2D2
**Solution:**
1. Cek resistor pull-up 4.7K sudah terpasang di PA9 ke 3.3V
2. Verifikasi PB12 dapat toggle (LOW saat TX, HIGH saat RX)
3. Periksa supply voltage 74HC125N (16pin = +5V, pin 8 = GND)

### Problem: Sinyal corrupted
**Solution:**
1. Periksa timing toggle PB12 (harus sebelum TX/RX)
2. Validasi pull-up resistor value (harus 4.7K ± 10%)
3. Gunakan osiloskop untuk verify PA9 signal integrity

### Problem: Buffer tidak merespon
**Solution:**
1. Periksa koneksi VCC ke 74HC125N (pin 16)
2. Verifikasi GND connection (pin 8)
3. Test dengan LED toggle untuk debug output enable

---

## Catatan Penting

⚠️ **PERHATIAN:**
- Pastikan pull-up resistor **4.7K** terpasang di hardware
- Timing toggle PB12 harus sinkron dengan UART
- Voltage 74HC125N = **+5V** (NOT 3.3V)
- STM32 GPIO **open-drain** untuk PA9 agar kompatibel dengan pull-up

---

## Referensi IC 74HC125N

- **Datasheet**: Texas Instruments 74HC125 Quad 3-State Buffer
- **Aplikasi**: Bus buffering, line driver protection, multi-device communication
- **Max Frequency**: 25 MHz
- **Operating Voltage**: 2V - 6V (recommended 5V)
- **Output Impedance (Z)**: ~30Ω push, ~30Ω pull

---

## Testing Checklist

- [ ] Compile kode tanpa error
- [ ] Verify PB12 toggle saat TX/RX
- [ ] Check PA9 voltage levels (0-3.3V)
- [ ] Monitor PA8 input level
- [ ] Verify U2D2 komunikasi dengan Dynamixel
- [ ] Test calibration HX711 loadcell
- [ ] Verify sensor data reading


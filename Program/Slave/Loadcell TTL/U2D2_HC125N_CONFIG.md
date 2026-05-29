# U2D2 TTL dengan 74HC125N TX Buffer Configuration

## Konfigurasi Hardware

```
╔════════════════════════════════════════════════════════════════╗
║                  STM32F401CEU (Loadcell TTL)                  ║
╠════════════════════════════════════════════════════════════════╣
║                                                                ║
║  UART1 TX (PA9)  ────────┐                                    ║
║                          ▼                                    ║
║                   ┌──────────────┐                            ║
║                   │ 74HC125N     │  Pin 1: 1OE ◄─ PB12       ║
║                   │ 3-State      │  Pin 2: 1A                ║
║                   │ Buffer       │  Pin 3: 1Y                ║
║                   │              │  Pin 8: GND               ║
║                   │              │  Pin 16: +5V              ║
║                   └──────────────┘                            ║
║                          │                                    ║
║                          ▼ (HC125N Output with 4.7K pull-up)║
║                                                              ║
║  UART1 RX (PA10) ───────────────┐                           ║
║                                 ▼                           ║
║                          U2D2 DATA (TTL)                    ║
║                                                              ║
║  GPIO PB12 (Output Enable) ──────────────────────────────   ║
║                                                              ║
╚════════════════════════════════════════════════════════════════╝
```

## Alur Komunikasi

### MODE TRANSMIT (TX)
```
STM32 TX (PA9)
    │
    ├─ UART1 mengirim data
    │
    ▼
HC125N Input (Pin 1A / Pin 2)
    │
    ├─ PB12 = LOW (OE aktif)
    │
    ▼
HC125N Output (Pin 1Y / Pin 3) ──► U2D2 DATA (dengan 4.7K pull-up)
```

### MODE RECEIVE (RX)
```
U2D2 DATA (TTL) ──► STM32 RX (PA10)
                          │
                          ├─ UART1 menerima data
                          │
                          ▼
                    HC125N Output: TRI-STATE
                    (PB12 = HIGH, OE nonaktif)
```

## Pin Configuration

| Pin    | Function        | Mode            | Notes                          |
|--------|-----------------|-----------------|--------------------------------|
| PA9    | UART1 TX        | Alternate UART  | Input 74HC125N Pin 1A (Pin 2) |
| PA10   | UART1 RX        | Alternate UART  | Direct dari U2D2 DATA          |
| PB12   | HC125N OE       | GPIO Output PP  | Output Enable control          |

## HC125N Pin Mapping

| HC125N Pin | Signal | STM32 Pin | Description                    |
|-----------|--------|-----------|--------------------------------|
| 1         | 1OE    | PB12      | Output Enable (active LOW)     |
| 2         | 1A     | PA9       | Data Input dari UART1 TX       |
| 3         | 1Y     | -         | Data Output ke U2D2            |
| 8         | GND    | GND       | Ground                         |
| 16        | VCC    | +5V       | Power supply                   |

## Fungsi-Fungsi Kontrol

```c
/* Initialize 74HC125N (hanya setup PB12) */
HC125N_Init();

/* Enable TX buffer (OE=LOW) - siap mengirim ke U2D2 */
HC125N_Enable();

/* Disable TX buffer (OE=HIGH) - hanya bisa menerima dari U2D2 */
HC125N_Disable();

/* Cek status buffer */
uint8_t status = HC125N_IsEnabled();
```

## Operasi PB12

```
┌─────────────────────────────────┐
│ PB12 State Control              │
├─────────────────────────────────┤
│ PB12 = 0 (LOW)                  │
│  ├─ HC125N Output: AKTIF        │
│  ├─ Data PA9 → U2D2             │
│  └─ Mode: TRANSMIT              │
│                                 │
│ PB12 = 1 (HIGH)                 │
│  ├─ HC125N Output: TRI-STATE    │
│  ├─ Hanya RX dari U2D2          │
│  └─ Mode: RECEIVE               │
└─────────────────────────────────┘
```

## Timing Sequence

```
┌──────────────────────────────────────────────────┐
│ UART TX Sequence                                 │
├──────────────────────────────────────────────────┤
│ 1. Set PB12 = 0        (HC125N_Enable)          │
│ 2. Wait ~5µs           (setup time)              │
│ 3. Send data via UART  (HAL_UART_Transmit)     │
│ 4. Wait tx complete    (while TC flag)          │
│ 5. Set PB12 = 1        (HC125N_Disable)        │
│ 6. Ready for RX        (listen for response)    │
└──────────────────────────────────────────────────┘
```

## Hardware Requirements

✅ **IC 74HC125N**
   - Quad 3-state buffer
   - Supply: +5V
   - Kapasitas: 24mA typical per output

✅ **Resistor Pull-Up 4.7K**
   - Connected: U2D2 DATA line to 3.3V STM32

✅ **U2D2 Module**
   - USB to TTL converter
   - Baud rate: 1000000 (1Mbps)
   - Protokol: Dynamixel 2.0

## Catatan Penting

⚠️ **Jangan lupa:**
- PB12 harus dapat toggle dengan cepat (< 1µs)
- Resistor pull-up 4.7K **WAJIB** pada U2D2 DATA line
- HC125N supply harus +5V, bukan 3.3V
- GND HC125N harus common dengan STM32 GND

⚠️ **Voltage Levels:**
- STM32 GPIO: 0-3.3V
- HC125N Output: 0-5V (TTL compatible)
- U2D2 Input: TTL level (3.3V dianggap HIGH)

## Perubahan Kode dari Versi Sebelumnya

✓ **HC125N_Init()** - Hanya setup PB12 (OE control)
✓ **PA9, PA10** - Tetap UART1 (jangan ubah ke GPIO)
✓ **HC125N_Enable()** - Set PB12 = 0
✓ **HC125N_Disable()** - Set PB12 = 1

## Testing Steps

1. **Compile & Flash**
   ```
   make clean
   make
   st-flash write build/main.bin 0x08000000
   ```

2. **Monitor PB12 dengan Oscilloscope**
   - Harus toggle saat TX/RX transition
   - Frekuensi: sesuai dengan command rate

3. **Verify Data Line**
   - Monitor U2D2 DATA line dengan logic analyzer
   - Harus HIGH (pull-up) saat RX idle
   - Harus LOW/HIGH sesuai data saat TX

4. **Test Communication**
   - Send PING command ke Dynamixel via U2D2
   - Terima response
   - Monitor load cell data

---

**Kode siap untuk U2D2 TTL dengan 74HC125N TX Buffer!** ✅

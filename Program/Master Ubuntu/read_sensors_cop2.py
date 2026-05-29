import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

try:
    import msvcrt
except ImportError:  # pragma: no cover - non-Windows fallback
    msvcrt = None


HEADER = bytes((0xFF, 0xFF, 0xFD, 0x00))
INST_READ = 0x02
INST_STATUS = 0x55

DEFAULT_PORT = "COM5"
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_DXL_ID = 21
DEFAULT_ADDRESS = 160
DEFAULT_LENGTH = 12
DEFAULT_TIMEOUT = 0.2
DEFAULT_INTERVAL = 0.05
DEFAULT_THRESHOLD = 50
DEFAULT_CALIBRATION_SAMPLES = 20
DEFAULT_CALIBRATION_DELAY = 0.05
DEFAULT_CALIBRATION_KEY = "c"
DEFAULT_CENTER_REFERENCE_KEY = "z"
DEFAULT_BALANCE_TOLERANCE_KEY = "b"
DEFAULT_MEDIAN_WINDOW = 3
DEFAULT_CALIBRATION_FILE = Path(__file__).with_name("calibration.json") 
DEFAULT_REGRESSION_FILE = Path(__file__).with_name("regression_coefficients.json")
DEFAULT_BALANCE_TOLERANCE_FILE = Path(__file__).with_name("balance_tolerance.json")

DEFAULT_SENSOR_LAYOUT = (
    {"name": "front_left", "x": -4.0, "y": 4.0},
    {"name": "front_right", "x": 4.0, "y": 4.0},
    {"name": "rear_left", "x": -4.0, "y": -4.0},
    {"name": "rear_right", "x": 4.0, "y": -4.0},
)


def update_crc(crc_accum: int, data_blk: bytes) -> int:
    crc_table = (
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
    )

    for byte in data_blk:
        index = ((crc_accum >> 8) ^ byte) & 0xFF
        crc_accum = ((crc_accum << 8) ^ crc_table[index]) & 0xFFFF
    return crc_accum


def apply_stuffing(payload: bytes) -> bytes:
    stuffed = bytearray()
    for byte in payload:
        stuffed.append(byte)
        if len(stuffed) >= 3 and stuffed[-3:] == b"\xFF\xFF\xFD":
            stuffed.append(0xFD)
    return bytes(stuffed)


def remove_stuffing(payload: bytes) -> bytes:
    unstuffed = bytearray()
    index = 0
    while index < len(payload):
        if payload[index:index + 4] == b"\xFF\xFF\xFD\xFD":
            unstuffed.extend(b"\xFF\xFF\xFD")
            index += 4
            continue
        unstuffed.append(payload[index])
        index += 1
    return bytes(unstuffed)


def build_read_packet(dxl_id: int, address: int, length: int) -> bytes:
    params = bytes((
        address & 0xFF,
        (address >> 8) & 0xFF,
        length & 0xFF,
        (length >> 8) & 0xFF,
    ))
    body = bytes((INST_READ,)) + params
    stuffed = apply_stuffing(body)
    packet_length = len(stuffed) + 2

    packet = bytearray()
    packet.extend(HEADER)
    packet.append(dxl_id)
    packet.extend(packet_length.to_bytes(2, "little"))
    packet.extend(stuffed)

    crc = update_crc(0, packet)
    packet.extend(crc.to_bytes(2, "little"))
    return bytes(packet)


def format_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def read_exact_packet(port: serial.Serial, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    buffer = bytearray()

    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            buffer.extend(chunk)

        while len(buffer) >= 7:
            if buffer[:4] != HEADER:
                del buffer[0]
                continue

            packet_length = int.from_bytes(buffer[5:7], "little")
            total_length = packet_length + 7
            if len(buffer) < total_length:
                break

            packet = bytes(buffer[:total_length])
            del buffer[:total_length]
            return packet

    raise TimeoutError("Timeout menunggu status packet dari device")


def parse_status_packet(packet: bytes, expected_id: int, expected_param_length: int) -> bytes:
    if len(packet) < 11:
        raise ValueError("Packet terlalu pendek")
    if packet[:4] != HEADER:
        raise ValueError("Header packet tidak valid")
    if packet[4] != expected_id:
        raise ValueError(f"ID balasan tidak cocok: {packet[4]}")

    packet_length = int.from_bytes(packet[5:7], "little")
    if packet_length + 7 != len(packet):
        raise ValueError("Panjang packet tidak cocok")

    crc_expected = int.from_bytes(packet[-2:], "little")
    crc_actual = update_crc(0, packet[:-2])
    if crc_expected != crc_actual:
        raise ValueError(
            f"CRC tidak valid: dapat 0x{crc_expected:04X}, hitung 0x{crc_actual:04X}"
        )

    decoded = remove_stuffing(packet[7:-2])
    if len(decoded) < 2:
        raise ValueError("Isi status packet tidak lengkap")
    if decoded[0] != INST_STATUS:
        raise ValueError(f"Bukan status packet: instruksi 0x{decoded[0]:02X}")

    error = decoded[1]
    params = decoded[2:]
    if error != 0:
        raise ValueError(f"Device mengembalikan error 0x{error:02X}")
    if len(params) != expected_param_length:
        raise ValueError(
            f"Jumlah data tidak sesuai: {len(params)} byte, expected {expected_param_length}"
        )
    return params


def parse_sensor_values(payload: bytes) -> list[int]:
    if len(payload) % 3 != 0:
        raise ValueError("Payload sensor harus kelipatan 3 byte")

    sensors = []
    for index in range(0, len(payload), 3):
        value = payload[index] | (payload[index + 1] << 8) | (payload[index + 2] << 16)
        if value & 0x800000:
            value -= 1 << 24
        sensors.append(value)
    return sensors


def read_sensor_frame(
    port: serial.Serial,
    request: bytes,
    dxl_id: int,
    length: int,
    timeout: float,
) -> tuple[list[int], bytes]:
    port.reset_input_buffer()
    port.write(request)
    port.flush()

    response = read_exact_packet(port, timeout)
    payload = parse_status_packet(response, dxl_id, length)
    sensors = parse_sensor_values(payload)
    return sensors, response


def get_sensor_layout(sensor_count: int) -> list[dict[str, float | str]]:
    if sensor_count == len(DEFAULT_SENSOR_LAYOUT):
        return [dict(item) for item in DEFAULT_SENSOR_LAYOUT]

    return [
        {"name": f"sensor_{index + 1}", "x": float(index), "y": 0.0}
        for index in range(sensor_count)
    ]


def load_calibration(calibration_path: Path, sensor_count: int) -> list[float] | None:
    if not calibration_path.exists():
        return None

    with calibration_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    offsets = data.get("offsets")
    if not isinstance(offsets, list) or len(offsets) != sensor_count:
        raise ValueError("Isi calibration.json tidak cocok dengan jumlah sensor")
    return [float(value) for value in offsets]


def save_calibration(calibration_path: Path, offsets: list[float], sensor_layout: list[dict[str, float | str]]) -> None:
    payload = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "offsets": offsets,
        "sensor_layout": sensor_layout,
    }
    with calibration_path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2)


def load_regression_coefficients(regression_path: Path, sensor_count: int) -> list[float] | None:
    """Load koefisien regresi dari file JSON."""
    if not regression_path.exists():
        return None

    with regression_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    coefficients = data.get("coefficients")
    if not isinstance(coefficients, list) or len(coefficients) != sensor_count:
        raise ValueError("Isi regression_coefficients.json tidak cocok dengan jumlah sensor")
    return [float(value) for value in coefficients]


def save_regression_coefficients(
    regression_path: Path,
    coefficients: list[float],
    r_squared: float,
    rms_error: float,
    sensor_layout: list[dict[str, float | str]]
) -> None:
    """Simpan koefisien regresi ke file JSON."""
    payload = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "coefficients": coefficients,
        "r_squared": r_squared,
        "rms_error": rms_error,
        "sensor_layout": sensor_layout,
    }
    with regression_path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2)


def load_balance_tolerance(balance_tolerance_path: Path, sensor_count: int) -> list[float] | None:
    """Load balance tolerance range untuk setiap sensor dari file JSON."""
    if not balance_tolerance_path.exists():
        return None

    with balance_tolerance_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    tolerances = data.get("tolerances")
    if not isinstance(tolerances, list) or len(tolerances) != sensor_count:
        raise ValueError("Isi balance_tolerance.json tidak cocok dengan jumlah sensor")
    return [float(value) for value in tolerances]


def save_balance_tolerance(
    balance_tolerance_path: Path,
    tolerances: list[float],
    sensor_layout: list[dict[str, float | str]]
) -> None:
    """Simpan balance tolerance range untuk setiap sensor ke file JSON."""
    payload = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "tolerances": tolerances,
        "sensor_layout": sensor_layout,
    }
    with balance_tolerance_path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2)


def perform_offset_calibration(
    port: serial.Serial,
    request: bytes,
    dxl_id: int,
    length: int,
    timeout: float,
    sample_count: int,
    sample_delay: float,
) -> list[float]:
    """Kalibrasi offset (zero-point calibration) tanpa beban."""
    print("\n=== MODE KALIBRASI OFFSET ===")
    print("Pastikan sensor TIDAK ada beban sama sekali.")
    print(f"Mengambil {sample_count} sampel untuk menghitung offset...")

    sensor_sums: list[float] | None = None
    collected = 0

    while collected < sample_count:
        sensors, _ = read_sensor_frame(port, request, dxl_id, length, timeout)

        if sensor_sums is None:
            sensor_sums = [0.0] * len(sensors)

        for index, value in enumerate(sensors):
            sensor_sums[index] += value

        collected += 1
        print(f"  Sampel {collected}/{sample_count}: {sensors}")
        time.sleep(sample_delay)

    assert sensor_sums is not None
    offsets = [value / sample_count for value in sensor_sums]
    rounded_offsets = [round(value, 3) for value in offsets]

    print(f"Offset tersimpan: {rounded_offsets}\n")
    return offsets


def perform_center_reference_capture(
    port: serial.Serial,
    request: bytes,
    dxl_id: int,
    length: int,
    timeout: float,
    offsets: list[float],
    threshold: float,
    sample_count: int,
    sample_delay: float,
    coefficients: list[float] | None,
    use_absolute_force: bool,
    sensor_layout: list[dict[str, float | str]],
    rolling_windows: list,
    median_window: int,
) -> tuple[list[float], tuple[float, float] | None]:
    """Simpan kondisi awal sebagai referensi tengah."""
    print("\n=== MODE REFERENSI TENGAH ===")
    print("Ambil posisi awal yang ingin dianggap SEIMBANG DI TENGAH.")
    print(f"Mengambil {sample_count} sampel untuk referensi tengah...")

    sensor_sums: list[float] | None = None
    collected = 0

    while collected < sample_count:
        raw_values, _ = read_sensor_frame(port, request, dxl_id, length, timeout)
        # Aplikasikan median filter untuk konsistensi dengan real-time reading
        raw_values = apply_median_filter(raw_values, rolling_windows, median_window)
        corrected_values = apply_offsets(raw_values, offsets, threshold)

        if sensor_sums is None:
            sensor_sums = [0.0] * len(corrected_values)

        for index, value in enumerate(corrected_values):
            sensor_sums[index] += value

        collected += 1
        print(f"  Sampel referensi {collected}/{sample_count}: {format_sensor_line(corrected_values)}")
        time.sleep(sample_delay)

    assert sensor_sums is not None
    reference_values = [value / sample_count for value in sensor_sums]

    if coefficients:
        reference_sensor_forces = [value * coefficient for value, coefficient in zip(reference_values, coefficients)]
    else:
        reference_sensor_forces = reference_values

    reference_force_values = normalize_force_values(reference_sensor_forces, use_absolute_force)
    reference_cop = compute_cop(reference_force_values, sensor_layout)

    print(f"Referensi tengah tersimpan: {format_sensor_line(reference_values)}")
    if reference_cop is None:
        print("CoP referensi tidak dapat dihitung (beban terlalu kecil).\n")
    else:
        print(f"CoP referensi tersimpan: ({reference_cop[0]: .2f}, {reference_cop[1]: .2f})\n")

    # Reset rolling windows setelah capture referensi agar median filter mulai fresh
    reset_rolling_windows(rolling_windows, len(reference_values), median_window)

    return reference_values, reference_cop


def perform_balance_tolerance_capture(
    port: serial.Serial,
    request: bytes,
    dxl_id: int,
    length: int,
    timeout: float,
    offsets: list[float],
    threshold: float,
    sample_count: int,
    sample_delay: float,
) -> list[float]:
    """Tangkap range tolerance untuk setiap sensor saat posisi seimbang."""
    print("\n=== MODE KALIBRASI BALANCE TOLERANCE ===")
    print("Asumsikan posisi SEIMBANG dengan berdiri normal di tengah platform.")
    print(f"Mengambil {sample_count} sampel untuk menentukan range tolerance...")

    collected_samples: list[list[float]] = []
    collected = 0

    while collected < sample_count:
        raw_values, _ = read_sensor_frame(port, request, dxl_id, length, timeout)
        corrected_values = apply_offsets(raw_values, offsets, threshold)
        collected_samples.append(corrected_values)
        collected += 1
        print(f"  Sampel {collected}/{sample_count}: {format_sensor_line(corrected_values)}")
        time.sleep(sample_delay)

    # Hitung toleransi sebagai nilai maksimum absolut dari setiap sensor
    tolerances = []
    for sensor_idx in range(len(collected_samples[0])):
        max_abs_value = max(abs(sample[sensor_idx]) for sample in collected_samples)
        tolerances.append(max_abs_value)

    print(f"\nToleransi dihitung dari nilai maksimum absolut:")
    for idx, tol in enumerate(tolerances):
        print(f"  Sensor {idx + 1}: ±{tol:.2f}")
    print()

    return tolerances


def collect_regression_data(
    port: serial.Serial,
    request: bytes,
    dxl_id: int,
    length: int,
    timeout: float,
    offsets: list[float],
    num_points: int = 50,
    interval: float = 0.1,
) -> tuple[list[list[float]], list[float]]:
    """
    Kumpulkan data untuk regresi linear.
    User diminta untuk memberikan beban pada 9 lokasi berbeda dengan beban yang berbeda.
    Return: (list[sensor_values], list[actual_force])
    """
    print("\n=== MODE PENGUMPULAN DATA REGRESI ===")
    print(f"Akan mengumpulkan {num_points} titik data kalibrasi.")
    print("Prosedur:")
    print("  1. Awal program: tidak ada beban (F=0)")
    print("  2. Letakkan beban di lokasi & berikan masukan ke program")
    print("  3. Program akan merekam ADC value dan F_actual")
    print("  4. Ulangi di 9 lokasi berbeda dengan beban 0, 1kg, 2kg, 3kg, 5kg, dst.")
    print()

    raw_data = []
    force_data = []
    collected = 0

    while collected < num_points:
        try:
            # Baca sensor raw
            raw_values, _ = read_sensor_frame(port, request, dxl_id, length, timeout)
            
            # Aplikasikan offset
            corrected = [raw - offset for raw, offset in zip(raw_values, offsets)]
            
            # Minta input dari user
            sys.stdout.write(f"\n[{collected+1}/{num_points}] Sensor terkoreksi: {corrected}\n")
            sys.stdout.write("Masukkan berat beban AKTUAL (gram), atau 'q' untuk selesai: ")
            sys.stdout.flush()

            user_input = input().strip()
            if user_input.lower() == 'q':
                print("Selesai mengumpulkan data.")
                break

            try:
                actual_force = float(user_input)
            except ValueError:
                print("Input tidak valid, coba lagi.")
                continue

            raw_data.append(corrected)
            force_data.append(actual_force)
            collected += 1
            time.sleep(interval)

        except KeyboardInterrupt:
            print("\nPengumpulan data dibatalkan.")
            break
        except Exception as exc:
            print(f"Gagal baca sensor: {exc}")
            time.sleep(0.1)

    return raw_data, force_data


def perform_regression_calibration(
    port: serial.Serial,
    request: bytes,
    dxl_id: int,
    length: int,
    timeout: float,
    offsets: list[float],
) -> tuple[list[float], float, float]:
    """
    Lakukan regresi linear untuk mendapatkan koefisien kalibrasi.
    Return: (coefficients, r_squared, rms_error)
    """
    import numpy as np
    from numpy.linalg import lstsq

    # Kumpulkan data
    raw_data, force_data = collect_regression_data(port, request, dxl_id, length, timeout, offsets)

    if len(raw_data) < 4:
        raise ValueError("Minimal diperlukan 4 titik data untuk regresi linear")

    # Buat matrix X (sensor values) dan y (actual force)
    X = np.array(raw_data, dtype=float)
    y = np.array(force_data, dtype=float)

    print(f"\n=== HASIL REGRESI LINEAR ===")
    print(f"Jumlah data point: {len(raw_data)}")
    print(f"Sensor data shape: {X.shape}")

    # Lakukan least-squares fit tanpa intercept: y = X @ coefficients
    try:
        result = lstsq(X, y, rcond=None)
        coefficients = result[0]
        residuals = result[1]
        rank = result[2]
        singular = result[3]

        if len(residuals) > 0:
            rms_error = np.sqrt(residuals[0] / len(y))
        else:
            rms_error = 0.0

        # Hitung R-squared
        y_pred = X @ coefficients
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        r_squared = 1.0 - (ss_res / ss_tot) if ss_tot != 0 else 0.0

        print(f"\nKoefisien regresi (C1, C2, C3, C4):")
        for idx, coef in enumerate(coefficients):
            print(f"  C{idx+1} = {coef:.6f}")

        print(f"\nR-squared: {r_squared:.6f}")
        print(f"RMS Error: {rms_error:.4f} gram")
        print(f"% Full Scale (asumsi max=5000g): {(rms_error/5000)*100:.3f}%\n")

        return coefficients.tolist(), r_squared, rms_error

    except Exception as exc:
        raise RuntimeError(f"Regresi linear gagal: {exc}")


def apply_offsets(raw_values: list[int], offsets: list[float], threshold: float) -> list[float]:
    corrected = []
    for raw, offset in zip(raw_values, offsets):
        value = raw - offset
        if abs(value) < threshold:
            value = 0.0
        corrected.append(value)
    return corrected


def apply_regression_coefficients(corrected_values: list[float], coefficients: list[float]) -> float:
    """
    Aplikasikan koefisien regresi: F_total = C1*V1 + C2*V2 + C3*V3 + C4*V4
    """
    if len(corrected_values) != len(coefficients):
        raise ValueError("Jumlah sensor tidak cocok dengan jumlah koefisien")

    total_force = sum(v * c for v, c in zip(corrected_values, coefficients))
    return total_force


def normalize_force_values(corrected_values: list[float], use_absolute_force: bool) -> list[float]:
    if use_absolute_force:
        return [abs(value) for value in corrected_values]
    return [max(value, 0.0) for value in corrected_values]


def compute_direction_scores(sensor_values: list[float]) -> dict[str, float] | None:
    if len(sensor_values) != 4:
        return None

    activity = [abs(value) for value in sensor_values]
    return {
        "front": activity[0] + activity[1],
        "rear": activity[2] + activity[3],
        "left": activity[0] + activity[2],
        "right": activity[1] + activity[3],
    }


def compute_cop(
    force_values: list[float],
    sensor_layout: list[dict[str, float | str]],
) -> tuple[float, float] | None:
    total_force = sum(force_values)
    if total_force <= 0:
        return None

    x = sum(force * float(sensor["x"]) for force, sensor in zip(force_values, sensor_layout)) / total_force
    y = sum(force * float(sensor["y"]) for force, sensor in zip(force_values, sensor_layout)) / total_force
    return x, y


def infer_balance_status(
    sensor_values: list[float],
    sensor_layout: list[dict[str, float | str]],
    cop: tuple[float, float] | None,
    balance_tolerances: list[float] | None = None,
) -> tuple[str, bool]:
    """
    Infer balance status berdasarkan sensor values dan CoP.
    Jika balance_tolerances disediakan, cek apakah sensor berada dalam range tolerance terlebih dahulu.
    Return: (status_string, is_balanced)
    """
    # Jika balance tolerance tersedia, cek apakah semua sensor dalam range
    if balance_tolerances is not None and len(balance_tolerances) == len(sensor_values):
        all_within_tolerance = all(
            abs(value) <= tolerance
            for value, tolerance in zip(sensor_values, balance_tolerances)
        )
        if all_within_tolerance:
            return "Seimbang di tengah (dalam tolerance)", True

    direction_scores = compute_direction_scores(sensor_values)
    if direction_scores is not None:
        front = direction_scores["front"]
        rear = direction_scores["rear"]
        left = direction_scores["left"]
        right = direction_scores["right"]
        total_activity = front + rear

        if total_activity <= 0:
            return "Tidak ada beban yang terbaca", False

        front_back_diff = front - rear
        left_right_diff = left - right
        balance_threshold = max(total_activity * 0.05, 1.0)

        if (
            abs(front_back_diff) < balance_threshold
            and abs(left_right_diff) < balance_threshold
        ):
            return "Seimbang di tengah", True

        if abs(front_back_diff) >= abs(left_right_diff):
            if front_back_diff > 0:
                return "Condong ke DEPAN (sensor 0 dan 1 lebih berubah)", False
            return "Condong ke BELAKANG (sensor 2 dan 3 lebih berubah)", False

        if left_right_diff > 0:
            return "Condong ke KIRI (sensor 0 dan 2 lebih berubah)", False
        return "Condong ke KANAN (sensor 1 dan 3 lebih berubah)", False

    total_force = sum(sensor_values)
    if total_force <= 0:
        return "Tidak ada beban yang terbaca", False

    if cop is None:
        return "Tidak ada beban yang terbaca", False

    x_cop, y_cop = cop
    axis_threshold = 0.2

    if abs(x_cop) < axis_threshold and abs(y_cop) < axis_threshold:
        return "Seimbang di tengah", True
    if y_cop >= axis_threshold and abs(x_cop) < axis_threshold:
        return "Condong ke DEPAN", False
    if y_cop <= -axis_threshold and abs(x_cop) < axis_threshold:
        return "Condong ke BELAKANG", False
    if x_cop <= -axis_threshold and abs(y_cop) < axis_threshold:
        return "Condong ke KIRI", False
    if x_cop >= axis_threshold and abs(y_cop) < axis_threshold:
        return "Condong ke KANAN", False

    if abs(y_cop) >= abs(x_cop):
        return ("Condong ke DEPAN", False) if y_cop > 0 else ("Condong ke BELAKANG", False)
    return ("Condong ke KIRI", False) if x_cop < 0 else ("Condong ke KANAN", False)


def poll_pressed_keys() -> set[str]:
    if msvcrt is None:
        return set()

    pressed: set[str] = set()
    while msvcrt.kbhit():
        key = msvcrt.getwch()
        pressed.add(key.lower())
    return pressed


def apply_median_filter(sensor_values: list[int], rolling_windows: list, window_size: int) -> list[int]:
    """
    Aplikasikan median filter ke sensor values menggunakan rolling window.
    
    Args:
        sensor_values: List nilai sensor raw saat ini
        rolling_windows: List of deque untuk setiap sensor (diupdate in-place)
        window_size: Ukuran window untuk median filter
    
    Returns:
        List nilai sensor setelah median filter diterapkan
    """
    from collections import deque
    import statistics
    
    filtered_values = []
    
    for idx, value in enumerate(sensor_values):
        # Pastikan rolling_windows sudah initialized dengan deque
        if idx >= len(rolling_windows):
            rolling_windows.append(deque(maxlen=window_size))
        
        # Tambahkan nilai baru ke window
        rolling_windows[idx].append(value)
        
        # Hitung median dari window
        if len(rolling_windows[idx]) > 0:
            median_value = statistics.median(rolling_windows[idx])
            filtered_values.append(int(median_value))
        else:
            filtered_values.append(value)
    
    return filtered_values


def reset_rolling_windows(rolling_windows: list, sensor_count: int, window_size: int) -> None:
    """
    Clear dan reinitialize semua rolling windows.
    Dipanggil setelah setiap calibration untuk memastikan median filter mulai fresh.
    """
    from collections import deque
    rolling_windows.clear()
    for _ in range(sensor_count):
        rolling_windows.append(deque(maxlen=window_size))


def format_sensor_line(values: list[float]) -> str:
    return "[" + ", ".join(f"{value:9.2f}" for value in values) + "]"


def format_direction_scores(direction_scores: dict[str, float] | None) -> str:
    if direction_scores is None:
        return "-"

    return (
        f"DEPAN={direction_scores['front']:.2f}  "
        f"KANAN={direction_scores['right']:.2f}  "
        f"KIRI={direction_scores['left']:.2f}  "
        f"BELAKANG={direction_scores['rear']:.2f}"
    )


def run_reader(
    port_name: str,
    baudrate: int,
    dxl_id: int,
    address: int,
    length: int,
    timeout: float,
    interval: float,
    threshold: float,
    sample_count: int,
    sample_delay: float,
    calibration_path: Path,
    regression_path: Path,
    balance_tolerance_path: Path,
    calibration_key: str,
    center_reference_key: str,
    balance_tolerance_key: str,
    force_calibration: bool,
    force_regression: bool,
    use_absolute_force: bool,
    median_window: int,
) -> None:
    if length % 3 != 0:
        raise ValueError("Length pembacaan harus kelipatan 3 byte")

    request = build_read_packet(dxl_id, address, length)
    sensor_count = length // 3
    sensor_layout = get_sensor_layout(sensor_count)
    
    # Initialize rolling windows untuk median filter
    rolling_windows: list[list[int]] = [list() for _ in range(sensor_count)]

    with serial.Serial(port=port_name, baudrate=baudrate, timeout=0.01) as port:
        port.reset_input_buffer()
        port.reset_output_buffer()

        # ===== STEP 1: Load atau lakukan offset calibration =====
        offsets: list[float] | None = None
        if not force_calibration:
            try:
                offsets = load_calibration(calibration_path, sensor_count)
                print(f"✓ Offset calibration dimuat dari: {calibration_path}")
            except Exception as exc:
                print(f"⚠ Gagal load offset calibration: {exc}")

        if offsets is None or force_calibration:
            offsets = perform_offset_calibration(
                port=port,
                request=request,
                dxl_id=dxl_id,
                length=length,
                timeout=timeout,
                sample_count=sample_count,
                sample_delay=sample_delay,
            )
            save_calibration(calibration_path, offsets, sensor_layout)
            print(f"✓ Offset calibration disimpan ke: {calibration_path}")

        # ===== STEP 2: Load atau lakukan regression calibration =====
        coefficients: list[float] | None = None
        if not force_regression:
            try:
                coefficients = load_regression_coefficients(regression_path, sensor_count)
                print(f"✓ Regression coefficients dimuat dari: {regression_path}")
            except Exception as exc:
                print(f"⚠ Gagal load regression coefficients: {exc}")

        if coefficients is None or force_regression:
            print("\n>>> PERSIAPAN: Lakukan offset calibration di atas terlebih dahulu <<<")
            print(">>> Pastikan platform KOSONG, tidak ada beban <<<\n")
            try:
                coefficients, r_sq, rms = perform_regression_calibration(
                    port=port,
                    request=request,
                    dxl_id=dxl_id,
                    length=length,
                    timeout=timeout,
                    offsets=offsets,
                )
                save_regression_coefficients(regression_path, coefficients, r_sq, rms, sensor_layout)
                print(f"✓ Regression coefficients disimpan ke: {regression_path}")
            except Exception as exc:
                print(f"✗ Regression calibration gagal: {exc}")
                print("Program dilanjutkan dengan mode offset-only (tanpa regression).")
                coefficients = None

        # ===== STEP 3: Load atau capture balance tolerance =====
        balance_tolerances: list[float] | None = None
        try:
            balance_tolerances = load_balance_tolerance(balance_tolerance_path, sensor_count)
            print(f"✓ Balance tolerance dimuat dari: {balance_tolerance_path}")
        except Exception as exc:
            print(f"⚠ Gagal load balance tolerance: {exc}")

        if balance_tolerances is None:
            print("⚠ Balance tolerance belum diset. Seimbang hanya akan dideteksi melalui CoP.")
            print(f"Tekan '{balance_tolerance_key}' saat program berjalan untuk set tolerance.\n")

        # ===== STEP 4: Real-time reading dengan offset dan/atau regression =====
        center_reference_values, center_reference_cop = perform_center_reference_capture(
            port=port,
            request=request,
            dxl_id=dxl_id,
            length=length,
            timeout=timeout,
            offsets=offsets,
            threshold=threshold,
            sample_count=sample_count,
            sample_delay=sample_delay,
            coefficients=coefficients,
            use_absolute_force=use_absolute_force,
            sensor_layout=sensor_layout,
            rolling_windows=rolling_windows,
            median_window=median_window,
        )

        print("\n=== MODE PEMBACAAN REAL-TIME ===")
        print(f"Offset calibration: aktif")
        print(f"Regression calibration: {'aktif' if coefficients else 'tidak aktif (offset-only)'}")
        print(f"Balance tolerance: {'aktif' if balance_tolerances else 'tidak aktif'}")
        print(f"Median filter: aktif (window size = {median_window})")
        print(f"Tekan '{calibration_key}' untuk kalibrasi ulang offset")
        print(f"Tekan '{center_reference_key}' untuk ambil ulang referensi tengah")
        print(f"Tekan '{balance_tolerance_key}' untuk set/update balance tolerance")
        print()

        iteration = 0
        while True:
            try:
                pressed_keys = poll_pressed_keys()

                if calibration_key.lower() in pressed_keys:
                    offsets = perform_offset_calibration(
                        port=port,
                        request=request,
                        dxl_id=dxl_id,
                        length=length,
                        timeout=timeout,
                        sample_count=sample_count,
                        sample_delay=sample_delay,
                    )
                    save_calibration(calibration_path, offsets, sensor_layout)
                    print(f"✓ Offset calibration diperbarui: {calibration_path}")
                    center_reference_values, center_reference_cop = perform_center_reference_capture(
                        port=port,
                        request=request,
                        dxl_id=dxl_id,
                        length=length,
                        timeout=timeout,
                        offsets=offsets,
                        threshold=threshold,
                        sample_count=sample_count,
                        sample_delay=sample_delay,
                        coefficients=coefficients,
                        use_absolute_force=use_absolute_force,
                        sensor_layout=sensor_layout,
                        rolling_windows=rolling_windows,
                        median_window=median_window,
                    )
                elif center_reference_key.lower() in pressed_keys:
                    center_reference_values, center_reference_cop = perform_center_reference_capture(
                        port=port,
                        request=request,
                        dxl_id=dxl_id,
                        length=length,
                        timeout=timeout,
                        offsets=offsets,
                        threshold=threshold,
                        sample_count=sample_count,
                        sample_delay=sample_delay,
                        coefficients=coefficients,
                        use_absolute_force=use_absolute_force,
                        sensor_layout=sensor_layout,
                        rolling_windows=rolling_windows,
                        median_window=median_window,
                    )
                elif balance_tolerance_key.lower() in pressed_keys:
                    balance_tolerances = perform_balance_tolerance_capture(
                        port=port,
                        request=request,
                        dxl_id=dxl_id,
                        length=length,
                        timeout=timeout,
                        offsets=offsets,
                        threshold=threshold,
                        sample_count=sample_count,
                        sample_delay=sample_delay,
                    )
                    save_balance_tolerance(balance_tolerance_path, balance_tolerances, sensor_layout)
                    print(f"✓ Balance tolerance disimpan ke: {balance_tolerance_path}")

                raw_values, _response = read_sensor_frame(port, request, dxl_id, length, timeout)
                
                # Aplikasikan median filter
                raw_values = apply_median_filter(raw_values, rolling_windows, median_window)
                
                # Aplikasikan offset
                corrected_values = apply_offsets(raw_values, offsets, threshold)
                relative_values = [
                    current - reference
                    for current, reference in zip(corrected_values, center_reference_values)
                ]
                
                # Aplikasikan regression (jika ada) atau gunakan raw corrected
                if coefficients:
                    total_force = apply_regression_coefficients(corrected_values, coefficients)
                    # Untuk CoP, gunakan individual sensor forces (proportional ke regression coef)
                    sensor_forces = [v * c for v, c in zip(corrected_values, coefficients)]
                else:
                    total_force = sum(corrected_values)
                    sensor_forces = corrected_values

                force_values = normalize_force_values(sensor_forces, use_absolute_force)
                current_cop = compute_cop(force_values, sensor_layout)
                if current_cop is not None and center_reference_cop is not None:
                    cop = (
                        current_cop[0] - center_reference_cop[0],
                        current_cop[1] - center_reference_cop[1],
                    )
                else:
                    cop = current_cop

                direction_scores = compute_direction_scores(relative_values)
                status_line, is_balanced = infer_balance_status(relative_values, sensor_layout, cop, balance_tolerances)

                if cop is None:
                    cop_text = "( None,  None)"
                else:
                    cop_text = f"({cop[0]: .2f}, {cop[1]: .2f})"

                sys.stdout.write("\x1b[2J\x1b[H")
                sys.stdout.write(f"Iteration   : {iteration}\n")
                sys.stdout.write(f"Raw values  : {raw_values}\n")
                sys.stdout.write(f"Corrected   : {format_sensor_line(corrected_values)}\n")
                sys.stdout.write(f"Relative    : {format_sensor_line(relative_values)}\n")
                sys.stdout.write(f"Direction   : {format_direction_scores(direction_scores)}\n")
                if coefficients:
                    sys.stdout.write(f"Total Force : {total_force:.2f} gram\n")
                sys.stdout.write(f"CoP         : {cop_text} (relatif ke referensi tengah)\n")
                if balance_tolerances:
                    tolerance_status = "✓ DALAM TOLERANSI" if is_balanced else "✗ LUAR TOLERANSI"
                    sys.stdout.write(f"Toleransi   : {format_sensor_line(balance_tolerances)} {tolerance_status}\n")
                sys.stdout.write(f"Status      : {status_line}\n")
                sys.stdout.write(
                    f"\nPress '{calibration_key}' untuk re-calibrate offset, "
                    f"'{center_reference_key}' untuk set referensi tengah, "
                    f"'{balance_tolerance_key}' untuk set tolerance, Ctrl+C untuk exit\n"
                )
                sys.stdout.flush()

                iteration += 1
                time.sleep(interval)

            except KeyboardInterrupt:
                print("\n\nStop.")
                return
            except Exception as exc:
                print(f"Read gagal: {exc}")
                time.sleep(max(interval, 0.1))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Baca loadcell dari STM32 via Dynamixel Protocol 2.0, "
            "lakukan offset calibration dan regression calibration (seperti paper Schafer), "
            "lalu hitung Center of Pressure secara real-time."
        )
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Port serial, mis. COM5")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="Baudrate serial")
    parser.add_argument("--id", type=int, default=DEFAULT_DXL_ID, help="DYNAMIXEL ID device")
    parser.add_argument("--address", type=int, default=DEFAULT_ADDRESS, help="Alamat awal register")
    parser.add_argument("--length", type=int, default=DEFAULT_LENGTH, help="Jumlah byte yang dibaca")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Timeout baca packet")
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL, help="Jeda antar pembacaan")
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD, help="Threshold noise setelah offset")
    parser.add_argument(
        "--median-window",
        type=int,
        default=DEFAULT_MEDIAN_WINDOW,
        help="Ukuran window untuk median filter (hapus outlier/spike sensor)",
    )
    parser.add_argument(
        "--calibration-samples",
        type=int,
        default=DEFAULT_CALIBRATION_SAMPLES,
        help="Jumlah sampel saat offset calibration",
    )
    parser.add_argument(
        "--calibration-delay",
        type=float,
        default=DEFAULT_CALIBRATION_DELAY,
        help="Jeda antar sampel saat offset calibration",
    )
    parser.add_argument(
        "--calibration-file",
        type=Path,
        default=DEFAULT_CALIBRATION_FILE,
        help="Path file JSON penyimpanan offset",
    )
    parser.add_argument(
        "--regression-file",
        type=Path,
        default=DEFAULT_REGRESSION_FILE,
        help="Path file JSON penyimpanan regression coefficients",
    )
    parser.add_argument(
        "--balance-tolerance-file",
        type=Path,
        default=DEFAULT_BALANCE_TOLERANCE_FILE,
        help="Path file JSON penyimpanan balance tolerance",
    )
    parser.add_argument(
        "--calibration-key",
        default=DEFAULT_CALIBRATION_KEY,
        help="Tombol keyboard untuk kalibrasi ulang offset",
    )
    parser.add_argument(
        "--center-reference-key",
        default=DEFAULT_CENTER_REFERENCE_KEY,
        help="Tombol keyboard untuk mengambil ulang referensi tengah",
    )
    parser.add_argument(
        "--balance-tolerance-key",
        default=DEFAULT_BALANCE_TOLERANCE_KEY,
        help="Tombol keyboard untuk set/update balance tolerance",
    )
    parser.add_argument(
        "--force-calibration",
        action="store_true",
        help="Paksa offset calibration ulang saat program mulai",
    )
    parser.add_argument(
        "--force-regression",
        action="store_true",
        help="Paksa regression calibration ulang saat program mulai",
    )
    parser.add_argument(
        "--signed-force",
        action="store_true",
        help="Gunakan hanya nilai positif untuk CoP, tanpa abs()",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        # Import numpy untuk regresi
        import numpy as np
        print(f"✓ NumPy terdeteksi: {np.__version__}\n")
    except ImportError:
        print("✗ ERROR: NumPy tidak terinstall!")
        print("Install dengan: pip install numpy")
        return 1

    try:
        run_reader(
            port_name=args.port,
            baudrate=args.baudrate,
            dxl_id=args.id,
            address=args.address,
            length=args.length,
            timeout=args.timeout,
            interval=args.interval,
            threshold=args.threshold,
            sample_count=args.calibration_samples,
            sample_delay=args.calibration_delay,
            calibration_path=args.calibration_file,
            regression_path=args.regression_file,
            balance_tolerance_path=args.balance_tolerance_file,
            calibration_key=args.calibration_key,
            center_reference_key=args.center_reference_key,
            balance_tolerance_key=args.balance_tolerance_key,
            force_calibration=args.force_calibration,
            force_regression=args.force_regression,
            use_absolute_force=not args.signed_force,
            median_window=args.median_window,
        )
        return 0
    except serial.SerialException as exc:
        print(f"Gagal membuka serial port: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Program gagal dijalankan: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

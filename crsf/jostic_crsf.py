import pygame
import time
import serial
import struct

# Инициализация pygame и джойстика
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Настройка UART
try:
    ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
    time.sleep(2)
    print(f"UART подключен: {ser.port}")                                                   
except Exception as e:
    print(f"Ошибка UART: {e}")
    exit()

# CRSF протокол константы
CRSF_SYNC_BYTE = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16

def map_joystick_to_rc(axis_value):
    """Конвертирует значение джойстика (-1.0 до 1.0) в RC значение (172-1811)"""
    return int(992 + (axis_value * 819))

def calculate_crsf_crc8(data):
    """Расчет CRC8 для CRSF протокола"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
            crc &= 0xFF
    return crc

def pack_channels(channels):
    """Упаковка 16 каналов в 22 байта (11 бит на канал)"""
    packed = bytearray(22)
    
    for i in range(16):
        # Нормализуем значение канала
        channel_val = max(172, min(1811, channels[i])) - 172
        
        # Позиция в упакованном массиве
        bit_pos = i * 11
        byte_pos = bit_pos // 8
        bit_offset = bit_pos % 8
        
        # Упаковываем 11 бит
        remaining_bits = 11
        while remaining_bits > 0:
            bits_in_byte = min(8 - bit_offset, remaining_bits)
            
            # Извлекаем биты
            mask = ((1 << bits_in_byte) - 1) << (remaining_bits - bits_in_byte)
            bits = (channel_val & mask) >> (remaining_bits - bits_in_byte)
            
            # Записываем биты
            packed[byte_pos] |= bits << (8 - bit_offset - bits_in_byte)
            
            remaining_bits -= bits_in_byte
            bit_offset += bits_in_byte
            
            if bit_offset >= 8:
                bit_offset = 0
                byte_pos += 1
    
    return packed

def send_rc_channels(axes, buttons):
    """Отправка RC каналов в формате CRSF"""
    channels = [992] * 16  # Нейтральные значения
    
    # Каналы 1-6: оси джойстика
    for i in range(min(6, len(axes))):
        channels[i] = map_joystick_to_rc(axes[i])
    
    # Каналы 7-12: кнопки
    for i in range(min(6, len(buttons))):
        channels[6 + i] = 1800 if buttons[i] else 172
    
    # Упаковываем каналы
    channels_data = pack_channels(channels)
    
    # Собираем пакет: sync(1) + len(1) + type(1) + data(22) + crc(1) = 26 байт
    packet = bytearray()
    packet.append(CRSF_SYNC_BYTE)  # Sync byte
    packet.append(24)              # Length: type(1) + data(22) + crc(1) = 24
    packet.append(CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    packet.extend(channels_data)   # 22 байта данных каналов
    
    # CRC рассчитывается для данных после length байта
    crc_data = packet[2:]  # type + channels_data
    crc = calculate_crsf_crc8(crc_data)
    packet.append(crc)
    
    # Отправка пакета
    try:
        ser.write(packet)
        print(f"CRSF отправлен, длина: {len(packet)} байт")
        print(f"Каналы: 1:{channels[0]}, 2:{channels[1]}, 3:{channels[2]}, 4:{channels[3]}")
    except Exception as e:
        print(f"Ошибка отправки: {e}")

def read_joystick():
    """Чтение данных джойстика"""
    pygame.event.pump()
    
    axes = []
    for i in range(joystick.get_numaxes()):
        axes.append(joystick.get_axis(i))
    
    buttons = []
    for i in range(joystick.get_numbuttons()):
        buttons.append(joystick.get_button(i))
    
    return axes, buttons

# Основной цикл
try:
    print("Начинаем передачу данных джойстика по протоколу CRSF...")
    
    while True:
        axes, buttons = read_joystick()
        send_rc_channels(axes, buttons)
        time.sleep(0.02)  # 50Hz

except KeyboardInterrupt:
    print("\nЗавершение работы...")
    ser.close()
    pygame.quit()
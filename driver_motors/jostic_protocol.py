import pygame
import time
import serial
import struct

# Инициализация pygame и джойстика
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Настройка UART (замените порт на ваш)
"""
try:
    ser = serial.Serial('/dev/ttyACM0, 115200, timeout=1')
    time.sleep(2)  # Ожидание инициализации UART
except Exception as e:
    print(f"Ошибка UART: {e}")
    exit()
"""
# Словари для осей и кнопок
AXIS_NAMES = {0: "LX", 1: "LY", 2: "RX", 3: "RY", 4: "LT", 5: "RT"}
BUTTON_NAMES = {0: "A", 1: "B", 3: "X", 4: "Y", 5: "RB", 6: "LB", 7: "RT", 8: "LT", 10: "SELECT", 11: "START"}

# Предыдущие значения для обнаружения изменений
prev_axes = [0] * joystick.get_numaxes()
prev_buttons = [0] * joystick.get_numbuttons()

def calculate_checksum(data):
    """Расчет контрольной суммы XOR"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def send_axis_data(axis_id, value):
    """Отправка данных оси"""
    # Конвертируем float (-1.0 до 1.0) в int16 (-32768 до 32767)
    int_value = int(value * 32767)
    
    # Создаем пакет: [старт][тип=0][id_оси][значение_2байта][checksum][стоп]
    packet = bytearray()
    packet.append(0xAA)  # Старт
    packet.append(0x00)  # Тип: ось
    packet.append(axis_id)  # ID оси
    packet.extend(struct.pack('>h', int_value))  # Значение как 2 байта big-endian
    
    # Контрольная сумма и стоп
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    packet.append(0x55)  # Стоп

   
def send_button_data(button_id, state):
    """Отправка данных кнопки"""
    packet = bytearray()
    packet.append(0xAA)  # Старт
    packet.append(0x01)  # Тип: кнопка
    packet.append(button_id)  # ID кнопки
    packet.append(0x01 if state else 0x00)  # Состояние
    
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    packet.append(0x55)  # Стоп
    print(packet)
    
def send_emergency_stop():
    """Экстренная остановка"""
    packet = bytearray([0xAA, 0x02, 0x00, 0x00, 0x55])
    packet[3] = calculate_checksum(packet[:3])  # Контрольная сумма
    
    """
    try:
        ser.write(packet)
        print("Экстренная остановка отправлена!")
    except Exception as e:
        print(f"Ошибка отправки STOP: {e}")
    """
def read_joystick():
    """Чтение данных джойстика"""
    pygame.event.pump()
    
    # Читаем оси
    axes = []
    for i in range(joystick.get_numaxes()):
        axis_value = joystick.get_axis(i)
        axes.append(round(axis_value, 2))
    
    # Читаем кнопки
    buttons = []
    for i in range(joystick.get_numbuttons()):
        button_state = joystick.get_button(i)
        buttons.append(button_state)
    
    return axes, buttons

# Основной цикл
try:
    print("Начинаем передачу данных джойстика...")
    
    while True:
        axes, buttons = read_joystick()
        
        # Проверяем изменения осей (только значительные)
        for i in range(min(len(axes), len(prev_axes))):
            if abs(axes[i] - prev_axes[i]) > 0.1:  # Порог чувствительности
                send_axis_data(i, axes[i])
                print(f"Ось {AXIS_NAMES.get(i, i)}: {axes[i]}")
        
        # Проверяем изменения кнопок
        for i in range(min(len(buttons), len(prev_buttons))):
            if buttons[i] != prev_buttons[i]:
                send_button_data(i, buttons[i])
                print(f"Кнопка {BUTTON_NAMES.get(i, i)}: {'НАЖАТА' if buttons[i] else 'ОТПУЩЕНА'}")
        
        # Проверяем кнопку экстренной остановки (например, SELECT)
        if buttons[10] == 1 and prev_buttons[10] == 0:
            send_emergency_stop()
        
        # Обновляем предыдущие значения
        prev_axes = axes.copy()
        prev_buttons = buttons.copy()
        
        time.sleep(0.02)  # 50Hz update rate

except KeyboardInterrupt:
    print("\nЗавершение работы...")
    send_emergency_stop()
    #ser.close()
    pygame.quit()
from machine import Pin, PWM, UART
import time
import struct

# Пины для управления моторами
FwdPin_A = 10
BwdPin_A = 9
FwdPin_B = 6
BwdPin_B = 5

# Инициализация ШИМ для моторов
fwd_a = PWM(Pin(FwdPin_A))
bwd_a = PWM(Pin(BwdPin_A))
fwd_b = PWM(Pin(FwdPin_B))
bwd_b = PWM(Pin(BwdPin_B))

# Настройка частоты ШИМ
fwd_a.freq(1000)
bwd_a.freq(1000)
fwd_b.freq(1000)
bwd_b.freq(1000)

# Настройка UART
uart = UART(1, baudrate=115200, tx=17, rx=16)  # Настройте пины под вашу плату

# Текущее состояние осей и кнопок
current_axes = [0.0] * 6  # LX, LY, RX, RY, LT, RT
current_buttons = [0] * 12

# Переменные для парсинга
packet_buffer = bytearray()
packet_started = False

def stop_all():
    """Остановить все моторы"""
    fwd_a.duty_u16(0)
    bwd_a.duty_u16(0)
    fwd_b.duty_u16(0)
    bwd_b.duty_u16(0)

def calculate_checksum(data):
    """Расчет контрольной суммы XOR"""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def parse_packet(packet):
    """Разбор принятого пакета"""
    if len(packet) < 5:
        return False
    
    # Проверяем стартовый и стоповый байты
    if packet[0] != 0xAA or packet[-1] != 0x55:
        return False
    
    # Проверяем контрольную сумму
    data = packet[:-2]  # Все кроме контрольной суммы и стопа
    received_checksum = packet[-2]
    calculated_checksum = calculate_checksum(data)
    
    if received_checksum != calculated_checksum:
        return False
    
    # Разбираем пакет по типу
    packet_type = packet[1]
    
    if packet_type == 0x00:  # Данные оси
        axis_id = packet[2]
        axis_value = struct.unpack('>h', packet[3:5])[0] / 32767.0  # Конвертируем обратно в float
        current_axes[axis_id] = axis_value
        print(f"Ось {axis_id}: {axis_value:.2f}")
        return True
        
    elif packet_type == 0x01:  # Данные кнопки
        button_id = packet[2]
        button_state = packet[3]
        current_buttons[button_id] = button_state
        print(f"Кнопка {button_id}: {button_state}")
        return True
        
    elif packet_type == 0x02:  # Экстренная остановка
        print("ЭКСТРЕННАЯ ОСТАНОВКА!")
        stop_all()
        return True
    
    return False

def control_motors():
    """Управление моторами на основе данных джойстика"""
    # Левый стик (оси 0 и 1) для движения вперед/назад и поворотов
    left_y = current_axes[1]  # LY: вперед/назад
    left_x = current_axes[0]  # LX: повороты
    
    # Правый стик (оси 2 и 3) для тонкого управления
    right_x = current_axes[2]  # RX: дополнительные повороты
    
    # Триггеры (оси 4 и 5) для скорости
    left_trigger = (current_axes[4] + 1) / 2  # LT: 0-1
    right_trigger = (current_axes[5] + 1) / 2  # RT: 0-1
    
    # Базовая скорость (масштабируем 0-100 в 0-65535)
    base_speed = int(max(left_trigger, right_trigger) * 65535)
    
    # Движение вперед/назад на основе левого стика
    if abs(left_y) > 0.1:
        speed = int(abs(left_y) * base_speed)
        if left_y > 0:  # Вперед
            fwd_a.duty_u16(speed)
            bwd_a.duty_u16(0)
            fwd_b.duty_u16(speed)
            bwd_b.duty_u16(0)
        else:  # Назад
            fwd_a.duty_u16(0)
            bwd_a.duty_u16(speed)
            fwd_b.duty_u16(0)
            bwd_b.duty_u16(speed)
    else:
        # Повороты на месте на основе левого стика
        if abs(left_x) > 0.1:
            speed = int(abs(left_x) * base_speed)
            if left_x > 0:  # Поворот направо
                fwd_a.duty_u16(0)
                bwd_a.duty_u16(speed)
                fwd_b.duty_u16(speed)
                bwd_b.duty_u16(0)
            else:  # Поворот налево
                fwd_a.duty_u16(speed)
                bwd_a.duty_u16(0)
                fwd_b.duty_u16(0)
                bwd_b.duty_u16(speed)
        else:
            # Тонкие повороты на основе правого стика
            if abs(right_x) > 0.1:
                speed = int(abs(right_x) * base_speed // 2)
                if right_x > 0:  # Плавный поворот направо
                    fwd_a.duty_u16(speed // 2)
                    bwd_a.duty_u16(0)
                    fwd_b.duty_u16(speed)
                    bwd_b.duty_u16(0)
                else:  # Плавный поворот налево
                    fwd_a.duty_u16(speed)
                    bwd_a.duty_u16(0)
                    fwd_b.duty_u16(speed // 2)
                    bwd_b.duty_u16(0)
            else:
                # Остановка если нет ввода
                stop_all()

def uart_receiver():
    """Прием данных по UART"""
    global packet_buffer, packet_started
    
    if uart.any():
        data = uart.read(1)
        if data:
            byte = data[0]
            
            if byte == 0xAA:  # Начало пакета
                packet_buffer = bytearray([byte])
                packet_started = True
                
            elif packet_started and byte == 0x55:  # Конец пакета
                packet_buffer.append(byte)
                if parse_packet(packet_buffer):
                    control_motors()  # Обновляем управление моторами
                packet_started = False
                packet_buffer = bytearray()
                
            elif packet_started:
                packet_buffer.append(byte)
                # Защита от слишком длинных пакетов
                if len(packet_buffer) > 20:
                    packet_started = False
                    packet_buffer = bytearray()

# Основной цикл ESP32
print("ESP32 готов к приему данных джойстика...")
stop_all()

try:
    while True:
        uart_receiver()
        time.sleep(0.01)  # 100Hz update rate

except KeyboardInterrupt:
    stop_all()
    print("Программа завершена")

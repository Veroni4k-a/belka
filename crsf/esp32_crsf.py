from machine import Pin, PWM, UART
import time

# Пины для управления моторами
FwdPin_A = 0  # Левый мотор вперед
BwdPin_A = 1  # Левый мотор назад
FwdPin_B = 7  # Правый мотор вперед  
BwdPin_B = 6  # Правый мотор назад

MaxSpd = 100  # Максимальная скорость в %

# Создаем объекты PWM для управления скоростью
fwd_a = PWM(Pin(FwdPin_A))
bwd_a = PWM(Pin(BwdPin_A))
fwd_b = PWM(Pin(FwdPin_B))
bwd_b = PWM(Pin(BwdPin_B))

# Настраиваем частоту ШИМ
fwd_a.freq(1000)
bwd_a.freq(1000)
fwd_b.freq(1000)
bwd_b.freq(1000)

# Настройка UART для приема данных CRSF
uart = UART(1, baudrate=115200, rx=20, tx=21)

# CRSF протокол константы
CRSF_SYNC_BYTE = 0xC8

# Значения каналов CRSF (172-1811, центр 992)
channel1 = 992  # Канал 1: левый/правый борт
channel2 = 992  # Канал 2: вперед/назад

def stop_all():
    """Остановить все моторы"""
    fwd_a.duty_u16(0)
    bwd_a.duty_u16(0)
    fwd_b.duty_u16(0)
    bwd_b.duty_u16(0)

def set_motor_speed(motor_fwd, motor_bwd, speed):
    """Установить скорость мотора (-100 до 100)"""
    speed = max(-100, min(100, speed))
    
    if speed > 0:
        # Вперед
        pwm_value = int(speed * 65535 // 100)
        motor_fwd.duty_u16(pwm_value)
        motor_bwd.duty_u16(0)
    elif speed < 0:
        # Назад
        pwm_value = int(-speed * 65535 // 100)
        motor_fwd.duty_u16(0)
        motor_bwd.duty_u16(pwm_value)
    else:
        # Стоп
        motor_fwd.duty_u16(0)
        motor_bwd.duty_u16(0)

def unpack_crsf_channels(data):
    """Простая распаковка каналов из CRSF пакета"""
    # CRSF пакет: [0xC8][len=24][type=0x16][22 байт данных][CRC]
    if len(data) < 26 or data[0] != CRSF_SYNC_BYTE or data[1] != 24 or data[2] != 0x16:
        return None
    
    # Берем только первые 2 канала для простоты
    channels_data = data[3:25]  # 22 байта данных каналов
    
    # Простая распаковка первых двух 11-битных значений
    # Первый канал: биты 0-10
    ch1 = ((channels_data[0] << 3) | (channels_data[1] >> 5)) & 0x7FF
    # Второй канал: биты 11-21  
    ch2 = (((channels_data[1] & 0x1F) << 6) | (channels_data[2] >> 2)) & 0x7FF
    
    return [ch1 + 172, ch2 + 172]  # Преобразуем в абсолютные значения

def read_crsf():
    """Чтение CRSF пакетов из UART"""
    global channel1, channel2
    
    if uart.any():
        data = uart.read()
        if data and len(data) >= 26:
            # Ищем CRSF пакет
            for i in range(len(data) - 25):
                if data[i] == CRSF_SYNC_BYTE:
                    packet = data[i:i+26]
                    channels = unpack_crsf_channels(packet)
                    if channels:
                        channel1, channel2 = channels[0], channels[1]
                        print(f"Каналы: 1={channel1}, 2={channel2}")
                        break

def process_motors():
    """Управление моторами на основе CRSF каналов"""
    # Канал 1: 172-1811 (центр 992)
    # Меньше центра - левый борт, больше центра - правый борт
    turn_value = (channel1 - 992) / 819.0  # -1.0 до +1.0
    
    # Канал 2: 172-1811 (центр 992)  
    # Больше центра - вперед, меньше центра - назад
    forward_value = (channel2 - 992) / 819.0  # -1.0 до +1.0
    
    # Преобразуем в скорости
    turn_speed = turn_value * MaxSpd
    forward_speed = forward_value * MaxSpd
    
    # Логика управления как в оригинальном коде:
    # Если turn_speed отрицательный - двигаем левый борт
    # Если turn_speed положительный - двигаем правый борт
    # forward_speed определяет направление и скорость
    
    if turn_speed < -10:  # Поворот влево (левый борт двигается)
        left_speed = forward_speed
        right_speed = 0
    elif turn_speed > 10:  # Поворот вправо (правый борт двигается)
        left_speed = 0
        right_speed = forward_speed
    else:  # Прямо (оба мотора)
        left_speed = forward_speed
        right_speed = forward_speed
    
    # Установка скоростей моторов
    set_motor_speed(fwd_a, bwd_a, left_speed)   # Левый мотор
    set_motor_speed(fwd_b, bwd_b, right_speed)  # Правый мотор
    
    # Отладочный вывод при движении
    if abs(forward_speed) > 5 or abs(turn_speed) > 5:
        print(f"Управление: вперед={forward_speed:.1f}%, поворот={turn_speed:.1f}%")
        print(f"Моторы: левый={left_speed:.1f}%, правый={right_speed:.1f}%")

# Основной цикл
print("ESP32 готов к приему CRSF...")
print("Канал 1: левый/правый борт")
print("Канал 2: вперед/назад")
stop_all()

try:
    while True:
        # Чтение данных CRSF
        read_crsf()
        
        # Управление моторами
        process_motors()
        
        # Небольшая задержка
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Остановка...")
    stop_all()
except Exception as e:
    print(f"Ошибка: {e}")
    stop_all()
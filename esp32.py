from machine import Pin, PWM
import time

# Пины для управления моторами
FwdPin_A = 10
BwdPin_A = 9
FwdPin_B = 6
BwdPin_B = 5

MaxSpd = 100  # Скорость, значение 0-100 (в MicroPython обычно 0-65535)

# Создаем объекты PWM для управления скоростью
fwd_a = PWM(Pin(FwdPin_A))
bwd_a = PWM(Pin(BwdPin_A))
fwd_b = PWM(Pin(FwdPin_B))
bwd_b = PWM(Pin(BwdPin_B))

# Настраиваем частоту ШИМ (опционально)
fwd_a.freq(1000)
bwd_a.freq(1000)
fwd_b.freq(1000)
bwd_b.freq(1000)

def setup():
    # В MicroPython пины настраиваются автоматически при создании PWM
    pass

def stop_all():
    """Остановить все моторы"""
    fwd_a.duty_u16(0)
    bwd_a.duty_u16(0)
    fwd_b.duty_u16(0)
    bwd_b.duty_u16(0)

def loop():
    while True:
        # Движение двигателем A вперед
        bwd_a.duty_u16(0)
        fwd_a.duty_u16(MaxSpd * 65535 // 100)  # Конвертируем 0-100 в 0-65535
        time.sleep(5)
        fwd_a.duty_u16(0)
        
        # Движение двигателем A назад
        bwd_a.duty_u16(MaxSpd * 65535 // 100)
        fwd_a.duty_u16(0)
        time.sleep(5)
        bwd_a.duty_u16(0)
        
        # Движение двигателем B вперед
        bwd_b.duty_u16(0)
        fwd_b.duty_u16(MaxSpd * 65535 // 100)
        time.sleep(5)
        fwd_b.duty_u16(0)
        
        # Движение двигателем B назад
        bwd_b.duty_u16(MaxSpd * 65535 // 100)
        fwd_b.duty_u16(0)
        time.sleep(5)
        bwd_b.duty_u16(0)

# Альтернативный вариант с использованием диапазона 0-65535
def loop_alternative():
    MaxSpd_16 = 30000  # Пример значения для диапазона 0-65535
    
    while True:
        # Движение двигателем A вперед
        bwd_a.duty_u16(0)
        fwd_a.duty_u16(MaxSpd_16)
        time.sleep(5)
        stop_all()
        time.sleep(0.1)
        
        # Движение двигателем A назад
        bwd_a.duty_u16(MaxSpd_16)
        fwd_a.duty_u16(0)
        time.sleep(5)
        stop_all()
        time.sleep(0.1)
        
        # Движение двигателем B вперед
        bwd_b.duty_u16(0)
        fwd_b.duty_u16(MaxSpd_16)
        time.sleep(5)
        stop_all()
        time.sleep(0.1)
        
        # Движение двигателем B назад
        bwd_b.duty_u16(MaxSpd_16)
        fwd_b.duty_u16(0)
        time.sleep(5)
        stop_all()
        time.sleep(0.1)

# Запуск программы
if __name__ == "__main__":
    setup()
    loop()  # или loop_alternative()

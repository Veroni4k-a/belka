import pygame
import time

# Инициализация pygame
pygame.init()

# Проверяем количество доступных джойстиков
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("Джойстик не найден! Подключите джойстик и перезапустите программу.")
    exit()

# Инициализируем первый джойстик
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Джойстик подключен: {joystick.get_name()}")
print(f"Количество осей: {joystick.get_numaxes()}")
print(f"Количество кнопок: {joystick.get_numbuttons()}")
print(f"Количество шапочек: {joystick.get_numhats()}")

# Словари с названиями для разных типов джойстиков
# Стандартные названия для Xbox-совместимых контроллеров
BUTTON_NAMES = {
    0: "A",
    1: "B",
    3: "X",
    4: "Y",     # Left Bumper
    5: "RB",     # Right Bumper
    6: "1L",   # Select
    7: "1R",
    8: "2L", # Left Stick Press
    9: "2R",  # Right Stick Press
    10:"SELECT",
    11:"START"
}

AXIS_NAMES = {
    0: "LEFT_X",
    1: "LEFT_Y", 
    2: "RIGHT_X",
    3: "RIGHT_Y",
    4: "LT",     # Left Trigger
    5: "RT"      # Right Trigger
}

HAT_NAMES = {
    0: "DPAD"
}

# Для отслеживания предыдущего состояния
prev_buttons = []
prev_axes = []
prev_hats = []

def read_joystick():
    """Чтение положения джойстика"""
    pygame.event.pump()  # Обновляем события
    
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
    
    # Читаем шапочки (D-pad)
    hats = []
    for i in range(joystick.get_numhats()):
        hat_value = joystick.get_hat(i)
        hats.append(hat_value)
    
    return axes, buttons, hats

def detect_button_changes(current_buttons, prev_buttons):
    """Обнаруживает изменения состояния кнопок"""
    pressed = []
    released = []
    
    for i in range(len(current_buttons)):
        if i >= len(prev_buttons):
            continue
            
        if current_buttons[i] != prev_buttons[i]:
            if current_buttons[i] == 1:
                pressed.append(i)
            else:
                released.append(i)
    
    return pressed, released

def detect_axis_changes(current_axes, prev_axes):
    """Обнаруживает значительные изменения осей"""
    changes = []
    
    for i in range(len(current_axes)):
        if i >= len(prev_axes):
            continue
            
        # Сравниваем с округлением до 1 десятичного знака для уменьшения шума
        current_rounded = round(current_axes[i], 1)
        prev_rounded = round(prev_axes[i], 1)
        
        if current_rounded != prev_rounded:
            changes.append((i, current_axes[i]))
    
    return changes

def detect_hat_changes(current_hats, prev_hats):
    """Обнаруживает изменения D-pad"""
    changes = []
    
    for i in range(len(current_hats)):
        if i >= len(prev_hats):
            continue
            
        if current_hats[i] != prev_hats[i]:
            changes.append((i, current_hats[i]))
    
    return changes

def get_button_name(button_index):
    """Получить название кнопки по индексу"""
    return BUTTON_NAMES.get(button_index, f"BTN_{button_index}")

def get_axis_name(axis_index):
    """Получить название оси по индексу"""
    return AXIS_NAMES.get(axis_index, f"AXIS_{axis_index}")

def get_hat_name(hat_index):
    """Получить название hat по индексу"""
    return HAT_NAMES.get(hat_index, f"HAT_{hat_index}")

def get_dpad_direction(hat_value):
    """Преобразует значение D-pad в текстовое описание"""
    x, y = hat_value
    directions = []
    
    if y == 1:
        directions.append("UP")
    elif y == -1:
        directions.append("DOWN")
    
    if x == 1:
        directions.append("RIGHT")
    elif x == -1:
        directions.append("LEFT")
    
    if not directions:
        return "CENTER"
    
    return "+".join(directions)

def print_detailed_state(axes, buttons, hats, pressed_buttons, released_buttons, axis_changes, hat_changes):
    """Вывод детального состояния джойстика"""
    output = []
    
    # Изменения осей
    if axis_changes:
        axis_output = []
        for axis_index, value in axis_changes:
            axis_name = get_axis_name(axis_index)
            axis_output.append(f"{axis_name}: {value}")
        output.append(f"Оси: {', '.join(axis_output)}")
    
    # Нажатые кнопки
    pressed_names = [get_button_name(btn) for btn in pressed_buttons]
    if pressed_names:
        output.append(f"НАЖАТЫ: {', '.join(pressed_names)}")
    
    # Отпущенные кнопки
    released_names = [get_button_name(btn) for btn in released_buttons]
    if released_names:
        output.append(f"ОТПУЩЕНЫ: {', '.join(released_names)}")
    
    # Активные кнопки
    active_buttons = []
    for i, state in enumerate(buttons):
        if state == 1:
            active_buttons.append(get_button_name(i))
    
    if active_buttons:
        output.append(f"УДЕРЖИВАЮТСЯ: {', '.join(active_buttons)}")
    
    # Изменения D-pad
    for hat_index, hat_value in hat_changes:
        hat_name = get_hat_name(hat_index)
        direction = get_dpad_direction(hat_value)
        output.append(f"{hat_name}: {direction}")
    
    # Очищаем строку и выводим
    print("\r" + " " * 150, end="")
    if output:
        print(f"\r{' | '.join(output)}", end="")
    else:
        print(f"\rДжойстик в покое", end="")

def print_complete_debug(axes, buttons, hats):
    """Функция для отладки - показывает все данные"""
    print("\n=== ДЕБАГ ИНФОРМАЦИЯ ===")
    print("Оси:", [f"{get_axis_name(i)}: {v}" for i, v in enumerate(axes)])
    print("Кнопки:", [f"{get_button_name(i)}: {v}" for i, v in enumerate(buttons)])
    print("D-pad:", [f"{get_hat_name(i)}: {v}" for i, v in enumerate(hats)])
    print("=====================\n")

# Основной цикл
try:
    # Инициализируем предыдущие состояния
    axes, prev_buttons, prev_hats = read_joystick()
    prev_axes = axes.copy()
    
    # Выводим отладочную информацию при старте
    print_complete_debug(axes, prev_buttons, prev_hats)
    print("Двигайте стиками и нажимайте кнопки для тестирования...")
    
    while True:
        axes, buttons, hats = read_joystick()
        
        # Обнаруживаем изменения
        pressed_buttons, released_buttons = detect_button_changes(buttons, prev_buttons)
        axis_changes = detect_axis_changes(axes, prev_axes)
        hat_changes = detect_hat_changes(hats, prev_hats)
        
        # Выводим информацию только если есть изменения
        if pressed_buttons or released_buttons or axis_changes or hat_changes:
            print_detailed_state(axes, buttons, hats, pressed_buttons, released_buttons, axis_changes, hat_changes)
        
        # Обновляем предыдущие состояния
        prev_buttons = buttons.copy()
        prev_axes = axes.copy()
        prev_hats = hats.copy()
        
        time.sleep(0.05)  

except KeyboardInterrupt:
    print("\n\nПрограмма завершена")
    joystick.quit()
    pygame.quit()
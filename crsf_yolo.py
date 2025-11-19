#!/usr/bin/env python3
import cv2
import numpy as np
import time
import json
import os
import serial
import struct
import math
import threading
from collections import deque
from ultralytics import YOLO
import logging

# Создаем папку для логов если не существует
os.makedirs('logs', exist_ok=True)

# Настройка логирования
logging.basicConfig(
    filename='logs/detections.log', 
    level=logging.INFO, 
    format='%(asctime)s - %(message)s'
)

class CRSFController:
    def __init__(self, uart_port='/dev/ttyACM0', baud_rate=420000, frame_size=320, max_deflection_us=300):
        # Параметры
        self.uart_port = uart_port
        self.baud_rate = baud_rate
        self.frame_size = frame_size
        self.max_deflection_us = max_deflection_us
        
        # CRSF параметры
        self.CENTER_TICKS = 992
        self.MIN_TICKS = 172
        self.MAX_TICKS = 1811
        self.MAX_OFFSET_PX = self.frame_size // 2
        self.MAX_DEFLECTION_TICKS = int(self.max_deflection_us * 8 / 5)
        
        # Текущие отклонения
        self.offset_x = 0
        self.offset_y = 0
        self.obstacles_data = []
        
        # Режимы работы
        self.auto_mode = True  # По умолчанию авторежим включен
        self.running = True
        
        # Инициализация UART
        self.uart = self.initialize_uart()
        
        # Поток для отправки пакетов
        self.packet_thread = threading.Thread(target=self.send_packets_loop)
        self.packet_thread.daemon = True
        
        # CRC таблица
        self.crc8tab = [
            0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
            0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
            0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
            0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
            0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
            0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
            0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
            0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
            0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
            0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
            0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
            0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
            0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
            0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
            0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
            0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
        ]
        
        print('🚀 CRSF Controller запущен')
        print(f'UART: {self.uart_port}, Baud: {self.baud_rate}')
        
    def initialize_uart(self):
        """Инициализация UART соединения"""
        try:
            uart = serial.Serial(self.uart_port, self.baud_rate, timeout=1)
            print(f'✅ UART {self.uart_port} успешно инициализирован')
            return uart
        except Exception as e:
            print(f'❌ Ошибка инициализации UART: {e}')
            return None
    
    def crc8(self, data):
        """Вычисление CRC8 для данных"""
        crc = 0
        for byte in data:
            crc = self.crc8tab[crc ^ byte]
        return crc
    
    def pack_channels(self, channel_data):
        """Упаковка 16 каналов в 22 байта CRSF формата"""
        channel_data = list(reversed(channel_data))
        pack_bit = []
        for idx, channel in enumerate(channel_data):
            pack_bit[idx*11: (idx+1)*11] = "{0:011b}".format(channel)
        pack_bit = ''.join(pack_bit)
        pack_byte = []
        for idx in range(22):
            current_byte = int(pack_bit[idx*8:(idx+1)*8], 2)
            pack_byte.append(current_byte)
        pack_byte = list(reversed(pack_byte))
        return pack_byte
    
    def scale_offset_to_ticks(self, offset_px):
        """Масштабирование отклонения в пикселях в тики CRSF"""
        return int(offset_px * self.MAX_DEFLECTION_TICKS / self.MAX_OFFSET_PX)
    
    def choose_avoidance_direction(self, obstacles, frame_width):
        """Выбор направления для обхода препятствий"""
        center_threshold = 25
        
        center_obstacles = []
        left_obstacles = []
        right_obstacles = []
        
        for obs in obstacles:
            if abs(obs['x']) < center_threshold:
                center_obstacles.append(obs)
            elif obs['x'] < -center_threshold:
                left_obstacles.append(obs)
            else:
                right_obstacles.append(obs)
        
        if center_obstacles:
            left_occupied = any(obs['area'] > 2000 for obs in left_obstacles)
            right_occupied = any(obs['area'] > 2000 for obs in right_obstacles)
            
            if not left_occupied and right_occupied:
                return -60  # Двигаться влево
            elif left_occupied and not right_occupied:
                return 60   # Двигаться вправо
            elif not left_occupied and not right_occupied:
                left_count = len(left_obstacles)
                right_count = len(right_obstacles)
                return -60 if left_count < right_count else 60
            else:
                return 40 if len(left_obstacles) > len(right_obstacles) else -40
        
        return 0
    
    def update_offsets(self, offset_x, offset_y):
        """Обновление отклонений от YOLO"""
        self.offset_x = offset_x
        self.offset_y = offset_y
        
        # Ограничение значений
        self.offset_x = max(-self.MAX_OFFSET_PX, min(self.offset_x, self.MAX_OFFSET_PX))
        self.offset_y = max(-self.MAX_OFFSET_PX, min(self.offset_y, self.MAX_OFFSET_PX))
        
        print(f'Offset: X={self.offset_x:.1f}, Y={self.offset_y:.1f}')
    
    def update_obstacles(self, obstacles_data):
        """Обновление данных о препятствиях"""
        self.obstacles_data = obstacles_data
        if obstacles_data:
            print(f'Obstacles: {len(self.obstacles_data)}')
    
    def create_default_channels(self):
        """Создание каналов по умолчанию (нейтральное положение)"""
        channels = [self.CENTER_TICKS] * 16
        
        # Каналы по умолчанию:
        # 0: Roll - центр
        # 1: Pitch - центр  
        # 2: Throttle - минимум (для безопасности)
        # 3: Yaw - центр
        # 4-15: Резерв/дополнительные функции
        
        channels[2] = self.MIN_TICKS + 100  # Небольшой газ для движения
        channels[11] = 1811  # Авторежим включен
        
        return channels
    
    def calculate_auto_channels(self):
        """Расчет каналов для автоматического режима"""
        channels = self.create_default_channels()
        
        if not self.auto_mode:
            return channels
        
        # Управление Roll и Pitch на основе отклонений YOLO
        roll_ticks = self.scale_offset_to_ticks(self.offset_x)
        pitch_ticks = self.scale_offset_to_ticks(self.offset_y)
        
        channels[0] = max(self.MIN_TICKS, min(self.MAX_TICKS, self.CENTER_TICKS + roll_ticks))
        channels[1] = max(self.MIN_TICKS, min(self.MAX_TICKS, self.CENTER_TICKS + pitch_ticks))
        
        # Управление обходом препятствий
        avoidance_offset = self.choose_avoidance_direction(self.obstacles_data, self.frame_size)
        if avoidance_offset != 0:
            avoidance_ticks = self.scale_offset_to_ticks(avoidance_offset)
            channels[0] = max(self.MIN_TICKS, min(self.MAX_TICKS, channels[0] + avoidance_ticks))
            print(f'🚨 Обход препятствия: {avoidance_offset}px')
        
        # Throttle управление - небольшой газ для движения вперед
        channels[2] = self.CENTER_TICKS - 100  # Постоянное движение вперед
        
        return channels
    
    def send_crsf_packet(self):
        """Отправка CRSF пакета через UART"""
        if self.uart is None or not self.uart.is_open:
            return False
        
        try:
            # Создаем каналы в зависимости от режима
            if self.auto_mode:
                channels = self.calculate_auto_channels()
            else:
                channels = self.create_default_channels()
            
            # Формируем CRSF пакет
            packet_type = 0x16  # RC Channels packet
            packet_length = 24  # 1(type) + 22(channels) + 1(CRC)
            
            # Заголовок пакета
            packet = bytearray()
            packet.append(0xC8)  # Sync byte
            packet.append(packet_length)
            packet.append(packet_type)
            
            # Упакованные каналы
            packed_channels = self.pack_channels(channels)
            packet.extend(packed_channels)
            
            # CRC
            crc_data = packet[2:]  # Все после длины пакета
            crc = self.crc8(crc_data)
            packet.append(crc)
            
            # Отправка через UART
            self.uart.write(packet)
            
            # Отладочная информация (редко чтобы не засорять вывод)
            if time.time() % 5 < 0.1:  # Каждые 5 секунд
                self.print_debug_info(channels)
            
            return True
            
        except Exception as e:
            print(f'❌ Ошибка отправки CRSF пакета: {e}')
            return False
    
    def print_debug_info(self, channels):
        """Вывод отладочной информации"""
        debug_info = (
            f"CRSF: Roll={channels[0]}, Pitch={channels[1]}, "
            f"Throttle={channels[2]}, Yaw={channels[3]}, "
            f"Offset=({self.offset_x:.1f}, {self.offset_y:.1f})"
        )
        print(debug_info)
    
    def send_packets_loop(self):
        """Цикл отправки пакетов (50Hz)"""
        while self.running:
            self.send_crsf_packet()
            time.sleep(0.02)  # 50Hz
    
    def set_auto_mode(self, enable):
        """Установка автоматического режима"""
        self.auto_mode = enable
        mode_str = "АВТОМАТИЧЕСКИЙ" if enable else "РУЧНОЙ"
        print(f'🔀 Режим изменен: {mode_str}')
    
    def start(self):
        """Запуск контроллера"""
        if self.uart and self.uart.is_open:
            self.packet_thread.start()
            print("✅ CRSF Controller запущен")
            return True
        else:
            print("❌ Не удалось запустить CRSF Controller - UART не инициализирован")
            return False
    
    def stop(self):
        """Остановка контроллера"""
        self.running = False
        if self.packet_thread.is_alive():
            self.packet_thread.join(timeout=2.0)
        
        if self.uart and self.uart.is_open:
            self.uart.close()
        
        print("🛑 CRSF Controller остановлен")


class YOLODetector:
    def __init__(self, model_path=None, camera_index=0, crsf_controller=None):
        # Загрузка модели YOLO
        if model_path and os.path.exists(model_path):
            self.model = YOLO(model_path)
            print(f"Загружена локальная модель: {model_path}")
        else:
            self.model = YOLO('yolov8n.pt')  # Используем стандартную модель
            print("⚠️ Локальная модель не найдена, используем YOLOv8n")
        
        # CRSF контроллер
        self.crsf_controller = crsf_controller
        
        # Инициализация камеры
        self.cap = self.initialize_camera(camera_index)
        self.offset_buffer = deque(maxlen=20)
        self.obstacles_buffer = deque(maxlen=10)
        
        # Параметры отображения
        self.screen_width = 720
        self.screen_height = 576
        self.crop_size = 320
        
        try:
            cv2.namedWindow("Detection", cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty("Detection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        except:
            cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    
    def initialize_camera(self, camera_index):
        """Инициализация камеры с обработкой ошибок"""
        camera_sources = [
            camera_index,
            "/dev/video8",
            0,
            "/dev/video9",
            1
        ]
        
        for source in camera_sources:
            try:
                print(f"Попытка подключения к камере: {source}")
                cap = cv2.VideoCapture(source)
                
                if cap.isOpened():
                    # Пробуем разные разрешения
                    resolutions = [(640, 480), (320, 240), (800, 600)]
                    
                    for width, height in resolutions:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                        time.sleep(0.1)
                        
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            print(f"✅ Камера подключена: {source}, разрешение: {frame.shape[1]}x{frame.shape[0]}")
                            return cap
                    
                    ret, frame = cap.read()
                    if ret:
                        print(f"✅ Камера подключена: {source}")
                        return cap
                    
                    cap.release()
                    
            except Exception as e:
                print(f"Ошибка при подключении к {source}: {e}")
        
        print("❌ Не удалось подключиться к камере")
        return None
    
    def crop_frame(self, frame, center_x, center_y, size):
        """Обрезка кадра вокруг центра"""
        if frame is None:
            return None
            
        crop_x1 = max(center_x - size // 2, 0)
        crop_x2 = min(center_x + size // 2, frame.shape[1])
        crop_y1 = max(center_y - size // 2, 0)
        crop_y2 = min(center_y + size // 2, frame.shape[0])
        
        if crop_y2 <= crop_y1 or crop_x2 <= crop_x1:
            return frame
            
        return frame[crop_y1:crop_y2, crop_x1:crop_x2].copy()
    
    def save_offset(self, avg_x, avg_y, obstacles_data):
        """Сохранение смещений в JSON файл"""
        data = {
            'x': avg_x, 
            'y': avg_y,
            'obstacles': obstacles_data,
            'timestamp': time.time()
        }
        
        tmp_filename = 'logs/offsets_tmp.json'
        final_filename = 'logs/offsets.json'
        try:
            with open(tmp_filename, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp_filename, final_filename)
        except Exception as e:
            logging.error(f"Error saving offsets: {e}")
    
    def analyze_obstacles(self, boxes, classes, confidences, frame_width, frame_height):
        """Анализ препятствий на кадре"""
        obstacles = []
        
        for (x1, y1, x2, y2), obj_class, confidence in zip(boxes, classes, confidences):
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            
            frame_center_x = frame_width // 2
            frame_center_y = frame_height // 2
            norm_x = x_center - frame_center_x
            norm_y = y_center - frame_center_y
            
            width = x2 - x1
            height = y2 - y1
            area = width * height
            
            # Получаем имя класса
            class_name = self.model.names[int(obj_class)] if hasattr(self.model, 'names') else str(obj_class)
            
            obstacles.append({
                'x': norm_x,
                'y': norm_y,
                'width': width,
                'height': height,
                'area': area,
                'class': class_name,
                'confidence': float(confidence)
            })
        
        return obstacles
    
    def choose_avoidance_direction(self, obstacles, frame_width):
        """Выбор направления для обхода препятствий"""
        return self.crsf_controller.choose_avoidance_direction(obstacles, frame_width)
    
    def display_frame(self, frame_resized, cropped_frame, fps, center_x, center_y, avg_x, avg_y, avoidance_active, detections_info):
        """Отображение кадра с информацией"""
        detection_preview_size = 200
        
        if cropped_frame is not None:
            cropped_resized = cv2.resize(cropped_frame, (detection_preview_size, detection_preview_size))
            y_offset = self.screen_height - detection_preview_size
            x_offset = self.screen_width - detection_preview_size
            
            # Вставляем превью в правый нижний угол
            if y_offset >= 0 and x_offset >= 0:
                frame_resized[y_offset:y_offset + detection_preview_size, 
                             x_offset:x_offset + detection_preview_size] = cropped_resized

        # Отображаем информацию
        cv2.putText(frame_resized, f"FPS: {fps:.2f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Центральная точка
        cv2.circle(frame_resized, (center_x, center_y), 5, (0, 0, 255), -1)

        # Точка обнаружения
        detection_point_x = center_x + avg_x
        detection_point_y = center_y + avg_y
        
        color = (0, 255, 0) if not avoidance_active else (0, 0, 255)
        cv2.circle(frame_resized, (detection_point_x, detection_point_y), 5, color, -1)
        
        # Линия от центра к точке обнаружения
        cv2.line(frame_resized, (center_x, center_y), (detection_point_x, detection_point_y), color, 2)
        
        # Режим работы
        mode_text = "TRACKING" if not avoidance_active else "AVOIDANCE"
        cv2.putText(frame_resized, f"MODE: {mode_text}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Информация об обнаружениях
        cv2.putText(frame_resized, f"Objects: {detections_info}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Информация о CRSF
        cv2.putText(frame_resized, f"CRSF: Auto", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Detection", frame_resized)
    
    def run(self):
        """Основной цикл программы"""
        if self.cap is None:
            print("❌ Камера не инициализирована, выход")
            return
            
        prev_time = time.time()
        frame_count = 0
        
        print("Запуск основного цикла...")
        print("Нажмите 'q' для выхода")
        print("Нажмите 'a' для переключения авторежима")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("❌ Ошибка получения кадра от камеры")
                break

            current_time = time.time()
            fps = 1 / (current_time - prev_time) if current_time != prev_time else 0
            prev_time = current_time
            frame_count += 1

            # Получаем центр кадра
            center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2
            
            # Обрезаем кадр для обработки
            cropped_frame = self.crop_frame(frame, center_x, center_y, self.crop_size)
            
            if cropped_frame is None:
                print("❌ Ошибка обрезки кадра")
                continue

            # Детекция объектов с YOLO
            offset_x, offset_y = 0, 0
            avoidance_offset = 0
            obstacles_data = []
            detections_info = "0"
            
            try:
                results = self.model(cropped_frame, imgsz=320, conf=0.5, verbose=False)
                
                if results and len(results) > 0:
                    result = results[0]
                    
                    if result.boxes is not None and len(result.boxes) > 0:
                        boxes = result.boxes.xyxy.cpu().numpy()
                        classes = result.boxes.cls.cpu().numpy()
                        confidences = result.boxes.conf.cpu().numpy()
                        
                        # Анализ препятствий
                        obstacles_data = self.analyze_obstacles(boxes, classes, confidences, 
                                                               cropped_frame.shape[1], cropped_frame.shape[0])
                        
                        # Выбор направления обхода
                        avoidance_offset = self.choose_avoidance_direction(obstacles_data, cropped_frame.shape[1])
                        
                        # Используем первое обнаружение для отслеживания
                        if len(boxes) > 0:
                            x1, y1, x2, y2 = boxes[0]
                            x = int((x1 + x2) / 2)
                            y = int((y1 + y2) / 2)

                            cropped_center_x = cropped_frame.shape[1] // 2
                            cropped_center_y = cropped_frame.shape[0] // 2
                            offset_x = x - cropped_center_x + avoidance_offset
                            offset_y = y - cropped_center_y
                        
                        # Информация об обнаружениях
                        class_counts = {}
                        for cls in classes:
                            class_name = self.model.names[int(cls)] if hasattr(self.model, 'names') else str(cls)
                            class_counts[class_name] = class_counts.get(class_name, 0) + 1
                        
                        detections_info = ", ".join([f"{k}:{v}" for k, v in class_counts.items()])
                        
                        # Визуализация на обрезанном кадре
                        result.plot(img=cropped_frame)
                        
                        # Передача данных в CRSF контроллер
                        if self.crsf_controller:
                            self.crsf_controller.update_offsets(offset_x, offset_y)
                            self.crsf_controller.update_obstacles(obstacles_data)
                        
            except Exception as e:
                print(f"❌ Ошибка детекции: {e}")
                detections_info = f"Error: {e}"

            # Буферизация смещений
            self.offset_buffer.append((offset_x, offset_y))
            self.obstacles_buffer.append(obstacles_data)

            # Усреднение смещений
            avg_x = int(np.mean([x for x, _ in self.offset_buffer])) if self.offset_buffer else 0
            avg_y = int(np.mean([y for _, y in self.offset_buffer])) if self.offset_buffer else 0
            
            # Сохранение данных
            self.save_offset(avg_x, avg_y, obstacles_data)

            # Подготовка кадра для отображения
            frame_resized = cv2.resize(frame, (self.screen_width, self.screen_height))
            avoidance_active = (avoidance_offset != 0)
            
            # Отображение
            self.display_frame(frame_resized, cropped_frame, fps, center_x, center_y, 
                             avg_x, avg_y, avoidance_active, detections_info)

            # Обработка клавиш
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Выход из программы")
                break
            elif key == ord('a'):
                if self.crsf_controller:
                    self.crsf_controller.set_auto_mode(not self.crsf_controller.auto_mode)
            elif key == ord('r'):
                print("Перезапуск детекции...")

        self.cap.release()
        cv2.destroyAllWindows()


def main():
    print("=== YOLO + CRSF СИСТЕМА УПРАВЛЕНИЯ ===")
    
    # Создаем CRSF контроллер
    crsf_controller = CRSFController(uart_port='/dev/ttyACM0')
    
    # Запускаем CRSF контроллер
    if not crsf_controller.start():
        print("❌ Не удалось запустить CRSF контроллер")
        return
    
    # Создаем YOLO детектор с привязкой к CRSF контроллеру
    model_path = 'models/best_rknn_model'  # Укажите путь к вашей модели
    
    detector = YOLODetector(model_path=model_path, camera_index=0, crsf_controller=crsf_controller)
    
    try:
        # Запускаем детектор
        detector.run()
        
    except KeyboardInterrupt:
        print("\n                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  Программа завершена пользователем")
    except Exception as e:
        print(f" Критическая ошибка: {e}")
    finally:
        # Останавливаем контроллер
        crsf_controller.stop()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import cv2
import numpy as np
import time
import json
import os
from ultralytics import YOLO
from collections import deque
import logging

# Создаем папку для логов если не существует
os.makedirs('logs', exist_ok=True)

# Настройка логирования
logging.basicConfig(
    filename='logs/detections.log', 
    level=logging.INFO, 
    format='%(asctime)s - %(message)s'
)

class YOLODetector:
    def __init__(self, model_path=None, camera_index=0):
        # Загрузка модели YOLO
        if model_path and os.path.exists(model_path):
            self.model = YOLO(model_path)
            print(f"Загружена локальная модель: {model_path}")
        else:
            self.model = YOLO('yolov8n.pt')  # Используем стандартную модель
            print(" Локальная модель не найдена, используем YOLOv8n")
        
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
                            print(f"✓ Камера подключена: {source}, разрешение: {frame.shape[1]}x{frame.shape[0]}")
                            return cap
                    
                    ret, frame = cap.read()
                    if ret:
                        print(f"✓ Камера подключена: {source}")
                        return cap
                    
                    cap.release()
                    
            except Exception as e:
                print(f"Ошибка при подключении к {source}: {e}")
        
        print(" Не удалось подключиться к камере")
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
                return -60
            elif left_occupied and not right_occupied:
                return 60
            elif not left_occupied and not right_occupied:
                left_count = len(left_obstacles)
                right_count = len(right_obstacles)
                return -60 if left_count < right_count else 60
            else:
                return 40 if len(left_obstacles) > len(right_obstacles) else -40
        
        return 0
    
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

        cv2.imshow("Detection", frame_resized)
    
    def run(self):
        """Основной цикл программы"""
        if self.cap is None:
            print(" Камера не инициализирована, выход")
            return
            
        prev_time = time.time()
        frame_count = 0
        
        print("Запуск основного цикла...")
        print("Нажмите 'q' для выхода")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Ошибка получения кадра от камеры")
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
                print(" Ошибка обрезки кадра")
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
                        
            except Exception as e:
                print(f" Ошибка детекции: {e}")
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
            elif key == ord('r'):
                print("Перезапуск детекции...")

        self.cap.release()
        cv2.destroyAllWindows()

def check_system():
    """Проверка системы и зависимостей"""
    print("=== ПРОВЕРКА СИСТЕМЫ ===")
    
    # Проверяем доступные камеры
    print("Доступные камеры:")
    for i in range(5):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"  ✓ Камера {i}: {frame.shape[1]}x{frame.shape[0]}")
            else:
                print(f"  ✗ Камера {i}: подключена, но не возвращает кадры")
            cap.release()
        else:
            print(f"  ✗ Камера {i}: недоступна")
    
    # Проверяем модель YOLO
    print("\nПроверка модели YOLO...")
    try:
        model = YOLO('yolov8n.pt')
        print("  ✓ YOLO модель загружена успешно")
    except Exception as e:
        print(f"  ✗ Ошибка загрузки YOLO: {e}")

if __name__ == "__main__":
    print("=== YOLO ДЕТЕКТОР ===")
    
    # Проверка системы
    check_system()
    
    # Запуск детектора
    # Пробуем загрузить локальную модель, если есть
    model_path = 'models/best_rknn_model'  # Укажите путь к вашей модели
    
    detector = YOLODetector(model_path=model_path, camera_index=0)
    
    try:
        detector.run()
    except KeyboardInterrupt:
        print("\nПрограмма завершена пользователем")
    except Exception as e:
        print(f" Критическая ошибка: {e}")
        
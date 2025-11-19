import cv2
import time
import os
from datetime import datetime
import subprocess

class PeopleDetector:
    def __init__(self):
        self.body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')
        self.upper_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_upperbody.xml')
        self.lower_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_lowerbody.xml')
        
        self.save_dir = "detected_people"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
    
    def find_sunplus_camera(self):
        """Поиск USB-камеры Sunplus по V4L2 устройствам"""
        usb_camera_paths = [
            "/dev/video8",  # Основной видеоинтерфейс USB-камеры
            "/dev/video9",  # Дополнительный видеоинтерфейс
            #"/dev/video0",  # Первая камера (rp1-cfe)
            #"/dev/video1",  # Вторая камера (rp1-cfe)
            #"/dev/video19"  # rpivid
        ]
        
        for device_path in usb_camera_paths:
            if os.path.exists(device_path):
                print(f"Проверяем устройство: {device_path}")
                try:
                    # Пробуем открыть устройство
                    cap = cv2.VideoCapture(device_path)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret:
                            print(f"✓ Успешно: {device_path} - разрешение {frame.shape[1]}x{frame.shape[0]}")
                            cap.release()
                            return device_path
                        cap.release()
                except Exception as e:
                    print(f"✗ Ошибка с {device_path}: {e}")
        
        print("Камера не найдена, пробуем стандартные индексы...")
        return None
    
    def get_camera_info(self):
        """Получение информации о камерах в системе"""
        try:
            result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                  capture_output=True, text=True)
            print("Доступные камеры в системе:")
            print(result.stdout)
            
            # Проверяем конкретно USB-камеру
            print("\nИнформация о USB-камере:")
            subprocess.run(['v4l2-ctl', '-d', '/dev/video8', '--list-formats'], check=False)
            
        except Exception as e:
            print(f"Ошибка получения информации: {e}")
    
    def detect_people(self, frame):
        """Обнаружение людей на кадре"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        bodies = self.body_cascade.detectMultiScale(gray, 1.1, 3, minSize=(30, 30))
        upper_bodies = self.upper_body_cascade.detectMultiScale(gray, 1.1, 3, minSize=(30, 30))
        lower_bodies = self.lower_body_cascade.detectMultiScale(gray, 1.1, 3, minSize=(30, 30))
        
        detections = []
        
        for (x, y, w, h) in bodies:
            detections.append((x, y, w, h, 'body'))
        
        for (x, y, w, h) in upper_bodies:
            detections.append((x, y, w, h, 'upper_body'))
            
        for (x, y, w, h) in lower_bodies:
            detections.append((x, y, w, h, 'lower_body'))
        
        return detections
    
    def draw_detections(self, frame, detections):
        for (x, y, w, h, body_part) in detections:
            if body_part == 'body':
                color = (0, 255, 0)  
            elif body_part == 'upper_body':
                color = (255, 0, 0) 
            else:
                color = (0, 0, 255)  
                
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, body_part, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        cv2.putText(frame, f"People detected: {len(detections)}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return frame
    
    def save_detection(self, frame, detections):
        """Сохранение кадра с обнаруженными людьми"""
        if detections:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{self.save_dir}/detection_{timestamp}_{len(detections)}_people.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename}")
    
    def run_detection(self, camera_device=None):
        """Запуск обнаружения людей с веб-камеры"""
        
        if camera_device is None:
            camera_device = self.find_sunplus_camera()
        
        self.get_camera_info()
        
        if camera_device is None:
            print("Камера не найдена! Пробуем стандартные индексы...")
            for i in range(5):
                print(f"Пробуем индекс {i}...")
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        print(f"Найдена камера с индексом {i}")
                        camera_device = i
                        break
                    cap.release()
            
            if camera_device is None:
                print("Ошибка: Не удалось найти ни одну камеру!")
                return
        
        print(f"Подключаемся к камере: {camera_device}")
        
        
        if isinstance(camera_device, str):  # Если это путь к устройству
            cap = cv2.VideoCapture(camera_device)
        else:  
            cap = cv2.VideoCapture(camera_device)
        
        if not cap.isOpened():
            print("Пробуем с бэкендом V4L2...")
            if isinstance(camera_device, str):
                cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
            else:
                cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            print("Ошибка: Не удалось подключиться к камере")
            return
        
        resolutions = [
            (640, 480),
            (320, 240),
            (800, 600),
            (1280, 720)
        ]
        
        for width, height in resolutions:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Установлено разрешение: {actual_width}x{actual_height}")
            
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"✓ Работает с разрешением: {frame.shape[1]}x{frame.shape[0]}")
                break
            else:
                print(f"✗ Не работает с {actual_width}x{actual_height}")
        
        print("Камера успешно подключена!")
        print("Запуск обнаружения людей...")
        print("Нажмите 'q' для выхода")
        print("Нажмите 's' для сохранения текущего кадра")
        
        last_save_time = 0
        save_interval = 2  # Интервал сохранения в секундах
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Ошибка: Не удалось получить кадр")
                break
            
            if frame is None:
                print("Получен пустой кадр")
                continue
            
            detections = self.detect_people(frame)
            
            frame_with_boxes = self.draw_detections(frame.copy(), detections)
            
            current_time = time.time()
            if detections and (current_time - last_save_time) > save_interval:
                self.save_detection(frame, detections)
                last_save_time = current_time
            
            cv2.imshow('People Detection - Sunplus Camera', frame_with_boxes)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_detection(frame, detections)
        
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = PeopleDetector()
    
    
    detector.run_detection("/dev/video8")
    

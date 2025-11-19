import cv2
import time
import os
from datetime import datetime
import subprocess

class PeopleDetector:
    def __init__(self):
        # Инициализация классификаторов для обнаружения людей
        self.body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')
        self.upper_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_upperbody.xml')
        self.lower_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_lowerbody.xml')
        
        # Создаем папку для сохранения фото
        self.save_dir = "detected_people"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
    
    def find_sunplus_camera(self):
        """Поиск индекса камеры Sunplus в системе"""
        # Проверяем доступные камеры
        for i in range(10):  # Проверяем первые 10 индексов
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                # Получаем информацию о камере
                backend = cap.getBackendName()
                print(f"Camera {i}: Backend {backend}")
                
                # Пробуем получить кадр
                ret, frame = cap.read()
                if ret:
                    print(f"Camera {i}: Resolution {frame.shape[1]}x{frame.shape[0]}")
                    cap.release()
                    return i
                cap.release()
        
        print("Sunplus camera not found automatically, trying default index 0")
        return 0
    
    def get_camera_info(self):
        """Получение информации о камерах в системе"""
        try:
            # Используем v4l2-ctl для получения информации о камерах
            result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                  capture_output=True, text=True)
            print("Available cameras:")
            print(result.stdout)
        except:
            print("v4l2-ctl not available")
    
    def detect_people(self, frame):
        """Обнаружение людей на кадре"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Обнаружение разных частей тела
        bodies = self.body_cascade.detectMultiScale(gray, 1.1, 3)
        upper_bodies = self.upper_body_cascade.detectMultiScale(gray, 1.1, 3)
        lower_bodies = self.lower_body_cascade.detectMultiScale(gray, 1.1, 3)
        
        detections = []
        
        # Объединяем все обнаружения
        for (x, y, w, h) in bodies:
            detections.append((x, y, w, h, 'body'))
        
        for (x, y, w, h) in upper_bodies:
            detections.append((x, y, w, h, 'upper_body'))
            
        for (x, y, w, h) in lower_bodies:
            detections.append((x, y, w, h, 'lower_body'))
        
        return detections
    
    def draw_detections(self, frame, detections):
        """Отрисовка bounding boxes вокруг обнаруженных людей"""
        for (x, y, w, h, body_part) in detections:
            if body_part == 'body':
                color = (0, 255, 0)  # Зеленый для полного тела
            elif body_part == 'upper_body':
                color = (255, 0, 0)  # Синий для верхней части
            else:
                color = (0, 0, 255)  # Красный для нижней части
                
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
    
    def run_detection(self, camera_index=None):
        """Запуск обнаружения людей с веб-камеры"""
        
        # Если индекс не указан, ищем камеру Sunplus
        if camera_index is None:
            camera_index = self.find_sunplus_camera()
        
        # Показываем информацию о камерах
        self.get_camera_info()
        
        print(f"Trying to open camera with index: {camera_index}")
        cap = cv2.VideoCapture(camera_index)
        
        # Пробуем разные бэкенды если нужно
        if not cap.isOpened():
            print("Trying with V4L2 backend...")
            cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            print("Trying with ANY backend...")
            cap = cv2.VideoCapture(camera_index, cv2.CAP_ANY)
        
        if not cap.isOpened():
            print("Ошибка: Не удалось подключиться к камере")
            print("Попробуйте указать индекс камеры вручную:")
            print("0 - первая камера, 1 - вторая камера, и т.д.")
            return
        
        # Устанавливаем параметры камеры
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
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
            
            # Обнаружение людей
            detections = self.detect_people(frame)
            
            # Отрисовка bounding boxes
            frame_with_boxes = self.draw_detections(frame.copy(), detections)
            
            # Автосохранение при обнаружении (с интервалом)
            current_time = time.time()
            if detections and (current_time - last_save_time) > save_interval:
                self.save_detection(frame, detections)
                last_save_time = current_time
            
            # Показ результата
            cv2.imshow('People Detection - Sunplus Camera', frame_with_boxes)
            
            # Обработка клавиш
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_detection(frame, detections)
        
        cap.release()
        cv2.destroyAllWindows()

# Альтернативный вариант - прямое указание устройства V4L2
def run_with_device_path():
    """Запуск с прямым указанием пути к устройству"""
    detector = PeopleDetector()
    
    # Пробуем разные пути к устройствам
    device_paths = [
        "/dev/video0", "/dev/video1", "/dev/video2", 
        "/dev/video3", "/dev/video4"
    ]
    
    for device_path in device_paths:
        if os.path.exists(device_path):
            print(f"Trying device: {device_path}")
            # В OpenCV можно использовать индекс или путь
            try:
                # Преобразуем путь в индекс (например, /dev/video2 -> индекс 2)
                index = int(device_path.replace("/dev/video", ""))
                detector.run_detection(index)
                break
            except ValueError:
                continue

# Запуск детектора
if __name__ == "__main__":
    # Способ 1: Автоматический поиск
    detector = PeopleDetector()
    #detector.run_detection()
    detector.run_detection(2)
    # Способ 2: Ручное указание индекса
    # detector.run_detection(0)  # Первая камера
    # detector.run_detection(1)  # Вторая камера
    # detector.run_detection(2)  # Третья камера
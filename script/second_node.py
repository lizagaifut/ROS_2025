#!/usr/bin/env python3
import time
import datetime

def main():
    print("Программа запущена. Вывод времени каждые 5 секунд")
    print("Для остановки нажмите Ctrl+C\n")
    
    try:
        while True:
            # Получаем текущее время
            current_time = datetime.datetime.now()
            # Форматируем время в удобный вид
            formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
            # Выводим время
            print(f'Текущее время: {formatted_time}')
            # Ждем 5 секунд
            time.sleep(5)
            
    except KeyboardInterrupt:
        print("\nПрограмма остановлена")

if __name__ == '__main__':
    main()
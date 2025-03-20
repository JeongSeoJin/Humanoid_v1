#!/usr/bin/env python3

"""
이 스크립트는 Ubuntu에서 pyserial을 통해 ESP32(마스터)로 
문자열 명령을 전송하고, 그 응답(에코, 디버그 출력)을 
화면에 표시한다.

실행 예:
    python3 can_master_serial.py

명령 입력 예:
    S110 10
    S120 5.5
    R15
    ...
"""

import serial
import time
import sys

def main():
    # 1) 시리얼 포트 열기
    port_name = "/dev/ttyUSB0"  # 실제 상황에 맞춰 변경
    baud_rate = 115200
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=0.5)
        print(f"Opened serial port: {port_name} at {baud_rate} baud.")
    except Exception as e:
        print(f"Error opening serial port {port_name}: {e}")
        sys.exit(1)
    
    # 2) 사용자 입력 루프
    print("Type commands for the ESP32 (e.g. 'S110 10').")
    print("Press Ctrl+C or type 'exit' to quit.")
    
    try:
        while True:
            user_input = input(">> ").strip()
            if not user_input:
                continue
            
            # 종료 조건
            if user_input.lower() in ["exit", "quit"]:
                break
            
            # 시리얼로 전송 (Commander 명령 규칙에 따라 '\n' or '\r\n' 필요)
            # positions = ["R10", "S110 -10", "R0", "S110 0"]

            cmd_str = user_input + "\n"
            ser.write(cmd_str.encode('utf-8'))
            
            # 조금 기다렸다가(옵션)
            time.sleep(0.05)
            
            # ESP32가 보낸 응답 읽기
            # (Commander는 줄단위로 응답할 가능성이 있으므로 while로 여러 줄 받기)
            response_lines = []
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                if line:
                    response_lines.append(line)
            
            # 받은 라인들 출력
            if response_lines:
                print("<< Received:")
                for line in response_lines:
                    print("   " + line)
            else:
                print("<< (no response)")
            
    except KeyboardInterrupt:
        print("\nExiting...")

    # 3) 종료 처리
    ser.close()
    print("Serial port closed.")


if __name__ == "__main__":
    main()

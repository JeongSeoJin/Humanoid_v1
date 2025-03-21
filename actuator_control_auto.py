#!/usr/bin/env python3

"""
자동으로 여러 명령(R10, S110 -10 등)을 순서대로 ESP32(마스터)로 전송하고,
각 명령의 응답을 화면에 표시하는 예시 코드.

실행:
    python3 can_master_serial.py
(포트명, 명령 내용 등을 필요에 맞게 수정)
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
        print(f"Opened serial port: {port_name} at {baud_rate} baud.\n")
    except Exception as e:
        print(f"Error opening serial port {port_name}: {e}")
        sys.exit(1)
    
    # 2) 자동 전송할 명령 리스트
    # 필요한 명령을 자유롭게 추가/수정 가능
    positions = [
        "K1.5",
        "P110 1.5",
        "P120 2",

        "C0.5",
        "D110 0.7",
        "D120 0.3"

        "R-2",
        "S120 0",       # 예: 모든 슬레이브(브로드캐스트) ref angle=10
        "S110 -5",  # 예: 슬레이브 0x110에 angle=-10
        "R-20",
        "S120 15",
        "S110 10",
        "S120 0",
        "R-2",
        "S110 -5"
    ]

    # 3) 순차적으로 명령 전송
    try:
        for cmd in positions:
            cmd_str = cmd + "\n"
            
            print(f">> Sending: {cmd_str.strip()}")
            ser.write(cmd_str.encode('utf-8'))
            
            # 명령 전송 후, 대기 (하드웨어 반응 시간)
            time.sleep(0.2)

            # ESP32 응답 수신
            response_lines = []
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                if line:
                    response_lines.append(line)

            if response_lines:
                print("<< Received:")
                for line in response_lines:
                    print("   " + line)
            else:
                print("<< (no response)")

            # 명령 사이 간격 (필요시 변경)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    
    # 4) 종료 처리
    ser.close()
    print("\nSerial port closed.")

if __name__ == "__main__":
    main()

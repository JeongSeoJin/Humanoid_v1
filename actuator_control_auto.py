#!/usr/bin/env python3

# master : R10, K1, C0.5
# slaves : S110 P1, D0.5

import serial
import time
import sys

def main():
    # 1) 시리얼 포트 열기
    port_name = "/dev/ttyUSB0" # 실제 상황에 맞춰 변경
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
        "1r-7",
        "2r0",
        "3r0",
        "4r8",
        "1r0",
        "2r-5",
        "3r10",
        "4r0",
        "1r-7",
        "2r0",
        "3r0",
        "4r8",
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
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    
    # 4) 종료 처리
    ser.close()
    print("\nSerial port closed.")

if __name__ == "__main__":
    main()

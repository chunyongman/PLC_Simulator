#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESS HMI PLC 시뮬레이터
Engine Room Ventilation System PLC를 시뮬레이션합니다.
"""

import sys
import io
import time
import random
import threading
from datetime import datetime

# Windows 콘솔 인코딩 문제 해결
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

try:
    from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
    from pymodbus.datastore import ModbusSequentialDataBlock
    from pymodbus.server.sync import StartTcpServer
    from pymodbus.device import ModbusDeviceIdentification
except ImportError as e:
    print(f"ERROR: pymodbus library import failed: {e}")
    print("Trying alternative import...")
    try:
        from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
        from pymodbus.datastore import ModbusSequentialDataBlock
        from pymodbus.server import StartTcpServer
        from pymodbus.device import ModbusDeviceIdentification
    except ImportError:
        print("ERROR: pymodbus library is required.")
        print("Install: pip install pymodbus")
        sys.exit(1)


class ESSPLCSimulator:
    """ESS (Engine Room Ventilation) PLC 시뮬레이터"""

    def __init__(self):
        print("=" * 70)
        print("  ESS HMI PLC 시뮬레이터")
        print("  Engine Room Ventilation System")
        print("=" * 70)

        # Modbus 데이터 스토어 초기화 (65535개 레지스터/코일)
        self.store = ModbusSlaveContext(
            di=ModbusSequentialDataBlock(0, [0]*65535),    # Discrete Inputs
            co=ModbusSequentialDataBlock(0, [0]*65535),    # Coils
            hr=ModbusSequentialDataBlock(0, [0]*65535),    # Holding Registers
            ir=ModbusSequentialDataBlock(0, [0]*65535)     # Input Registers
        )

        # 시뮬레이션 상태
        self.running = True

        # 알람 시나리오 카운터 (60초마다 알람 조건 생성)
        self.alarm_scenario_counter = 0
        self.alarm_active = False

        # 장비 상태 (3 SWP, 3 FWP, 4 Fans)
        self.equipment = {
            # Sea Water Pumps
            'SWP1': {'running': True, 'ess_on': True, 'abnormal': False, 'hz': 45.5, 'auto_mode': True, 'vfd_mode': True},
            'SWP2': {'running': True, 'ess_on': True, 'abnormal': False, 'hz': 46.2, 'auto_mode': True, 'vfd_mode': True},
            'SWP3': {'running': False, 'ess_on': False, 'abnormal': False, 'hz': 0.0, 'auto_mode': True, 'vfd_mode': True},

            # Fresh Water Pumps
            'FWP1': {'running': True, 'ess_on': True, 'abnormal': False, 'hz': 48.1, 'auto_mode': True, 'vfd_mode': True},
            'FWP2': {'running': True, 'ess_on': True, 'abnormal': False, 'hz': 47.8, 'auto_mode': True, 'vfd_mode': True},
            'FWP3': {'running': False, 'ess_on': False, 'abnormal': False, 'hz': 0.0, 'auto_mode': True, 'vfd_mode': True},

            # E/R Fans
            'FAN1': {'running_fwd': True, 'running_bwd': False, 'abnormal': False, 'hz': 50.0, 'auto_mode': True, 'vfd_mode': True},
            'FAN2': {'running_fwd': True, 'running_bwd': False, 'abnormal': False, 'hz': 49.5, 'auto_mode': True, 'vfd_mode': True},
            'FAN3': {'running_fwd': False, 'running_bwd': False, 'abnormal': False, 'hz': 0.0, 'auto_mode': True, 'vfd_mode': True},
            'FAN4': {'running_fwd': False, 'running_bwd': False, 'abnormal': False, 'hz': 0.0, 'auto_mode': True, 'vfd_mode': True}
        }

        # 센서 베이스 값 (물리 법칙 적용)
        # 계절: 여름 (한국 근해 가정)
        self.seawater_temp = 24.0  # 바닷물 온도 (여름: 22-26°C, 겨울: 8-12°C)
        self.ambient_temp = 28.0   # 외기 온도 (여름: 26-32°C, 겨울: 0-10°C)

        self.base_temps = {
            'TX1': self.seawater_temp,  # COOLER SW INLET: 바닷물 온도
            'TX2': 0.0,  # NO.1 COOLER SW OUTLET: 계산됨 (>TX1, <49C)
            'TX3': 0.0,  # NO.2 COOLER SW OUTLET: 계산됨 (>TX1, <49C)
            'TX4': 0.0,  # COOLER FW INLET: 계산됨 (<48C)
            'TX5': 35.0, # COOLER FW OUTLET: 목표 34-36C
            'TX6': 35.0, # E/R INSIDE: 35C로 낮춤 (AI가 팬을 47Hz로 제어하도록)
            'TX7': self.ambient_temp   # E/R OUTSIDE: 외기 온도
        }

        self.base_pressure = {
            'DPX1': 2.0,  # SW DISCHARGE PRESS: 1.0~3.0 BAR
            'DPX2': 15.0  # E/R Diff Press (Pa)
        }

        self.me_load = 55.0  # M/E Load % (60% 이하)

        # Edge AI 결과 저장 레지스터 초기화
        # 5000-5009: AI 목표 주파수 (Hz × 10)
        # 5100-5109: 절감 전력 (kW × 10)
        # 5200-5209: VFD 진단 점수 (0-100)
        # 5300-5303: 시스템 절감률 (% × 10)
        self.store.setValues(3, 5000, [0] * 10)  # AI 목표 주파수
        self.store.setValues(3, 5100, [0] * 10)  # 절감 전력
        self.store.setValues(3, 5200, [100] * 10)  # VFD 진단 점수 (초기값 100=정상)
        self.store.setValues(3, 5300, [0] * 4)   # 시스템 절감률

        print("[OK] 데이터 스토어 초기화 완료")
        print("[INFO] Modbus TCP 서버: 192.168.0.130:502")
        print("[INFO] Node ID: 3")
        print("[INFO] Edge AI 결과 레지스터: 5000-5399 (Ready)")
        print("-" * 70)

    def temperature_to_raw(self, temp_celsius):
        """온도를 PLC raw 값으로 변환 (-24.3~100°C -> -243~1000)"""
        return int(temp_celsius * 10)

    def pressure_kgcm2_to_raw(self, pressure):
        """압력(kg/cm²)을 raw 값으로 변환 (0~6 -> 0~27648)"""
        return int(pressure * 4608)

    def pressure_pa_to_raw(self, pressure):
        """압력(Pa)을 raw 값으로 변환"""
        return int(pressure * 10)

    def percentage_to_raw(self, percentage):
        """퍼센트를 raw 값으로 변환 (0~100% -> 0~27648)"""
        return int(percentage * 276.48)

    def hz_to_raw(self, hz):
        """주파수를 raw 값으로 변환 (0~100Hz -> 0~1000)"""
        return int(hz * 10)

    def simulate_sensor_values(self):
        """센서 값 실시간 시뮬레이션"""
        print("[시작] 센서 데이터 시뮬레이션 스레드")

        while self.running:
            try:
                # 알람 시나리오: 비활성화 (정상 운전 모드)
                # self.alarm_scenario_counter += 1

                # if self.alarm_scenario_counter >= 60 and not self.alarm_active:
                #     # 알람 조건 시작
                #     self.alarm_active = True
                #     self.alarm_scenario_counter = 0
                #     print("=" * 70)
                #     print("[시뮬레이터] 🔔 알람 시나리오 시작 (15초간 유지)")
                #     print("  - 🔴 주기관 부하 과다 (PU1: 60% → 90%, CRITICAL)")
                #     print("  - 🔴 외부 공기 온도 상승 (TX7: 25°C → 42°C, CRITICAL)")
                #     print("  - ⚠️ E/R 내부 온도 상승 (TX6: 40°C → 52°C, WARNING)")
                #     print("  - ⚠️ SW 압력 저하 (DPX1: 3.5 → 1.3 kg/cm², WARNING)")
                #     print("=" * 70)

                # if self.alarm_active and self.alarm_scenario_counter >= 15:
                #     # 알람 조건 해제
                #     self.alarm_active = False
                #     self.alarm_scenario_counter = 0
                #     print("=" * 70)
                #     print("[시뮬레이터] ✅ 알람 시나리오 종료 (정상 복귀)")
                #     print("  알람은 165초 후 재발생")
                #     print("  (현재 알람은 확인 전까지 유지됨)")
                #     print("=" * 70)

                # === 물리 법칙 기반 온도 센서 시뮬레이션 ===

                # TX1 (COOLER SW INLET): 바닷물 온도 (여름: 22-26°C)
                tx1 = self.seawater_temp + random.uniform(-0.5, 0.5)

                # TX7 (E/R OUTSIDE): 외기 온도 - 알람 시나리오 적용
                if self.alarm_active:
                    tx7 = 42.0 + random.uniform(-0.5, 0.5)  # CRITICAL 알람 조건 (HIGH: 40°C 이상)
                else:
                    tx7 = self.ambient_temp + random.uniform(-1.0, 1.0)

                # 현재 M/E 부하율에 따른 열부하 계산
                heat_load_factor = self.me_load / 60.0  # 60%를 기준(1.0)으로 정규화

                # TX2 (NO.1 COOLER SW OUTLET): TX1보다 높고 49°C 이하
                # 냉각수가 엔진을 냉각하면서 온도 상승 (부하에 비례)
                delta_t_sw_no1 = 8.0 * heat_load_factor  # 기본 온도 상승: 8°C
                tx2 = min(tx1 + delta_t_sw_no1 + random.uniform(-0.5, 0.5), 48.5)

                # TX3 (NO.2 COOLER SW OUTLET): TX1보다 높고 49°C 이하
                # 2차 냉각기, NO.1보다 약간 낮을 수 있음
                delta_t_sw_no2 = 6.0 * heat_load_factor
                tx3 = min(tx1 + delta_t_sw_no2 + random.uniform(-0.5, 0.5), 48.5)

                # TX5 (COOLER FW OUTLET): 목표 34-36°C (AI 제어 목표)
                tx5 = 35.0 + random.uniform(-0.8, 0.8)  # 정상 범위 유지

                # TX4 (COOLER FW INLET): TX5보다 높고 48°C 이하
                # FW가 엔진을 냉각한 후 온도 (TX5보다 7-10°C 높음)
                delta_t_fw = 8.0 + 3.0 * heat_load_factor
                tx4 = min(tx5 + delta_t_fw + random.uniform(-0.5, 0.5), 47.5)

                # TX6 (E/R INSIDE): 목표 35°C로 낮춤 (AI가 팬을 47Hz로 제어하도록)
                if self.alarm_active:
                    tx6 = 52.0 + random.uniform(-0.5, 0.5)  # WARNING 알람 조건 (HIGH: 50°C 이상)
                else:
                    # 기본 온도 35°C 사용
                    tx6 = self.base_temps['TX6'] + random.uniform(-2.0, 2.0)

                # Holding Registers에 쓰기 (address 10~16)
                self.store.setValues(3, 10, [
                    self.temperature_to_raw(tx1),
                    self.temperature_to_raw(tx2),
                    self.temperature_to_raw(tx3),
                    self.temperature_to_raw(tx4),
                    self.temperature_to_raw(tx5),
                    self.temperature_to_raw(tx6),
                    self.temperature_to_raw(tx7)
                ])

                # === 압력 센서 (K400017~K400018) ===
                # PX1 (SW DISCHARGE PRESS): 1.0~3.0 BAR - 펌프 운전 대수와 부하에 비례
                if self.alarm_active:
                    dpx1 = 1.3 + random.uniform(-0.05, 0.05)  # 알람 조건 (LOW: 1.5 bar 이하)
                else:
                    # 운전 중인 SW 펌프 대수 확인
                    swp_running_count = sum([
                        1 for p in ['SWP1', 'SWP2', 'SWP3']
                        if self.equipment[p]['running']
                    ])
                    # 펌프 대수와 부하에 따라 압력 변동 (1대: 1.5 bar, 2대: 2.5 bar)
                    base_pressure = 1.0 + swp_running_count * 0.7
                    dpx1 = base_pressure + 0.3 * (self.me_load / 60.0) + random.uniform(-0.1, 0.1)
                    dpx1 = max(1.0, min(3.0, dpx1))  # 1.0~3.0 BAR 범위 제한

                # DPX2 (E/R Diff Press): E/R 내외부 압력차 (Pa)
                # 팬 운전 대수에 비례하여 양압 유지
                dpx2 = self.base_pressure['DPX2'] + random.uniform(-2.0, 2.0)

                self.store.setValues(3, 17, [
                    self.pressure_kgcm2_to_raw(dpx1),
                    self.pressure_pa_to_raw(dpx2)
                ])

                # M/E Load (K400019) - 알람 시나리오 적용
                if self.alarm_active:
                    self.me_load = 90.0 + random.uniform(-0.5, 0.5)  # CRITICAL 알람 조건 (HIGH: 85% 이상)
                else:
                    self.me_load += random.uniform(-0.8, 0.8)
                    self.me_load = max(35, min(60, self.me_load))  # 정상 범위: 35~60% (60% 이하)
                self.store.setValues(3, 19, [self.percentage_to_raw(self.me_load)])

                # 장비 상태 업데이트
                self.update_equipment_status()

                # VFD 데이터 업데이트
                self.update_vfd_data()

                time.sleep(1)  # 1초마다 업데이트

            except Exception as e:
                print(f"[ERROR] 센서 시뮬레이션 오류: {e}")
                time.sleep(1)

    def update_equipment_status(self):
        """장비 상태 비트 업데이트 (K4000~K4001)"""

        # K4000 (Word 0) - 비트 0~15
        word_4000 = 0
        if self.equipment['SWP1']['running']: word_4000 |= (1 << 0)
        if self.equipment['SWP1']['ess_on']: word_4000 |= (1 << 1)
        if self.equipment['SWP1']['abnormal']: word_4000 |= (1 << 2)
        if self.equipment['SWP2']['running']: word_4000 |= (1 << 3)
        if self.equipment['SWP2']['ess_on']: word_4000 |= (1 << 4)
        if self.equipment['SWP2']['abnormal']: word_4000 |= (1 << 5)
        if self.equipment['SWP3']['running']: word_4000 |= (1 << 6)
        if self.equipment['SWP3']['ess_on']: word_4000 |= (1 << 7)
        if self.equipment['SWP3']['abnormal']: word_4000 |= (1 << 8)
        if self.equipment['FWP1']['running']: word_4000 |= (1 << 9)
        if self.equipment['FWP1']['ess_on']: word_4000 |= (1 << 10)
        if self.equipment['FWP1']['abnormal']: word_4000 |= (1 << 11)
        if self.equipment['FWP2']['running']: word_4000 |= (1 << 12)
        if self.equipment['FWP2']['ess_on']: word_4000 |= (1 << 13)
        if self.equipment['FWP2']['abnormal']: word_4000 |= (1 << 14)
        if self.equipment['FWP3']['running']: word_4000 |= (1 << 15)

        # K4001 (Word 1) - 비트 0~15
        word_4001 = 0
        if self.equipment['FWP3']['ess_on']: word_4001 |= (1 << 0)
        if self.equipment['FWP3']['abnormal']: word_4001 |= (1 << 1)
        if self.equipment['FAN1']['running_fwd']: word_4001 |= (1 << 2)
        if self.equipment['FAN1']['running_bwd']: word_4001 |= (1 << 3)
        if self.equipment['FAN1']['abnormal']: word_4001 |= (1 << 4)
        if self.equipment['FAN2']['running_fwd']: word_4001 |= (1 << 5)
        if self.equipment['FAN2']['running_bwd']: word_4001 |= (1 << 6)
        if self.equipment['FAN2']['abnormal']: word_4001 |= (1 << 7)
        if self.equipment['FAN3']['running_fwd']: word_4001 |= (1 << 8)
        if self.equipment['FAN3']['running_bwd']: word_4001 |= (1 << 9)
        if self.equipment['FAN3']['abnormal']: word_4001 |= (1 << 10)
        if self.equipment['FAN4']['running_fwd']: word_4001 |= (1 << 11)
        if self.equipment['FAN4']['running_bwd']: word_4001 |= (1 << 12)
        if self.equipment['FAN4']['abnormal']: word_4001 |= (1 << 13)

        # Coil 주소: K4000.x = address 64000 + bit
        # 하지만 Modbus는 Word 단위로 저장하므로 Holding Register 사용
        self.store.setValues(3, 4000, [word_4000, word_4001])

    def update_vfd_data(self):
        """VFD 운전 데이터 업데이트 (K400160~K400238)"""

        # SWP1~3, FWP1~3, FAN1~4 각 8개 레지스터
        vfd_configs = [
            ('SWP1', 160), ('SWP2', 168), ('SWP3', 176),
            ('FWP1', 184), ('FWP2', 192), ('FWP3', 200),
            ('FAN1', 208), ('FAN2', 216), ('FAN3', 224), ('FAN4', 232)
        ]

        for i, (eq_name, start_addr) in enumerate(vfd_configs):
            eq = self.equipment[eq_name]
            running = eq.get('running', False) or eq.get('running_fwd', False) or eq.get('running_bwd', False)

            # ================================================================
            # 1단계: PLC가 Edge AI 목표 주파수를 VFD에 전송 (명령)
            # ================================================================
            vfd_command_freq = eq['hz']  # 현재 VFD 명령 주파수

            # AUTO 모드이고 VFD 모드일 때만 Edge AI 주파수 사용
            auto_mode = eq.get('auto_mode', True)
            vfd_mode = eq.get('vfd_mode', True)

            if auto_mode and vfd_mode and running:
                # Edge AI 목표 주파수 읽기 (레지스터 5000-5009)
                try:
                    ai_freq_raw = self.store.getValues(3, 5000 + i, 1)[0]
                    if ai_freq_raw > 0:  # AI 주파수가 설정되어 있으면
                        ai_freq_hz = ai_freq_raw / 10.0
                        # AI 목표 주파수로 서서히 변경 (급격한 변화 방지)
                        if abs(ai_freq_hz - vfd_command_freq) > 0.5:
                            # 0.5Hz씩 서서히 변경
                            if ai_freq_hz > vfd_command_freq:
                                vfd_command_freq = min(vfd_command_freq + 0.5, ai_freq_hz, 60.0)
                            else:
                                vfd_command_freq = max(vfd_command_freq - 0.5, ai_freq_hz, 0.0)
                        else:
                            vfd_command_freq = ai_freq_hz
                except:
                    pass  # AI 주파수 읽기 실패 시 현재 주파수 유지

            # ================================================================
            # 2단계: VFD 시뮬레이션 - 명령을 받아 실제 모터 제어 후 피드백
            # ================================================================
            # 실제 환경: VFD가 PLC 명령을 받아 모터 제어 → 실제 주파수를 PLC로 피드백
            # 시뮬레이터: 명령값에 약간의 오차 추가 (±0.3Hz, 실제 측정 오차 반영)
            if running:
                vfd_actual_freq = vfd_command_freq + random.uniform(-0.3, 0.3)
                vfd_actual_freq = max(0.0, min(60.0, vfd_actual_freq))  # 0-60Hz 범위
            else:
                vfd_actual_freq = 0.0

            # 명령 주파수를 내부 상태에 저장 (다음 cycle 명령 생성 시 기준)
            eq['hz'] = vfd_command_freq

            # ================================================================
            # 3단계: VFD 피드백을 PLC 레지스터 160-239에 저장 (HMI 표시용)
            # ================================================================
            # Frequency (Hz * 10) - VFD가 피드백한 실제 주파수
            frequency = self.hz_to_raw(vfd_actual_freq)

            # Power (kW) - VFD 실제 주파수 기반 전력
            if running:
                power = int(vfd_actual_freq * 0.8 + random.uniform(0, 5))
            else:
                power = 0

            # Edge AI가 계산한 절감량 읽기 (레지스터 5100-5109)
            try:
                savings_kw_raw = self.store.getValues(3, 5100 + i, 1)[0]
                savings_kw = savings_kw_raw / 10.0  # kW × 10 → kW
                # kW를 kWh로 변환 (1초마다 업데이트이므로 / 3600)
                savings = int(savings_kw * 1000)  # 임시로 kW를 정수로 저장
            except:
                savings = 0

            # Edge AI가 계산한 절감률 읽기 (레지스터 5300-5303)
            try:
                if i < 3:  # SWP
                    savings_ratio_raw = self.store.getValues(3, 5301, 1)[0]
                elif i < 6:  # FWP
                    savings_ratio_raw = self.store.getValues(3, 5302, 1)[0]
                else:  # FAN
                    savings_ratio_raw = self.store.getValues(3, 5303, 1)[0]
                savings_ratio = savings_ratio_raw / 10  # % × 10 → %
            except:
                savings_ratio = 0

            # Run Hours - 실제 운전 시간 (누적)
            current_hours = self.store.getValues(3, start_addr + 6, 1)[0]
            if running:
                run_hours = current_hours + 1  # 1초마다 1씩 증가
            else:
                run_hours = current_hours

            # Data: [Frequency, Power, AvgPower, Savings_L, Savings_H, Ratio, Hours_L, Hours_H]
            vfd_data = [
                frequency,              # Hz * 10 (Edge AI 목표 주파수 반영됨)
                power,                  # kW
                power,                  # Avg kW
                savings & 0xFFFF,       # Savings Low Word (Edge AI 값)
                (savings >> 16) & 0xFFFF,  # Savings High Word (Edge AI 값)
                int(savings_ratio),     # Savings Ratio % (Edge AI 값)
                run_hours & 0xFFFF,     # Run Hours Low
                (run_hours >> 16) & 0xFFFF  # Run Hours High
            ]

            self.store.setValues(3, start_addr, vfd_data)

        # AUTO/MANUAL, VFD/BYPASS 코일 업데이트
        for i, (eq_name, _) in enumerate(vfd_configs):
            eq = self.equipment[eq_name]
            # AUTO/MANUAL 코일 (64160 + eq_index)
            auto_coil_addr = 64160 + i
            self.store.setValues(1, auto_coil_addr, [1 if eq.get('auto_mode', True) else 0])

            # VFD/BYPASS 코일 (64320 + eq_index)
            vfd_coil_addr = 64320 + i
            self.store.setValues(1, vfd_coil_addr, [1 if eq.get('vfd_mode', True) else 0])

    def monitor_commands(self):
        """PLC 명령 모니터링 (HMI에서 전송하는 명령 처리)"""
        print("[시작] 명령 모니터링 스레드")

        # 장비 인덱스 맵
        equipment_names = ['SWP1', 'SWP2', 'SWP3', 'FWP1', 'FWP2', 'FWP3',
                          'FAN1', 'FAN2', 'FAN3', 'FAN4']

        while self.running:
            try:
                # HMI modbus_client.py의 코일 주소 매핑:
                # START 코일: 64064 + (eq_index * 2)
                # STOP 코일: 64064 + (eq_index * 2) + 1
                # FAN BWD 코일: 64084 + (fan_index - 6)
                # AUTO/MANUAL 코일: 64160 + eq_index (True=AUTO, False=MANUAL)
                # VFD/BYPASS 코일: 64320 + eq_index (True=VFD, False=BYPASS)

                for i, eq_name in enumerate(equipment_names):
                    # START 코일 확인
                    start_coil_addr = 64064 + (i * 2)
                    start_coil_value = self.store.getValues(1, start_coil_addr, 1)[0]

                    # STOP 코일 확인
                    stop_coil_addr = 64064 + (i * 2) + 1
                    stop_coil_value = self.store.getValues(1, stop_coil_addr, 1)[0]

                    # Pump 장비 (SWP1~3, FWP1~3)
                    if i < 6:
                        if start_coil_value:
                            # START 명령
                            if not self.equipment[eq_name]['running']:
                                self.equipment[eq_name]['running'] = True
                                self.equipment[eq_name]['ess_on'] = True
                                self.equipment[eq_name]['hz'] = 45.0 + random.uniform(-2, 2)
                                print(f"[제어] {eq_name} START 명령 수신 → 운전 시작 ({self.equipment[eq_name]['hz']:.1f} Hz)")
                            # 코일 리셋
                            self.store.setValues(1, start_coil_addr, [0])

                        if stop_coil_value:
                            # STOP 명령
                            if self.equipment[eq_name]['running']:
                                self.equipment[eq_name]['running'] = False
                                self.equipment[eq_name]['ess_on'] = False
                                self.equipment[eq_name]['hz'] = 0.0
                                print(f"[제어] {eq_name} STOP 명령 수신 → 운전 정지")
                            # 코일 리셋
                            self.store.setValues(1, stop_coil_addr, [0])

                    # Fan 장비 (FAN1~4)
                    else:
                        # FWD START
                        if start_coil_value:
                            if not self.equipment[eq_name]['running_fwd']:
                                self.equipment[eq_name]['running_fwd'] = True
                                self.equipment[eq_name]['running_bwd'] = False
                                self.equipment[eq_name]['hz'] = 45.0 + random.uniform(-2, 2)
                                print(f"[제어] {eq_name} FWD START 명령 수신 → 정방향 운전 시작 ({self.equipment[eq_name]['hz']:.1f} Hz)")
                            # 코일 리셋
                            self.store.setValues(1, start_coil_addr, [0])

                        # STOP
                        if stop_coil_value:
                            if self.equipment[eq_name]['running_fwd'] or self.equipment[eq_name]['running_bwd']:
                                self.equipment[eq_name]['running_fwd'] = False
                                self.equipment[eq_name]['running_bwd'] = False
                                self.equipment[eq_name]['hz'] = 0.0
                                print(f"[제어] {eq_name} STOP 명령 수신 → 운전 정지")
                            # 코일 리셋
                            self.store.setValues(1, stop_coil_addr, [0])

                        # BWD START (Fan only)
                        bwd_coil_addr = 64084 + (i - 6)
                        bwd_coil_value = self.store.getValues(1, bwd_coil_addr, 1)[0]
                        if bwd_coil_value:
                            if not self.equipment[eq_name]['running_bwd']:
                                self.equipment[eq_name]['running_fwd'] = False
                                self.equipment[eq_name]['running_bwd'] = True
                                self.equipment[eq_name]['hz'] = 45.0 + random.uniform(-2, 2)
                                print(f"[제어] {eq_name} BWD START 명령 수신 → 역방향 운전 시작 ({self.equipment[eq_name]['hz']:.1f} Hz)")
                            # 코일 리셋
                            self.store.setValues(1, bwd_coil_addr, [0])

                    # AUTO/MANUAL 모드 확인 (모든 장비 공통)
                    auto_coil_addr = 64160 + i
                    auto_coil_value = self.store.getValues(1, auto_coil_addr, 1)[0]
                    # 코일 값이 변경되었는지 확인
                    new_auto_mode = bool(auto_coil_value)
                    if self.equipment[eq_name]['auto_mode'] != new_auto_mode:
                        self.equipment[eq_name]['auto_mode'] = new_auto_mode
                        mode_str = "AUTO" if new_auto_mode else "MANUAL"
                        print(f"[제어] {eq_name} {mode_str} 모드 설정")

                    # VFD/BYPASS 모드 확인 (모든 장비 공통)
                    vfd_coil_addr = 64320 + i
                    vfd_coil_value = self.store.getValues(1, vfd_coil_addr, 1)[0]
                    # 코일 값이 변경되었는지 확인
                    new_vfd_mode = bool(vfd_coil_value)
                    if self.equipment[eq_name]['vfd_mode'] != new_vfd_mode:
                        self.equipment[eq_name]['vfd_mode'] = new_vfd_mode
                        mode_str = "VFD" if new_vfd_mode else "BYPASS"
                        print(f"[제어] {eq_name} {mode_str} 모드 설정")

                time.sleep(0.1)  # 100ms마다 체크

            except Exception as e:
                print(f"[ERROR] 명령 모니터링 오류: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1)

    def print_status(self):
        """주기적으로 시뮬레이터 상태 출력"""
        while self.running:
            try:
                time.sleep(10)
                print(f"\n[상태] {datetime.now().strftime('%H:%M:%S')}")
                print(f"  운전 중: SWP1={self.equipment['SWP1']['running']}, "
                      f"SWP2={self.equipment['SWP2']['running']}, "
                      f"FWP1={self.equipment['FWP1']['running']}, "
                      f"FWP2={self.equipment['FWP2']['running']}, "
                      f"FAN1={self.equipment['FAN1']['running_fwd']}, "
                      f"FAN2={self.equipment['FAN2']['running_fwd']}")
                print(f"  온도: TX1={self.base_temps['TX1']:.1f}°C, "
                      f"TX6={self.base_temps['TX6']:.1f}°C")
                print(f"  압력: DPX1={self.base_pressure['DPX1']:.2f} kg/cm², "
                      f"DPX2={self.base_pressure['DPX2']:.1f} Pa")
            except Exception as e:
                print(f"[ERROR] 상태 출력 오류: {e}")

    def start(self):
        """시뮬레이터 시작"""
        # 백그라운드 스레드 시작
        sensor_thread = threading.Thread(target=self.simulate_sensor_values, daemon=True)
        command_thread = threading.Thread(target=self.monitor_commands, daemon=True)
        status_thread = threading.Thread(target=self.print_status, daemon=True)

        sensor_thread.start()
        command_thread.start()
        status_thread.start()

        # Modbus TCP 서버 시작
        context = ModbusServerContext(slaves={3: self.store}, single=False)

        # 서버 식별 정보
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'OMTech'
        identity.ProductCode = 'ESS-HMI'
        identity.VendorUrl = 'http://www.omtech.com'
        identity.ProductName = 'ESS PLC Simulator'
        identity.ModelName = 'ESS-SIM-001'
        identity.MajorMinorRevision = '1.0.0'

        print("\n[시작] Modbus TCP 서버 구동 중...")
        print("[INFO] HMI에서 192.168.0.130:502 (Node ID: 3) 으로 연결하세요")
        print("[INFO] 종료: Ctrl+C\n")

        try:
            StartTcpServer(
                context=context,
                identity=identity,
                address=("0.0.0.0", 502)
            )
        except KeyboardInterrupt:
            print("\n[종료] 사용자가 중단했습니다")
            self.running = False
        except Exception as e:
            print(f"\n[ERROR] 서버 오류: {e}")
            self.running = False


if __name__ == "__main__":
    simulator = ESSPLCSimulator()
    simulator.start()

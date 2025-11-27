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

        # 알람 시나리오 카운터
        self.alarm_scenario_counter = 0
        self.alarm_active = False
        self.alarm_duration = 0  # 알람 지속 시간

        # 센서 동적 변동을 위한 사이클 변수
        self.simulation_tick = 0  # 1초마다 증가

        # 온도 사이클 상태 (사인파 기반 변동)
        # TX4: FWP 제어용 (40~50°C 사이클, 주기 120초)
        # TX5: SWP 제어용 (32~40°C 사이클, 주기 90초)
        # TX6: FAN 대수제어 테스트용 (38~48°C, 주기 180초)
        #      - 38~42°C (60초): AI가 주파수 감소 → 41Hz 도달 → 대수 감소 (4→3→2대)
        #      - 42~44°C (60초): 정상 범위, 대수 유지
        #      - 44~48°C (60초): AI가 주파수 증가 → 59Hz 도달 → 대수 증가 (2→3→4대)
        self.temp_cycle = {
            'TX4': {'min': 43.0, 'max': 47.0, 'period': 180, 'phase': 0},
            'TX5': {'min': 33.0, 'max': 37.0, 'period': 180, 'phase': 60},
            'TX6': {'min': 38.0, 'max': 48.0, 'period': 180, 'phase': 0},
        }

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
            'PX1': 2.0,  # SW DISCHARGE PRESS: 1.0~3.0 BAR
        }

        # M/E Load 사이클 (펌프 대수제어 테스트용)
        # 15~45% (90초): 펌프 1대 (< 30%)
        # 45~15% (90초): 펌프 2대 (≥ 30%)
        self.me_load_cycle = {'min': 15.0, 'max': 45.0, 'period': 180, 'phase': 0}
        self.me_load = 30.0  # 초기값

        # Edge AI 결과 저장 레지스터 초기화
        # 5000-5009: AI 목표 주파수 (Hz × 10)
        # 5100-5109: 절감 전력 (kW × 10)
        # 5200-5209: VFD 진단 점수 (0-100)
        # 5300-5303: 시스템 절감률 (% × 10)
        self.store.setValues(3, 5000, [0] * 10)  # AI 목표 주파수
        self.store.setValues(3, 5100, [0] * 10)  # 절감 전력
        self.store.setValues(3, 5200, [100] * 10)  # VFD 진단 점수 (초기값 100=정상)
        self.store.setValues(3, 5300, [0] * 4)   # 시스템 절감률

        # 알람 시스템 레지스터 초기화
        # 7000-7009: 알람 임계값 설정 (HMI → PLC)
        default_thresholds = [
            300,  # TX1: 30.0°C × 10
            500,  # TX2: 50.0°C × 10
            500,  # TX3: 50.0°C × 10
            500,  # TX4: 50.0°C × 10
            400,  # TX5: 40.0°C × 10
            500,  # TX6: 50.0°C × 10
            400,  # TX7: 40.0°C × 10
            150,  # PX1 하한: 1.5 bar × 100
            400,  # PX1 상한: 4.0 bar × 100
            850,  # PU1 상한: 85.0% × 10
        ]
        self.store.setValues(3, 7000, default_thresholds)

        # 7100-7103: 알람 상태 (PLC → HMI)
        self.store.setValues(3, 7100, [0, 0, 0, 0])  # [온도비트, 압력비트, 미확인개수, 새알람플래그]

        # 7200-7279: 최근 알람 10개 (순환 버퍼)
        self.store.setValues(3, 7200, [0] * 80)

        # 알람 관리
        self.recent_alarms = []  # 최근 알람 10개
        self.alarm_index = 0

        print("[OK] 데이터 스토어 초기화 완료")
        print("[INFO] Modbus TCP 서버: 192.168.0.130:502")
        print("[INFO] Node ID: 3")
        print("[INFO] Edge AI 결과 레지스터: 5000-5399 (Ready)")
        print("[INFO] 알람 시스템 레지스터: 7000-7279 (Ready)")
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

    def get_cyclic_temp(self, sensor_key):
        """사인파 기반 온도 사이클 계산"""
        import math
        cycle = self.temp_cycle[sensor_key]
        t = self.simulation_tick + cycle['phase']
        # 사인파: -1 ~ +1 → min ~ max 범위로 변환
        sine_value = math.sin(2 * math.pi * t / cycle['period'])
        mid = (cycle['max'] + cycle['min']) / 2
        amplitude = (cycle['max'] - cycle['min']) / 2
        return mid + amplitude * sine_value

    def simulate_sensor_values(self):
        """센서 값 실시간 시뮬레이션 (동적 변동 + 주기적 알람)"""
        print("[시작] 센서 데이터 시뮬레이션 스레드")
        print("[INFO] 대수제어 테스트 모드 활성화:")
        print()
        print("  📊 FAN 대수제어 (TX6 기반):")
        print("    - TX6: 38~48°C, 주기 180초")
        print("    - 38~42°C (60초): AI 주파수 감소 → 41Hz 도달 → FAN 대수 감소 (4→3→2대)")
        print("    - 42~44°C (60초): 정상 범위 → 대수 유지")
        print("    - 44~48°C (60초): AI 주파수 증가 → 59Hz 도달 → FAN 대수 증가 (2→3→4대)")
        print()
        print("  📊 펌프 대수제어 (PU1 기반):")
        print("    - PU1 (M/E Load): 15~45%, 주기 180초")
        print("    - < 30% (90초): SWP/FWP 각 1대")
        print("    - ≥ 30% (90초): SWP/FWP 각 2대")
        print()
        print("  📊 주파수 제어:")
        print("    - TX4 (FWP): 43~47°C, 주기 180초")
        print("    - TX5 (SWP): 33~37°C, 주기 180초")
        print()
        print("[INFO] 알람 시나리오: 비활성화 (대수제어 집중 관찰)")

        while self.running:
            try:
                # 시뮬레이션 틱 증가
                self.simulation_tick += 1

                # ========================================
                # 알람 시나리오 관리 (90초마다 15초간 발생) ← 대수제어 테스트용 단축
                # ========================================
                self.alarm_scenario_counter += 1

                # 알람 시작 조건: 90초 경과 후
                if not self.alarm_active and self.alarm_scenario_counter >= 90:
                    self.alarm_active = True
                    self.alarm_duration = 0
                    self.alarm_scenario_counter = 0
                    print("=" * 70)
                    print(f"[시뮬레이터] 🔔 알람 시나리오 시작 (15초간 유지) ⚡ 테스트 모드")
                    print("  - 🔴 TX6 E/R 내부 온도 상승: 52°C (임계값 50°C 초과)")
                    print("  - 🔴 TX7 외부 온도 상승: 42°C (임계값 40°C 초과)")
                    print("  - 🔴 PX1 압력 저하: 1.3 bar (임계값 1.5 bar 미만)")
                    print("  - 🔴 PU1 M/E 부하 과다: 90% (임계값 85% 초과)")
                    print("=" * 70)

                # 알람 종료 조건: 15초 경과 후
                if self.alarm_active:
                    self.alarm_duration += 1
                    if self.alarm_duration >= 15:
                        self.alarm_active = False
                        self.alarm_duration = 0
                        print("=" * 70)
                        print(f"[시뮬레이터] ✅ 알람 시나리오 종료 (정상 복귀)")
                        print("  다음 알람: 약 90초 후")
                        print("=" * 70)

                # ========================================
                # 물리 법칙 기반 온도 센서 시뮬레이션
                # ========================================

                # TX1 (COOLER SW INLET): 바닷물 온도 (여름: 22-26°C)
                tx1 = self.seawater_temp + random.uniform(-0.5, 0.5)

                # TX7 (E/R OUTSIDE): 외기 온도
                if self.alarm_active:
                    tx7 = 42.0 + random.uniform(-0.5, 0.5)  # 알람: 40°C 임계값 초과
                else:
                    tx7 = self.ambient_temp + random.uniform(-1.0, 1.0)

                # 현재 M/E 부하율에 따른 열부하 계산
                heat_load_factor = self.me_load / 60.0

                # TX2 (NO.1 COOLER SW OUTLET): TX1 + 열부하
                delta_t_sw_no1 = 8.0 * heat_load_factor
                tx2 = min(tx1 + delta_t_sw_no1 + random.uniform(-0.5, 0.5), 48.5)

                # TX3 (NO.2 COOLER SW OUTLET): TX1 + 열부하 (약간 낮음)
                delta_t_sw_no2 = 6.0 * heat_load_factor
                tx3 = min(tx1 + delta_t_sw_no2 + random.uniform(-0.5, 0.5), 48.5)

                # ========================================
                # 핵심: 동적 사이클 온도 (AI 목표주파수 변동 유발)
                # ========================================

                # TX5 (COOLER FW OUTLET): SWP 제어용 - 32~38°C 사이클
                # 목표 온도 35°C 기준, 상승 시 SWP 주파수 증가, 하강 시 감소
                if self.alarm_active:
                    tx5 = 42.0 + random.uniform(-0.5, 0.5)  # 알람: 높은 온도
                else:
                    tx5 = self.get_cyclic_temp('TX5') + random.uniform(-0.3, 0.3)

                # TX4 (COOLER FW INLET): FWP 제어용 - 40~46°C 사이클
                # 목표 온도 43°C 기준, 상승 시 FWP 주파수 증가
                if self.alarm_active:
                    tx4 = 48.0 + random.uniform(-0.5, 0.5)  # 알람: 높은 온도
                else:
                    tx4 = self.get_cyclic_temp('TX4') + random.uniform(-0.3, 0.3)

                # TX6 (E/R INSIDE): FAN 제어용 - 32~44°C 사이클
                # 목표 온도 38°C 기준, 상승 시 FAN 주파수 증가
                if self.alarm_active:
                    tx6 = 52.0 + random.uniform(-0.5, 0.5)  # 알람: 50°C 임계값 초과
                else:
                    tx6 = self.get_cyclic_temp('TX6') + random.uniform(-0.3, 0.3)

                # base_temps 업데이트 (상태 출력용)
                self.base_temps['TX4'] = tx4
                self.base_temps['TX5'] = tx5
                self.base_temps['TX6'] = tx6

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

                # ========================================
                # 압력 센서 (PX1)
                # ========================================
                if self.alarm_active:
                    px1 = 1.3 + random.uniform(-0.05, 0.05)  # 알람: 1.5 bar 임계값 미만
                else:
                    swp_running_count = sum([
                        1 for p in ['SWP1', 'SWP2', 'SWP3']
                        if self.equipment[p]['running']
                    ])
                    base_pressure = 1.0 + swp_running_count * 0.7
                    px1 = base_pressure + 0.3 * (self.me_load / 60.0) + random.uniform(-0.1, 0.1)
                    px1 = max(1.0, min(3.0, px1))

                self.store.setValues(3, 17, [
                    self.pressure_kgcm2_to_raw(px1)
                ])

                # ========================================
                # M/E Load (PU1) - 사이클 기반 (펌프 대수제어 테스트)
                # ========================================
                import math
                cycle = self.me_load_cycle
                t = self.simulation_tick + cycle['phase']
                sine_value = math.sin(2 * math.pi * t / cycle['period'])
                mid = (cycle['max'] + cycle['min']) / 2
                amplitude = (cycle['max'] - cycle['min']) / 2
                self.me_load = mid + amplitude * sine_value

                self.store.setValues(3, 19, [self.percentage_to_raw(self.me_load)])

                # 장비 상태 업데이트
                self.update_equipment_status()

                # VFD 데이터 업데이트
                self.update_vfd_data()

                # 알람 체크
                self.check_alarms()

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
                time.sleep(15)
                alarm_str = "🔴 알람 활성" if self.alarm_active else "✅ 정상"
                print(f"\n[상태] {datetime.now().strftime('%H:%M:%S')} | {alarm_str}")
                print(f"  장비: SWP1={self.equipment['SWP1']['running']}, "
                      f"SWP2={self.equipment['SWP2']['running']}, "
                      f"FWP1={self.equipment['FWP1']['running']}, "
                      f"FWP2={self.equipment['FWP2']['running']}, "
                      f"FAN1={self.equipment['FAN1']['running_fwd']}, "
                      f"FAN2={self.equipment['FAN2']['running_fwd']}")
                print(f"  동적온도: TX4={self.base_temps.get('TX4', 0):.1f}°C (FWP), "
                      f"TX5={self.base_temps.get('TX5', 0):.1f}°C (SWP), "
                      f"TX6={self.base_temps.get('TX6', 0):.1f}°C (FAN)")
                print(f"  주파수: SWP1={self.equipment['SWP1']['hz']:.1f}Hz, "
                      f"FWP1={self.equipment['FWP1']['hz']:.1f}Hz, "
                      f"FAN1={self.equipment['FAN1']['hz']:.1f}Hz")
                print(f"  M/E부하: {self.me_load:.1f}% | 다음알람: {300 - self.alarm_scenario_counter}초 후")
            except Exception as e:
                print(f"[ERROR] 상태 출력 오류: {e}")

    def check_alarms(self):
        """알람 체크 및 상태 업데이트"""

        try:
            # 임계값 읽기 (7000-7009)
            thresholds = self.store.getValues(3, 7000, 10)

            # 현재 센서값 읽기
            sensor_temps = self.store.getValues(3, 10, 7)  # TX1-TX7
            sensor_pressures = self.store.getValues(3, 17, 2)  # PX1, DPX2
            sensor_load = self.store.getValues(3, 19, 1)  # PU1

            alarm_bits_word0 = 0  # 온도 알람
            alarm_bits_word1 = 0  # 압력/부하 알람
            new_alarm_occurred = False

            # TX1-TX7 온도 체크
            for i in range(7):
                sensor_raw = sensor_temps[i]
                threshold_raw = thresholds[i]

                if sensor_raw > threshold_raw:  # 상한 초과
                    alarm_bits_word0 |= (1 << i)
                    self.add_recent_alarm(i+1, 1, sensor_raw, threshold_raw)
                    new_alarm_occurred = True

            # PX1 압력 체크 (DPX1을 bar로 변환)
            px1_raw = sensor_pressures[0]
            px1_bar = px1_raw / 4608.0  # raw → bar 변환
            px1_low_threshold = thresholds[7] / 100.0  # × 100 → bar
            px1_high_threshold = thresholds[8] / 100.0

            if px1_bar < px1_low_threshold:  # 하한 미만
                alarm_bits_word1 |= (1 << 0)
                self.add_recent_alarm(10, 2, int(px1_bar * 100), thresholds[7])
                new_alarm_occurred = True

            if px1_bar > px1_high_threshold:  # 상한 초과
                alarm_bits_word1 |= (1 << 1)
                self.add_recent_alarm(10, 1, int(px1_bar * 100), thresholds[8])
                new_alarm_occurred = True

            # PU1 부하 체크
            pu1_raw = sensor_load[0]
            pu1_percent = pu1_raw / 276.48  # raw → % 변환
            pu1_high_threshold = thresholds[9] / 10.0  # × 10 → %

            if pu1_percent > pu1_high_threshold:  # 상한 초과
                alarm_bits_word1 |= (1 << 2)
                self.add_recent_alarm(11, 1, int(pu1_percent * 10), thresholds[9])
                new_alarm_occurred = True

            # 알람 상태 레지스터 업데이트 (7100-7103)
            unack_count = len([a for a in self.recent_alarms if a['status'] == 0])
            new_alarm_flag = 1 if new_alarm_occurred else 0

            self.store.setValues(3, 7100, [alarm_bits_word0, alarm_bits_word1, unack_count, new_alarm_flag])

        except Exception as e:
            print(f"[ERROR] 알람 체크 오류: {e}")

    def add_recent_alarm(self, alarm_code, alarm_type, actual_value, threshold_value):
        """최근 알람에 추가 (10개 순환 버퍼)"""

        # 중복 체크
        for alarm in self.recent_alarms:
            if alarm['code'] == alarm_code and alarm['type'] == alarm_type and alarm['status'] == 0:
                return  # 이미 미확인 상태로 존재

        import time
        timestamp = int(time.time())

        alarm = {
            'code': alarm_code,
            'type': alarm_type,
            'actual': actual_value,
            'threshold': threshold_value,
            'timestamp': timestamp,
            'status': 0  # 0=미확인, 1=확인됨
        }

        # 순환 버퍼에 추가
        if len(self.recent_alarms) >= 10:
            self.recent_alarms.pop(0)  # 가장 오래된 것 제거
        self.recent_alarms.append(alarm)

        # PLC 레지스터에 쓰기
        self.write_recent_alarms_to_registers()

        sensor_names = {1: 'TX1', 2: 'TX2', 3: 'TX3', 4: 'TX4', 5: 'TX5', 6: 'TX6', 7: 'TX7', 10: 'PX1', 11: 'PU1'}
        sensor_name = sensor_names.get(alarm_code, f'CODE_{alarm_code}')
        alarm_type_str = '상한' if alarm_type == 1 else '하한'

        print(f"[PLC 알람] {sensor_name} {alarm_type_str} 초과!")

    def write_recent_alarms_to_registers(self):
        """최근 알람을 레지스터 7200-7279에 쓰기"""

        for i, alarm in enumerate(self.recent_alarms):
            if i >= 10:
                break

            start_addr = 7200 + (i * 8)
            timestamp_high = (alarm['timestamp'] >> 16) & 0xFFFF
            timestamp_low = alarm['timestamp'] & 0xFFFF

            alarm_data = [
                alarm['code'],
                alarm['type'],
                alarm['actual'],
                alarm['threshold'],
                timestamp_high,
                timestamp_low,
                alarm['status'],
                0  # 예약
            ]

            self.store.setValues(3, start_addr, alarm_data)

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

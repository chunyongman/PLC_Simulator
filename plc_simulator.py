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

# Windows 콘솔 인코딩 - Python 3.13에서는 기본 지원
# sys.stdout/stderr 래핑 제거 (호환성 문제)

try:
    # pymodbus 3.x
    from pymodbus.datastore import ModbusDeviceContext as ModbusSlaveContext
    from pymodbus.datastore import ModbusServerContext
    from pymodbus.datastore import ModbusSequentialDataBlock
    from pymodbus.server import StartTcpServer
    from pymodbus import ModbusDeviceIdentification
except ImportError as e:
    print(f"ERROR: pymodbus library import failed: {e}")
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
        self.current_alarm_sensors = []  # 현재 알람 발생 센서 (랜덤 2개)
        self.alarm_cycle_number = 0  # 알람 주기 번호 (랜덤 시드용)

        # ===== VFD 이상 시나리오 (예방진단 테스트용) =====
        self.vfd_anomaly_enabled = True  # VFD 이상 시나리오 활성화
        self.vfd_anomaly_counter = 0  # VFD 이상 시나리오 카운터
        self.vfd_anomaly_active = False  # VFD 이상 발생 중
        self.vfd_anomaly_duration = 0  # VFD 이상 지속 시간
        self.current_vfd_anomalies = []  # 현재 이상 발생 중인 VFD
        self.vfd_anomaly_cycle_number = 0  # VFD 이상 주기 번호

        # VFD 이상 유형 정의 (Edge Computer 임계값 기준으로 조정)
        # Edge 임계값: motor_thermal 80/90/100, heatsink_temp 60/70/80, inverter_thermal 80/90/100
        # 종합 점수: 0-2=정상, 3-5=주의, 6-8=경고, 9+=위험
        self.vfd_anomaly_types = {
            'motor_overheat': {
                'name': '모터 과열',
                'motor_thermal': (95, 110),   # 경고~위험 수준 (임계값 90/100 초과)
                'heatsink_temp': (72, 85),    # 경고 수준 (임계값 70/80)
                'inverter_thermal': (85, 95), # 주의~경고 수준
                'severity': 2  # 경고
            },
            'inverter_overheat': {
                'name': '인버터 과열',
                'inverter_thermal': (95, 115), # 경고~위험 수준
                'heatsink_temp': (75, 88),     # 경고~위험 수준
                'motor_thermal': (82, 92),     # 주의~경고 수준
                'severity': 2
            },
            'current_imbalance': {
                'name': '3상 전류 불균형',
                'phase_imbalance': (18, 35),  # 경고~위험 수준 (임계값 10/15/20)
                'motor_thermal': (83, 93),    # 주의~경고 동반
                'heatsink_temp': (62, 72),    # 주의~경고 동반
                'severity': 2
            },
            'overcurrent': {
                'name': '과전류',
                'current_ratio': (1.15, 1.35), # 정격 대비 115-135% (임계값 90/100/110)
                'motor_thermal': (98, 115),    # 경고~위험 수준
                'heatsink_temp': (74, 86),     # 경고~위험 동반
                'inverter_thermal': (88, 100), # 주의~경고 동반
                'severity': 3  # 위험
            },
            'high_load': {
                'name': '고부하 운전',
                'torque': (140, 180),         # 고토크
                'motor_thermal': (85, 98),    # 주의~경고 수준
                'heatsink_temp': (65, 75),    # 주의~경고 동반
                'severity': 1  # 주의
            },
            'dc_voltage_abnormal': {
                'name': 'DC 링크 전압 이상',
                'dc_link_voltage': (480, 520), # 저전압
                'motor_thermal': (82, 92),     # 동반 증상
                'inverter_thermal': (83, 93),  # 동반 증상
                'severity': 2
            }
        }

        # 알람 가능 센서 목록 및 알람값 정의
        self.alarm_sensors = {
            'TX1': {'name': '냉각수 토출 온도 상승', 'alarm_value': 32.0, 'threshold': 30.0, 'unit': '°C'},
            'TX2': {'name': 'NO.1 쿨러 출구 온도 상승', 'alarm_value': 52.0, 'threshold': 50.0, 'unit': '°C'},
            'TX3': {'name': 'NO.2 쿨러 출구 온도 상승', 'alarm_value': 52.0, 'threshold': 50.0, 'unit': '°C'},
            'TX4': {'name': '청수 입구 온도 상승', 'alarm_value': 52.0, 'threshold': 50.0, 'unit': '°C'},
            'TX5': {'name': '청수 출구 온도 상승', 'alarm_value': 42.0, 'threshold': 40.0, 'unit': '°C'},
            'TX6': {'name': 'E/R 내부 온도 상승', 'alarm_value': 52.0, 'threshold': 50.0, 'unit': '°C'},
            'TX7': {'name': 'E/R 외부 온도 상승', 'alarm_value': 42.0, 'threshold': 40.0, 'unit': '°C'},
            'PX1_LOW': {'name': '냉각수 압력 저하', 'alarm_value': 1.0, 'threshold': 1.2, 'unit': 'bar'},
            'PX1_HIGH': {'name': '냉각수 압력 과다', 'alarm_value': 4.2, 'threshold': 4.0, 'unit': 'bar'},
            'PU1': {'name': 'M/E 부하 과다', 'alarm_value': 90.0, 'threshold': 85.0, 'unit': '%'},
        }

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
        # 5400-5401: 누적 절감량 (kWh × 10) - 오늘/이번달
        # 5500-5503: 60Hz 고정 전력 (kW × 10) - total, swp, fwp, fan
        # 5510-5513: VFD 가변 전력 (kW × 10) - total, swp, fwp, fan
        # 5520-5523: 절감 전력 (kW × 10) - total, swp, fwp, fan
        self.store.setValues(3, 5000, [0] * 10)  # AI 목표 주파수
        self.store.setValues(3, 5100, [0] * 10)  # 절감 전력
        self.store.setValues(3, 5200, [100] * 10)  # VFD 진단 점수 (초기값 100=정상)
        self.store.setValues(3, 5300, [0] * 4)   # 시스템 절감률
        self.store.setValues(3, 5400, [0] * 2)   # 누적 절감량 (오늘/이번달)
        self.store.setValues(3, 5500, [0] * 4)   # 60Hz 고정 전력
        self.store.setValues(3, 5510, [0] * 4)   # VFD 가변 전력
        self.store.setValues(3, 5520, [0] * 4)   # 절감 전력

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
        print("[INFO] Edge AI 결과 레지스터: 5000-5523 (Ready)")
        print("[INFO] 알람 시스템 레지스터: 7000-7279 (Ready)")
        print("[INFO] 장비 상태 레지스터: 4000-4001 (Ready)")
        print("[INFO] VFD 데이터 레지스터: 160-359 (20 regs × 10 equip, 예방진단 포함)")
        print("-" * 70)
        print("[VFD 이상 시나리오] 활성화됨 (60초 정상 → 60초 이상)")
        print("  - 이상 유형: 모터 과열, 인버터 과열, 3상 전류 불균형,")
        print("              과전류, 고부하 운전, DC 링크 전압 이상")
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
        """센서 값 실시간 시뮬레이션 (동적 변동 + 랜덤 알람)"""
        print("[시작] 센서 데이터 시뮬레이션 스레드")
        print("[INFO] Random alarm system activated:")
        print()
        print("  [ALARM] Sensor alarm scenario:")
        print("    - 90s normal -> 15s alarm (random 2 sensors)")
        print("    - Target: TX1~TX7, PX1, PU1 (2 of 9)")
        print()
        print("  [VFD] VFD anomaly scenario (for preventive diagnosis):")
        print("    - 60s normal -> 60s anomaly (random 1-2 VFDs)")
        print("    - Types: motor_overheat, inverter_overheat, current_imbalance,")
        print("             overcurrent, high_load, dc_voltage_abnormal")
        print()
        print("  [TEMP] Dynamic temperature control:")
        print("    - TX4 (FWP): 43~47C, period 180s")
        print("    - TX5 (SWP): 33~37C, period 180s")
        print("    - TX6 (FAN): 38~48C, period 180s")
        print()

        while self.running:
            try:
                # 시뮬레이션 틱 증가
                self.simulation_tick += 1

                # ========================================
                # 랜덤 알람 시나리오 관리 (90초마다 15초간 발생)
                # ========================================
                self.alarm_scenario_counter += 1

                # 알람 시작 조건: 90초 경과 후
                if not self.alarm_active and self.alarm_scenario_counter >= 90:
                    self.alarm_active = True
                    self.alarm_duration = 0
                    self.alarm_scenario_counter = 0
                    self.alarm_cycle_number += 1

                    # 9개 센서 중 랜덤하게 2개 선택
                    all_sensors = list(self.alarm_sensors.keys())
                    random.seed(self.alarm_cycle_number)  # 같은 주기에서는 동일한 알람 유지
                    self.current_alarm_sensors = random.sample(all_sensors, 2)

                    print("=" * 70)
                    print(f"[SIM] ALARM triggered (15s) - cycle #{self.alarm_cycle_number}")
                    for sensor in self.current_alarm_sensors:
                        info = self.alarm_sensors[sensor]
                        print(f"  - [!] {info['name']}: {info['alarm_value']}{info['unit']} (threshold {info['threshold']}{info['unit']})")
                    print("=" * 70)

                # 알람 종료 조건: 15초 경과 후
                if self.alarm_active:
                    self.alarm_duration += 1
                    if self.alarm_duration >= 15:
                        self.alarm_active = False
                        self.alarm_duration = 0
                        self.current_alarm_sensors = []
                        print("=" * 70)
                        print(f"[SIM] ALARM scenario ended (back to normal)")
                        print("  Next alarm: ~90s later (random 2 sensors)")
                        print("=" * 70)

                # ========================================
                # VFD 이상 시나리오 관리 (60초 정상 -> 30초 이상)
                # ========================================
                if self.vfd_anomaly_enabled:
                    self.vfd_anomaly_counter += 1

                    # VFD 이상 시작 조건: 60초 경과 후
                    if not self.vfd_anomaly_active and self.vfd_anomaly_counter >= 60:
                        self.vfd_anomaly_active = True
                        self.vfd_anomaly_duration = 0
                        self.vfd_anomaly_counter = 0
                        self.vfd_anomaly_cycle_number += 1

                        # 랜덤하게 1-2개 VFD 선택 (운전 중인 것만)
                        running_vfds = [
                            name for name, eq in self.equipment.items()
                            if eq.get('running', False) or eq.get('running_fwd', False)
                        ]
                        if running_vfds:
                            random.seed(self.vfd_anomaly_cycle_number + 1000)
                            num_anomalies = random.randint(1, min(2, len(running_vfds)))
                            selected_vfds = random.sample(running_vfds, num_anomalies)

                            # 각 VFD에 랜덤 이상 유형 할당
                            anomaly_types = list(self.vfd_anomaly_types.keys())
                            self.current_vfd_anomalies = []
                            for vfd in selected_vfds:
                                anomaly_type = random.choice(anomaly_types)
                                self.current_vfd_anomalies.append({
                                    'vfd': vfd,
                                    'type': anomaly_type,
                                    'info': self.vfd_anomaly_types[anomaly_type]
                                })

                            print("=" * 70)
                            print(f"[VFD] ANOMALY triggered (30s) - cycle #{self.vfd_anomaly_cycle_number}")
                            for anomaly in self.current_vfd_anomalies:
                                severity_names = {1: '주의', 2: '경고', 3: '위험'}
                                severity = anomaly['info']['severity']
                                print(f"  - [!] {anomaly['vfd']}: {anomaly['info']['name']} (Severity: {severity_names[severity]})")
                            print("=" * 70)

                    # VFD 이상 종료 조건: 60초 경과 후 (확인할 시간 확보)
                    if self.vfd_anomaly_active:
                        self.vfd_anomaly_duration += 1
                        if self.vfd_anomaly_duration >= 60:
                            self.vfd_anomaly_active = False
                            self.vfd_anomaly_duration = 0
                            self.current_vfd_anomalies = []
                            print("=" * 70)
                            print(f"[VFD] ANOMALY scenario ended (back to normal)")
                            print("  Next VFD anomaly: ~60s later (random 1-2 VFDs)")
                            print("=" * 70)

                # ========================================
                # 물리 법칙 기반 온도 센서 시뮬레이션 (랜덤 알람 적용)
                # ========================================

                # 현재 M/E 부하율에 따른 열부하 계산
                heat_load_factor = self.me_load / 60.0

                # TX1 (COOLER SW INLET): 바닷물 온도 (여름: 22-26°C)
                if self.alarm_active and 'TX1' in self.current_alarm_sensors:
                    tx1 = self.alarm_sensors['TX1']['alarm_value'] + random.uniform(-0.5, 0.5)
                else:
                    tx1 = self.seawater_temp + random.uniform(-0.5, 0.5)

                # TX2 (NO.1 COOLER SW OUTLET): TX1 + 열부하
                if self.alarm_active and 'TX2' in self.current_alarm_sensors:
                    tx2 = self.alarm_sensors['TX2']['alarm_value'] + random.uniform(-0.5, 0.5)
                else:
                    delta_t_sw_no1 = 8.0 * heat_load_factor
                    tx2 = min(tx1 + delta_t_sw_no1 + random.uniform(-0.5, 0.5), 48.5)

                # TX3 (NO.2 COOLER SW OUTLET): TX1 + 열부하 (약간 낮음)
                if self.alarm_active and 'TX3' in self.current_alarm_sensors:
                    tx3 = self.alarm_sensors['TX3']['alarm_value'] + random.uniform(-0.5, 0.5)
                else:
                    delta_t_sw_no2 = 6.0 * heat_load_factor
                    tx3 = min(tx1 + delta_t_sw_no2 + random.uniform(-0.5, 0.5), 48.5)

                # TX4 (COOLER FW INLET): FWP 제어용 - 동적 사이클
                if self.alarm_active and 'TX4' in self.current_alarm_sensors:
                    tx4 = self.alarm_sensors['TX4']['alarm_value'] + random.uniform(-0.5, 0.5)
                else:
                    tx4 = self.get_cyclic_temp('TX4') + random.uniform(-0.3, 0.3)

                # TX5 (COOLER FW OUTLET): SWP 제어용 - 동적 사이클
                if self.alarm_active and 'TX5' in self.current_alarm_sensors:
                    tx5 = self.alarm_sensors['TX5']['alarm_value'] + random.uniform(-0.5, 0.5)
                else:
                    tx5 = self.get_cyclic_temp('TX5') + random.uniform(-0.3, 0.3)

                # TX6 (E/R INSIDE): FAN 제어용 - 동적 사이클
                if self.alarm_active and 'TX6' in self.current_alarm_sensors:
                    tx6 = self.alarm_sensors['TX6']['alarm_value'] + random.uniform(-0.5, 0.5)
                else:
                    tx6 = self.get_cyclic_temp('TX6') + random.uniform(-0.3, 0.3)

                # TX7 (E/R OUTSIDE): 외기 온도
                if self.alarm_active and 'TX7' in self.current_alarm_sensors:
                    tx7 = self.alarm_sensors['TX7']['alarm_value'] + random.uniform(-0.5, 0.5)
                else:
                    tx7 = self.ambient_temp + random.uniform(-1.0, 1.0)

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
                # 압력 센서 (PX1) - 랜덤 알람 적용 (하한/상한 분리)
                # ========================================
                if self.alarm_active and 'PX1_LOW' in self.current_alarm_sensors:
                    # 압력 저하 알람
                    px1 = self.alarm_sensors['PX1_LOW']['alarm_value'] + random.uniform(-0.05, 0.05)
                elif self.alarm_active and 'PX1_HIGH' in self.current_alarm_sensors:
                    # 압력 과다 알람
                    px1 = self.alarm_sensors['PX1_HIGH']['alarm_value'] + random.uniform(-0.05, 0.05)
                else:
                    # 정상 상태
                    swp_running_count = sum([
                        1 for p in ['SWP1', 'SWP2', 'SWP3']
                        if self.equipment[p]['running']
                    ])
                    base_pressure = 1.5 + swp_running_count * 0.5
                    px1 = base_pressure + 0.3 * (self.me_load / 60.0) + random.uniform(-0.1, 0.1)
                    px1 = max(1.5, min(3.5, px1))  # 정상 범위: 1.5~3.5 bar

                self.store.setValues(3, 17, [
                    self.pressure_kgcm2_to_raw(px1)
                ])

                # ========================================
                # M/E Load (PU1) - 사이클 기반 + 랜덤 알람 적용
                # ========================================
                import math
                if self.alarm_active and 'PU1' in self.current_alarm_sensors:
                    # 알람: 85% 임계값 초과
                    self.me_load = self.alarm_sensors['PU1']['alarm_value'] + random.uniform(-1, 1)
                else:
                    # 정상: 사이클 기반 (15~45%)
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
        """VFD 운전 데이터 업데이트 (확장: 장비당 20개 레지스터, 160~359)"""

        # SWP1~3, FWP1~3, FAN1~4 각 20개 레지스터
        vfd_configs = [
            ('SWP1', 160), ('SWP2', 180), ('SWP3', 200),
            ('FWP1', 220), ('FWP2', 240), ('FWP3', 260),
            ('FAN1', 280), ('FAN2', 300), ('FAN3', 320), ('FAN4', 340)
        ]

        # 장비별 정격 전류 (A)
        rated_currents = {
            'SWP1': 300, 'SWP2': 300, 'SWP3': 300,
            'FWP1': 370, 'FWP2': 370, 'FWP3': 370,
            'FAN1': 70, 'FAN2': 70, 'FAN3': 70, 'FAN4': 70
        }

        for i, (eq_name, start_addr) in enumerate(vfd_configs):
            eq = self.equipment[eq_name]
            running = eq.get('running', False) or eq.get('running_fwd', False) or eq.get('running_bwd', False)
            vfd_command_freq = eq['hz']
            auto_mode = eq.get('auto_mode', True)
            vfd_mode = eq.get('vfd_mode', True)

            if auto_mode and vfd_mode and running:
                try:
                    ai_freq_raw = self.store.getValues(3, 5000 + i, 1)[0]
                    if ai_freq_raw > 0:
                        ai_freq_hz = ai_freq_raw / 10.0
                        if abs(ai_freq_hz - vfd_command_freq) > 0.5:
                            if ai_freq_hz > vfd_command_freq:
                                vfd_command_freq = min(vfd_command_freq + 0.5, ai_freq_hz, 60.0)
                            else:
                                vfd_command_freq = max(vfd_command_freq - 0.5, ai_freq_hz, 0.0)
                        else:
                            vfd_command_freq = ai_freq_hz
                except:
                    pass

            if running:
                vfd_actual_freq = vfd_command_freq + random.uniform(-0.3, 0.3)
                vfd_actual_freq = max(0.0, min(60.0, vfd_actual_freq))
            else:
                vfd_actual_freq = 0.0

            eq['hz'] = vfd_command_freq

            # === VFD 데이터 (20개 레지스터) ===
            frequency = self.hz_to_raw(vfd_actual_freq)
            try:
                power_raw = self.store.getValues(3, 5620 + i, 1)[0]
                power = power_raw // 10
            except:
                power = 0
            avg_power = power
            rated_current = rated_currents[eq_name]

            # 예방진단 데이터 시뮬레이션
            # VFD 이상 시나리오 확인
            current_anomaly = None
            if self.vfd_anomaly_active:
                for anomaly in self.current_vfd_anomalies:
                    if anomaly['vfd'] == eq_name:
                        current_anomaly = anomaly
                        break

            if running:
                # 기본 정상 값
                motor_current = int(rated_current * random.uniform(0.70, 0.85) * 10)
                motor_thermal = random.randint(50, 75)
                heatsink_temp = random.randint(40, 55)
                torque = int(vfd_actual_freq * 2 + random.uniform(-5, 5))
                inverter_thermal = random.randint(45, 70)
                system_temp = random.randint(35, 50)
                base_phase = motor_current / 10 / 1.732
                phase_u = int((base_phase + random.uniform(-2, 2)) * 10)
                phase_v = int((base_phase + random.uniform(-2, 2)) * 10)
                phase_w = int((base_phase + random.uniform(-2, 2)) * 10)
                dc_link_voltage = random.randint(540, 560)

                # ===== VFD 이상 시나리오 적용 =====
                if current_anomaly:
                    anomaly_type = current_anomaly['type']
                    anomaly_info = current_anomaly['info']

                    if anomaly_type == 'motor_overheat':
                        # 모터 과열: motor_thermal, heatsink_temp, inverter_thermal 모두 상승
                        motor_thermal = random.randint(*anomaly_info['motor_thermal'])
                        heatsink_temp = random.randint(*anomaly_info['heatsink_temp'])
                        inverter_thermal = random.randint(*anomaly_info['inverter_thermal'])
                        system_temp = random.randint(55, 70)

                    elif anomaly_type == 'inverter_overheat':
                        # 인버터 과열: inverter_thermal, heatsink_temp, motor_thermal 모두 상승
                        inverter_thermal = random.randint(*anomaly_info['inverter_thermal'])
                        heatsink_temp = random.randint(*anomaly_info['heatsink_temp'])
                        motor_thermal = random.randint(*anomaly_info['motor_thermal'])
                        system_temp = random.randint(58, 72)

                    elif anomaly_type == 'current_imbalance':
                        # 3상 전류 불균형 + motor_thermal, heatsink_temp 동반 상승
                        imbalance_pct = random.uniform(*anomaly_info['phase_imbalance']) / 100
                        base_phase = motor_current / 10 / 1.732
                        phase_u = int((base_phase * (1 + imbalance_pct)) * 10)
                        phase_v = int((base_phase * (1 - imbalance_pct * 0.5)) * 10)
                        phase_w = int((base_phase * (1 - imbalance_pct * 0.5)) * 10)
                        motor_thermal = random.randint(*anomaly_info['motor_thermal'])
                        heatsink_temp = random.randint(*anomaly_info['heatsink_temp'])

                    elif anomaly_type == 'overcurrent':
                        # 과전류: motor_current, motor_thermal, heatsink_temp, inverter_thermal 모두 상승
                        current_ratio = random.uniform(*anomaly_info['current_ratio'])
                        motor_current = int(rated_current * current_ratio * 10)
                        motor_thermal = random.randint(*anomaly_info['motor_thermal'])
                        heatsink_temp = random.randint(*anomaly_info['heatsink_temp'])
                        inverter_thermal = random.randint(*anomaly_info['inverter_thermal'])
                        # 3상 전류도 상승
                        base_phase = motor_current / 10 / 1.732
                        phase_u = int((base_phase + random.uniform(-3, 3)) * 10)
                        phase_v = int((base_phase + random.uniform(-3, 3)) * 10)
                        phase_w = int((base_phase + random.uniform(-3, 3)) * 10)
                        system_temp = random.randint(60, 75)

                    elif anomaly_type == 'high_load':
                        # 고부하 운전: torque, motor_thermal, heatsink_temp 상승
                        torque = random.randint(*anomaly_info['torque'])
                        motor_thermal = random.randint(*anomaly_info['motor_thermal'])
                        heatsink_temp = random.randint(*anomaly_info['heatsink_temp'])

                    elif anomaly_type == 'dc_voltage_abnormal':
                        # DC 링크 전압 이상 + motor_thermal, inverter_thermal 동반 상승
                        dc_link_voltage = random.randint(*anomaly_info['dc_link_voltage'])
                        motor_thermal = random.randint(*anomaly_info['motor_thermal'])
                        inverter_thermal = random.randint(*anomaly_info['inverter_thermal'])

            else:
                motor_current = 0
                motor_thermal = 0
                heatsink_temp = 25
                torque = 0
                inverter_thermal = 0
                system_temp = 25
                phase_u = phase_v = phase_w = 0
                dc_link_voltage = 0

            # 누적 데이터
            try:
                kwh_lo = self.store.getValues(3, start_addr + 9, 1)[0]
                kwh_hi = self.store.getValues(3, start_addr + 10, 1)[0]
                kwh_counter = (kwh_hi << 16) | kwh_lo
                hours_lo = self.store.getValues(3, start_addr + 18, 1)[0]
                hours_hi = self.store.getValues(3, start_addr + 19, 1)[0]
                run_hours = (hours_hi << 16) | hours_lo
            except:
                kwh_counter = 0
                run_hours = 0
            if running:
                kwh_counter += 1
                run_hours += 1

            try:
                num_starts = self.store.getValues(3, start_addr + 11, 1)[0]
                if num_starts == 0:
                    num_starts = random.randint(100, 500)
            except:
                num_starts = random.randint(100, 500)

            # 20개 레지스터 데이터
            vfd_data = [
                frequency, power, avg_power, motor_current, motor_thermal,
                heatsink_temp, torque, inverter_thermal, system_temp,
                kwh_counter & 0xFFFF, (kwh_counter >> 16) & 0xFFFF,
                num_starts, 0, phase_u, phase_v, phase_w, 0, dc_link_voltage,
                run_hours & 0xFFFF, (run_hours >> 16) & 0xFFFF
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
                alarm_str = "[!] ALARM ACTIVE" if self.alarm_active else "[OK] Normal"
                vfd_str = "[!] VFD ANOMALY" if self.vfd_anomaly_active else "[OK] VFD Normal"
                print(f"\n[상태] {datetime.now().strftime('%H:%M:%S')} | {alarm_str} | {vfd_str}")
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
                print(f"  M/E부하: {self.me_load:.1f}% | 다음알람: {90 - self.alarm_scenario_counter}초 후")

                # VFD 이상 상태 출력
                if self.vfd_anomaly_enabled:
                    if self.vfd_anomaly_active:
                        anomaly_names = [f"{a['vfd']}({a['info']['name']})" for a in self.current_vfd_anomalies]
                        print(f"  [VFD 이상] {', '.join(anomaly_names)} | 남은시간: {60 - self.vfd_anomaly_duration}초")
                    else:
                        print(f"  [VFD 정상] 다음 이상 발생: {60 - self.vfd_anomaly_counter}초 후")
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

        # Modbus TCP 서버 시작 (pymodbus 3.x: slaves → devices)
        context = ModbusServerContext(devices={3: self.store}, single=False)

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

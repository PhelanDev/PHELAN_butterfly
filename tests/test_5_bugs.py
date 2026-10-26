import os
import re
import sys
import subprocess

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def read_file(rel_path):
    path = os.path.join(ROOT_DIR, rel_path)
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        return f.read()

# Audit 1: CRSF framing flow & Type 0x14 lock
def audit_bug1(content):
    # Check A: crsfIndex == 2 has continue after crsfIndex = 1
    has_continue_at_idx2 = bool(re.search(
        r'if\s*\(\s*(?:state|m_state)\.crsfIndex\s*==\s*2\s*\)[\s\S]*?'
        r'if\s*\(\s*c\s*==\s*0xC8\s*\)\s*\{[\s\S]*?'
        r'(?:state|m_state)\.crsfIndex\s*=\s*1;[\s\S]*?continue;',
        content
    ))
    # Check B: crsfIndex == 3 enforces type == 0x14 && crsfLength == 12
    has_type14_lock = bool(re.search(
        r'type\s*==\s*0x14\s*&&\s*(?:state|m_state)\.crsfLength\s*==\s*12',
        content
    ))
    return has_continue_at_idx2 and has_type14_lock

# Audit 2: Compile-time Servo Limit Constants
def audit_bug2(cfg_content, servo_content):
    has_lim_min = bool(re.search(r'constexpr\s+int\s+SERVO_US_LIM_MIN\s*=', cfg_content))
    has_lim_max = bool(re.search(r'constexpr\s+int\s+SERVO_US_LIM_MAX\s*=', cfg_content))
    uses_lim_in_update = (
        'SERVO_US_LIM_MIN' in servo_content and
        'SERVO_US_LIM_MAX' in servo_content and
        'angleToUs((float)Config::SERVO_MIN_ANGLE)' not in servo_content
    )
    return has_lim_min and has_lim_max and uses_lim_in_update

# Audit 3: Unified phase and flap direction order
def audit_bug3(flight_content):
    # Look for updateCurrentOffsetTriangle or updateCurrentOffsetSine
    # Order: phase += phaseDelta -> isFlapUp = isRisingPhase -> currentOffset = LUT
    match = re.search(
        r'phase\s*\+=\s*phaseDelta;[\s\S]*?'
        r'isFlapUp\s*=\s*isRisingPhase\((?:state|m_state)\.phase\);[\s\S]*?'
        r'currentOffset\s*=\s*\(?\(?float\)?\(?[a-zA-Z0-9_]+LUT',
        flight_content
    )
    return bool(match)

# Audit 4: Remove dead switchDelayStart
def audit_bug4(content):
    return 'switchDelayStart' not in content

# Audit 5: Servo kick prevention on Arm/Disarm/Kill
def audit_bug5(ino_content):
    # handleDisarming has currentOffset = 0.0f
    has_disarm_zero = bool(re.search(
        r'void\s+handleDisarming[\s\S]*?Flight::state\.currentOffset\s*=\s*0\.0f;',
        ino_content
    ))
    # killDebounce >= 5 has currentOffset = 0.0f
    has_kill_zero = bool(re.search(
        r'killDebounce\s*>=\s*5[\s\S]*?Flight::state\.currentOffset\s*=\s*0\.0f;',
        ino_content
    ))
    # handleInitMode arming has currentOffset = 0.0f, lastUsL/R, filteredL/R before attachAll
    has_arm_sync = bool(re.search(
        r'Flight::state\.currentOffset\s*=\s*0\.0f;[\s\S]*?'
        r'ServoSystem::state\.lastUsL\s*=\s*Config::SERVO_US_CENTER;[\s\S]*?'
        r'ServoSystem::state\.filteredL\s*=\s*\(?int32_t\)?Config::SERVO_US_CENTER\s*<<\s*Config::SERVO_EMA_ALPHA_SHIFT;[\s\S]*?'
        r'ServoSystem::attachAll\(\);',
        ino_content
    ))
    return has_disarm_zero and has_kill_zero and has_arm_sync

def run_audits():
    variants = [
        {
            'name': 'single_tria',
            'main': 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino',
            'cfg': 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino',
            'servo': 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino',
            'flight': 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino',
            'rc': 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino',
        },
        {
            'name': 'single_sin',
            'main': 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino',
            'cfg': 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino',
            'servo': 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino',
            'flight': 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino',
            'rc': 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino',
        },
        {
            'name': 'multi_tria',
            'main': 'butterfly_esp32c3mini_Phelan_tria/butterfly_esp32c3mini_Phelan_tria.ino',
            'cfg': 'butterfly_esp32c3mini_Phelan_tria/config.h',
            'servo': 'butterfly_esp32c3mini_Phelan_tria/servo_sys.cpp',
            'flight': 'butterfly_esp32c3mini_Phelan_tria/flight.cpp',
            'rc': 'butterfly_esp32c3mini_Phelan_tria/receiver.cpp',
        },
        {
            'name': 'multi_sin',
            'main': 'butterfly_esp32c3mini_Phelan_sin/butterfly_esp32c3mini_Phelan_sin.ino',
            'cfg': 'butterfly_esp32c3mini_Phelan_sin/config.h',
            'servo': 'butterfly_esp32c3mini_Phelan_sin/servo_sys.cpp',
            'flight': 'butterfly_esp32c3mini_Phelan_sin/flight.cpp',
            'rc': 'butterfly_esp32c3mini_Phelan_sin/receiver.cpp',
        },
    ]

    passed = 0
    failed = 0
    failed_details = []

    print("\n=== Static Audits for 5 Advanced Firmware Bugs ===")
    for v in variants:
        main_c = read_file(v['main'])
        cfg_c = read_file(v['cfg'])
        servo_c = read_file(v['servo'])
        flight_c = read_file(v['flight'])
        rc_c = read_file(v['rc'])

        # Bug 1
        b1 = audit_bug1(rc_c)
        if b1:
            passed += 1
            print(f"  [PASS] {v['name']}: Bug 1 - CRSF framing continue & Type 0x14 lock")
        else:
            failed += 1
            failed_details.append(f"{v['name']}: Bug 1 - CRSF framing continue & Type 0x14 lock")
            print(f"  [FAIL] {v['name']}: Bug 1 - CRSF framing continue & Type 0x14 lock")

        # Bug 2
        b2 = audit_bug2(cfg_c, servo_c)
        if b2:
            passed += 1
            print(f"  [PASS] {v['name']}: Bug 2 - Compile-time SERVO_US_LIM constants")
        else:
            failed += 1
            failed_details.append(f"{v['name']}: Bug 2 - Compile-time SERVO_US_LIM constants")
            print(f"  [FAIL] {v['name']}: Bug 2 - Compile-time SERVO_US_LIM constants")

        # Bug 3
        b3 = audit_bug3(flight_c)
        if b3:
            passed += 1
            print(f"  [PASS] {v['name']}: Bug 3 - Phase -> isFlapUp -> LUT order")
        else:
            failed += 1
            failed_details.append(f"{v['name']}: Bug 3 - Phase -> isFlapUp -> LUT order")
            print(f"  [FAIL] {v['name']}: Bug 3 - Phase -> isFlapUp -> LUT order")

        # Bug 4
        b4 = audit_bug4(rc_c)
        if b4:
            passed += 1
            print(f"  [PASS] {v['name']}: Bug 4 - switchDelayStart eliminated")
        else:
            failed += 1
            failed_details.append(f"{v['name']}: Bug 4 - switchDelayStart eliminated")
            print(f"  [FAIL] {v['name']}: Bug 4 - switchDelayStart eliminated")

        # Bug 5
        b5 = audit_bug5(main_c)
        if b5:
            passed += 1
            print(f"  [PASS] {v['name']}: Bug 5 - Servo kick prevention on Arm/Disarm/Kill")
        else:
            failed += 1
            failed_details.append(f"{v['name']}: Bug 5 - Servo kick prevention on Arm/Disarm/Kill")
            print(f"  [FAIL] {v['name']}: Bug 5 - Servo kick prevention on Arm/Disarm/Kill")

    # Run Host C++ math tests
    print("\n=== Host Math Unit Tests (g++) ===")
    test_math_src = os.path.join(ROOT_DIR, 'tests', 'host', 'test_math.cpp')
    test_math_bin = os.path.join(ROOT_DIR, 'tests', 'host', 'test_math.exe')
    build_cmd = ['g++', '-std=c++17', '-I', os.path.join(ROOT_DIR, 'tests', 'host'),
                 test_math_src, '-o', test_math_bin]
    build_res = subprocess.run(build_cmd, capture_output=True, text=True)
    if build_res.returncode == 0:
        passed += 1
        print("  [PASS] host: test_math compile")
        run_res = subprocess.run([test_math_bin], capture_output=True, text=True)
        if run_res.returncode == 0:
            passed += 1
            print("  [PASS] host: test_math execution")
        else:
            failed += 1
            failed_details.append("host: test_math execution")
            print(f"  [FAIL] host: test_math execution\n{run_res.stderr}\n{run_res.stdout}")
    else:
        failed += 1
        failed_details.append("host: test_math compile")
        print(f"  [FAIL] host: test_math compile\n{build_res.stderr}")

    print("\n-------------------------------------------")
    print(f"Results: {passed} passed, {failed} failed")
    if failed > 0:
        print("FAILURES:")
        for f in failed_details:
            print(f"  - {f}")
        sys.exit(1)
    else:
        print("ALL AUDITS AND TESTS PASSED!")
        sys.exit(0)

if __name__ == '__main__':
    run_audits()

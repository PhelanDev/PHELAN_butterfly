import os
import re
import sys

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def read_file(rel_path):
    path = os.path.join(ROOT_DIR, rel_path)
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        return f.read()

# Audit 1: CRSF Index 2 Continue placement outside if (c == 0xC8)
def audit_bug1_crsf_continue(content):
    # Pattern: crsfIndex == 2 -> if (crsfLength > CRSF_MAX_PAYLOAD_LEN || crsfLength < 2)
    # Inside: if (c == 0xC8) { crsfBuffer[0] = 0xC8; crsfIndex = 1; }
    # Followed by continue; outside of if (c == 0xC8)
    # And NO continue; inside if (c == 0xC8)
    m = re.search(
        r'if\s*\(\s*(?:state|m_state)\.crsfIndex\s*==\s*2\s*\)\s*\{[\s\S]*?'
        r'if\s*\(\s*(?:state|m_state)\.crsfLength\s*>\s*Config::CRSF_MAX_PAYLOAD_LEN\s*\|\|\s*'
        r'(?:state|m_state)\.crsfLength\s*<\s*2\s*\)\s*\{([\s\S]*?)\}',
        content
    )
    if not m:
        return False
    block = m.group(1)
    # Check that if (c == 0xC8) does NOT contain continue;
    c8_match = re.search(r'if\s*\(\s*c\s*==\s*0xC8\s*\)\s*\{([\s\S]*?)\}', block)
    if not c8_match:
        return False
    if 'continue;' in c8_match.group(1):
        return False
    # Check that continue; is present in the outer error block
    return 'continue;' in block

# Audit 2: UART buffer flush - drain HardwareSerial available bytes
def audit_bug2_uart_drain(content):
    # In switchProtocol, after uart_flush_input, should have:
    # while (state.serialPort->available()) state.serialPort->read();
    m = re.search(
        r'uart_flush_input\s*\(\s*UART_NUM_1\s*\);[\s\S]*?'
        r'while\s*\(\s*(?:state|m_state)\.serialPort->available\(\)\s*\)\s*\{?\s*'
        r'(?:state|m_state)\.serialPort->read\(\);',
        content
    )
    return bool(m)

# Audit 3: pollChannels signature without nowMs & internal millis()
def audit_bug3_poll_channels(rx_content, ino_content):
    # 1. Receiver header or definition: bool pollChannels(RcFrame &out) without unsigned long
    # Should NOT have unsigned long nowMs in parameters
    decl_match = re.search(
        r'bool\s+pollChannels\s*\(\s*(?:Receiver::)?RcFrame\s*&\s*out\s*\)',
        rx_content
    )
    has_no_nowms_param = bool(decl_match)
    
    # 2. Inside pollChannels body: nowMs = millis(); appears AFTER take rxMutex
    body_match = re.search(
        r'bool\s+pollChannels\s*\(\s*(?:Receiver::)?RcFrame\s*&\s*out\s*\)\s*\{[\s\S]*?'
        r'xSemaphoreTakeRecursive\s*\(\s*rxMutex[\s\S]*?'
        r'unsigned\s+long\s+nowMs\s*=\s*millis\(\);',
        rx_content
    )
    has_internal_millis = bool(body_match)
    
    # 3. ino calls pollChannels(rc) without millis()
    ino_call = bool(re.search(
        r'Receiver::pollChannels\s*\(\s*rc\s*\)',
        ino_content
    ))
    ino_no_millis_call = not bool(re.search(
        r'Receiver::pollChannels\s*\(\s*millis\(\)\s*,',
        ino_content
    ))
    
    return has_no_nowms_param and has_internal_millis and ino_call and ino_no_millis_call

# Audit 4: Flight history sync - reset wasFlapping & wasPerching on Disarm and Arm
def audit_bug4_flight_history_sync(ino_content):
    # 1. In handleDisarming() before handleModeTransition
    disarm_chunk = re.search(r'void\s+handleDisarming\s*\([^)]*\)\s*\{([\s\S]*?)(?:void\s+handleModeTransition|$)', ino_content)
    if not disarm_chunk:
        return False
    disarm_text = disarm_chunk.group(1)
    has_disarm_flags = (
        'wasFlapping = false;' in disarm_text and
        'wasPerching = false;' in disarm_text
    )

    # 2. In handleInitMode() on Arm (before handleDisarming)
    init_chunk = re.search(r'void\s+handleInitMode\s*\([^)]*\)\s*\{([\s\S]*?)(?:void\s+handleDisarming|$)', ino_content)
    if not init_chunk:
        return False
    init_text = init_chunk.group(1)
    has_arm_flags = (
        'isArmed = true;' in init_text and
        'wasFlapping = false;' in init_text and
        'wasPerching = false;' in init_text
    )

    return has_disarm_flags and has_arm_flags

def run_all_audits():
    variants = [
        {
            'name': 'single_tria',
            'rx_file': 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino',
            'ino_file': 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino',
        },
        {
            'name': 'single_sin',
            'rx_file': 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino',
            'ino_file': 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino',
        },
        {
            'name': 'multi_tria',
            'rx_file': 'butterfly_esp32c3mini_Phelan_tria/receiver.cpp',
            'rx_hdr': 'butterfly_esp32c3mini_Phelan_tria/receiver.h',
            'ino_file': 'butterfly_esp32c3mini_Phelan_tria/butterfly_esp32c3mini_Phelan_tria.ino',
        },
        {
            'name': 'multi_sin',
            'rx_file': 'butterfly_esp32c3mini_Phelan_sin/receiver.cpp',
            'rx_hdr': 'butterfly_esp32c3mini_Phelan_sin/receiver.h',
            'ino_file': 'butterfly_esp32c3mini_Phelan_sin/butterfly_esp32c3mini_Phelan_sin.ino',
        },
    ]

    passed = 0
    failed = 0
    failures = []

    print("\n=== Static Audits for 4 Edge Case Firmware Bugs ===")

    for v in variants:
        vname = v['name']
        rx_c = read_file(v['rx_file'])
        ino_c = read_file(v['ino_file'])
        rx_check_content = rx_c
        if 'rx_hdr' in v:
            rx_check_content += "\n" + read_file(v['rx_hdr'])

        # Audit 1
        t1_name = f"{vname}: Bug 1 - CRSF continue outside if(c==0xC8)"
        if audit_bug1_crsf_continue(rx_c):
            print(f"  [PASS] {t1_name}")
            passed += 1
        else:
            print(f"  [FAIL] {t1_name}")
            failed += 1
            failures.append(t1_name)

        # Audit 2
        t2_name = f"{vname}: Bug 2 - UART drain HardwareSerial in switchProtocol"
        if audit_bug2_uart_drain(rx_c):
            print(f"  [PASS] {t2_name}")
            passed += 1
        else:
            print(f"  [FAIL] {t2_name}")
            failed += 1
            failures.append(t2_name)

        # Audit 3
        t3_name = f"{vname}: Bug 3 - pollChannels signature & internal millis"
        if audit_bug3_poll_channels(rx_check_content, ino_c):
            print(f"  [PASS] {t3_name}")
            passed += 1
        else:
            print(f"  [FAIL] {t3_name}")
            failed += 1
            failures.append(t3_name)

        # Audit 4
        t4_name = f"{vname}: Bug 4 - Flight history sync on Disarm/Arm"
        if audit_bug4_flight_history_sync(ino_c):
            print(f"  [PASS] {t4_name}")
            passed += 1
        else:
            print(f"  [FAIL] {t4_name}")
            failed += 1
            failures.append(t4_name)

    print("\n-------------------------------------------")
    print(f"Results: {passed} passed, {failed} failed")
    if failed > 0:
        print("FAILURES:")
        for f in failures:
            print(f"  - {f}")
        return 1
    else:
        print("ALL EDGE-CASE AUDITS PASSED!")
        return 0

if __name__ == '__main__':
    sys.exit(run_all_audits())

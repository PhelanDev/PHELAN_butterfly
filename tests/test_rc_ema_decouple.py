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

def extract_block(src, header_regex):
    for m in re.finditer(header_regex, src, re.M):
        start_pos = m.end()
        next_semi = src.find(";", start_pos)
        next_brace = src.find("{", start_pos)
        if next_brace != -1 and (next_semi == -1 or next_brace < next_semi):
            depth = 0
            for i in range(next_brace, len(src)):
                if src[i] == "{":
                    depth += 1
                elif src[i] == "}":
                    depth -= 1
                    if depth == 0:
                        return src[next_brace + 1:i]
    return ""

def audit_variant(rx_content):
    # Audit A: pollChannels has NO EMA calculation
    poll_body = extract_block(rx_content, r'bool\s+pollChannels\s*\([^)]*\)')
    has_no_ema_in_poll = (
        'EMA_ALPHA_SHIFT' not in poll_body and
        'diff /' not in poll_body and
        'diff != 0' not in poll_body
    )

    # Audit B: updateEmaFilter exists and implements symmetric integer division
    ema_body = extract_block(rx_content, r'void\s+updateEmaFilter\s*\([^)]*\)')
    has_update_ema = (
        bool(ema_body) and
        'diff / (1 << Config::EMA_ALPHA_SHIFT)' in ema_body and
        'step == 0' in ema_body and
        'state.emaValues[ch] += step' in ema_body
    )

    # Audit C: parseCrsfChannels calls updateEmaFilter
    crsf_body = extract_block(rx_content, r'void\s+parseCrsfChannels\s*\([^)]*\)')
    has_crsf_call = (
        'updateEmaFilter()' in crsf_body and
        'syncFilterCache()' in crsf_body and
        'state.hasFrame = true' in crsf_body
    )

    # Audit D: parseSbusChannels calls updateEmaFilter when not frame lost
    sbus_body = extract_block(rx_content, r'void\s+parseSbusChannels\s*\([^)]*\)')
    has_sbus_call = (
        'updateEmaFilter()' in sbus_body and
        '!state.sbusFrameLost' in sbus_body and
        'syncFilterCache()' in sbus_body and
        'state.hasFrame = true' in sbus_body
    )

    # Audit E: updateEmaFilter forward declared or declared in header
    has_prototype = 'void updateEmaFilter();' in rx_content

    return {
        'A_poll_no_ema': has_no_ema_in_poll,
        'B_update_ema': has_update_ema,
        'C_crsf_call': has_crsf_call,
        'D_sbus_call': has_sbus_call,
        'E_prototype': has_prototype,
    }

def run_all():
    variants = [
        ('single_tria', 'butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino'),
        ('single_sin', 'butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino'),
        ('multi_tria', 'butterfly_esp32c3mini_Phelan_tria/receiver.cpp'),
        ('multi_sin', 'butterfly_esp32c3mini_Phelan_sin/receiver.cpp'),
    ]

    passed = 0
    failed = 0
    failures = []

    print("\n=== Static Audits for RC EMA Packet-Rate Decoupling ===")

    for vname, rel_path in variants:
        content = read_file(rel_path)
        # For modular variants, check header for prototype
        if 'modular' in vname or 'multi' in vname:
            hdr_path = rel_path.replace('.cpp', '.h')
            header_content = read_file(hdr_path)
            eval_content = content + "\n" + header_content
        else:
            eval_content = content

        res = audit_variant(eval_content)

        tA = f"{vname}: Audit A - Zero EMA in pollChannels"
        if res['A_poll_no_ema']:
            print(f"  [PASS] {tA}")
            passed += 1
        else:
            print(f"  [FAIL] {tA}")
            failed += 1
            failures.append(tA)

        tB = f"{vname}: Audit B - updateEmaFilter symmetric algorithm"
        if res['B_update_ema']:
            print(f"  [PASS] {tB}")
            passed += 1
        else:
            print(f"  [FAIL] {tB}")
            failed += 1
            failures.append(tB)

        tC = f"{vname}: Audit C - parseCrsfChannels calls updateEmaFilter & latches hasFrame"
        if res['C_crsf_call']:
            print(f"  [PASS] {tC}")
            passed += 1
        else:
            print(f"  [FAIL] {tC}")
            failed += 1
            failures.append(tC)

        tD = f"{vname}: Audit D - parseSbusChannels guarded updateEmaFilter & latches hasFrame"
        if res['D_sbus_call']:
            print(f"  [PASS] {tD}")
            passed += 1
        else:
            print(f"  [FAIL] {tD}")
            failed += 1
            failures.append(tD)

        tE = f"{vname}: Audit E - updateEmaFilter forward declaration exists"
        if res['E_prototype']:
            print(f"  [PASS] {tE}")
            passed += 1
        else:
            print(f"  [FAIL] {tE}")
            failed += 1
            failures.append(tE)

    print("\n-------------------------------------------")
    print(f"Results: {passed} passed, {failed} failed")
    if failed > 0:
        print("FAILURES:")
        for f in failures:
            print(f"  - {f}")
        return 1
    else:
        print("ALL RC EMA DECOUPLING AUDITS PASSED!")
        return 0

if __name__ == '__main__':
    sys.exit(run_all())


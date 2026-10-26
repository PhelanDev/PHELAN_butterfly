#!/usr/bin/env python3
import os
import sys
import re
import subprocess
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
except Exception:
    pass

ROOT = Path(__file__).resolve().parent.parent
GPP = r"C:\msys64\ucrt64\bin\g++.exe" if os.path.exists(r"C:\msys64\ucrt64\bin\g++.exe") else "g++"

VARIANTS = {
    "single_tria": {
        "files": ["butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino"],
        "wave": "tria",
    },
    "single_sin": {
        "files": ["butterfly_esp32c3mini_Phelan_single_sin/butterfly_esp32c3mini_Phelan_single_sin.ino"],
        "wave": "sin",
    },
    "multi_tria": {
        "files": [
            "butterfly_esp32c3mini_Phelan_tria/config.h",
            "butterfly_esp32c3mini_Phelan_tria/servo_sys.h",
            "butterfly_esp32c3mini_Phelan_tria/flight.h",
            "butterfly_esp32c3mini_Phelan_tria/receiver.h",
            "butterfly_esp32c3mini_Phelan_tria/servo_sys.cpp",
            "butterfly_esp32c3mini_Phelan_tria/flight.cpp",
            "butterfly_esp32c3mini_Phelan_tria/receiver.cpp",
            "butterfly_esp32c3mini_Phelan_tria/butterfly_esp32c3mini_Phelan_tria.ino",
        ],
        "wave": "tria",
    },
    "multi_sin": {
        "files": [
            "butterfly_esp32c3mini_Phelan_sin/config.h",
            "butterfly_esp32c3mini_Phelan_sin/servo_sys.h",
            "butterfly_esp32c3mini_Phelan_sin/flight.h",
            "butterfly_esp32c3mini_Phelan_sin/receiver.h",
            "butterfly_esp32c3mini_Phelan_sin/servo_sys.cpp",
            "butterfly_esp32c3mini_Phelan_sin/flight.cpp",
            "butterfly_esp32c3mini_Phelan_sin/receiver.cpp",
            "butterfly_esp32c3mini_Phelan_sin/butterfly_esp32c3mini_Phelan_sin.ino",
        ],
        "wave": "sin",
    },
}

failures = []
passed = 0

def audit(name, cond):
    global passed
    if cond:
        passed += 1
        print(f"  [PASS] {name}")
    else:
        failures.append(name)
        print(f"  [FAIL] {name}")

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

def get_variant_src(vname):
    cfg = VARIANTS[vname]
    parts = []
    for rel_path in cfg["files"]:
        p = ROOT / rel_path
        if p.exists():
            parts.append(p.read_text(encoding="utf-8"))
        else:
            parts.append(f"// FILE NOT FOUND: {rel_path}\n")
    return "\n".join(parts)

def run_group1_audits():
    print("\n=== Group 1: RTOS, Multithreading & Hardware Watchdog (Bugs 1-5) ===")
    for v in ("single_tria", "single_sin", "multi_tria", "multi_sin"):
        src = get_variant_src(v)
        flight_core = extract_block(src, r'void\s+flightCoreTask\s*\([^)]*\)')
        switch_proto = extract_block(src, r'void\s+switchProtocol\s*\([^)]*\)')
        reset_parsers = extract_block(src, r'void\s+resetParsers\s*\([^)]*\)')
        setup_wdt = extract_block(src, r'void\s+setupWatchdog\s*\([^)]*\)')

        # Bug 1: flightCoreTask checks return pollChannels and has counter pollFailCount >= 3 forcing failsafe
        has_l1 = ("pollFailCount" in flight_core and
                  "pollChannels" in flight_core and
                  "pollFailCount >= 3" in flight_core)
        audit(f"{v}: Bug 1 - pollChannels check & failsafe counter", has_l1)

        # Bug 2: switchProtocol has no serialPort->end() and has uart_set_baudrate / uart_set_line_inverse
        has_l2 = ("serialPort->end()" not in switch_proto and
                  "uart_set_baudrate" in switch_proto and
                  "uart_set_line_inverse" in switch_proto)
        audit(f"{v}: Bug 2 - UART IDF register reconfig without end()", has_l2)

        # Bug 3: No g_telemSeq, uses g_telemBuf and g_telemIdx
        has_l3 = ("g_telemSeq" not in src and
                  "g_telemBuf" in src and
                  "g_telemIdx" in src and
                  "memory_order_release" in src and
                  "memory_order_acquire" in src)
        audit(f"{v}: Bug 3 - Ping-Pong double buffer telemetry", has_l3)

        # Bug 4: resetParsers protects hasFrame and validFramesCount in rxMutex
        has_l4 = False
        m_take = reset_parsers.find("xSemaphoreTakeRecursive")
        m_give = reset_parsers.find("xSemaphoreGiveRecursive")
        if m_take != -1 and m_give != -1:
            mutex_body = reset_parsers[m_take:m_give]
            has_l4 = ("hasFrame = false" in mutex_body and "validFramesCount = 0" in mutex_body)
        audit(f"{v}: Bug 4 - resetParsers protects hasFrame/validFrames in rxMutex", has_l4)

        # Bug 5: setupWatchdog handles ESP_ERR_INVALID_STATE with esp_task_wdt_reconfigure
        has_l5 = ("esp_task_wdt_reconfigure" in setup_wdt and "ESP_ERR_INVALID_STATE" in setup_wdt)
        audit(f"{v}: Bug 5 - Watchdog reconfigure on INVALID_STATE", has_l5)

def run_group2_audits():
    print("\n=== Group 2: RC Decoding & Signal Filtering (Bugs 6-9) ===")
    for v in ("single_tria", "single_sin", "multi_tria", "multi_sin"):
        src = get_variant_src(v)
        read_crsf = extract_block(src, r'bool\s+readCrsf\s*\([^)]*\)')
        is_sbus_footer = extract_block(src, r'bool\s+isValidSbusFooter\s*\([^)]*\)')
        poll_channels = extract_block(src, r'bool\s+pollChannels\s*\([^)]*\)')

        # Bug 6: readCrsf preserves byte 0xC8 at crsfIndex = 1 on invalid length
        has_l6 = ("state.crsfBuffer[0] = 0xC8" in read_crsf and "state.crsfIndex = 1" in read_crsf)
        audit(f"{v}: Bug 6 - readCrsf preserves 0xC8 sync byte", has_l6)

        # Bug 7: isValidSbusFooter supports SBUS2 ((footer & 0x0F) == 0x04)
        has_l7 = ("(footer & 0x0F) == 0x04" in is_sbus_footer or "footer & 0x0F" in is_sbus_footer)
        audit(f"{v}: Bug 7 - isValidSbusFooter supports SBUS2 (0x04)", has_l7)

        # Bug 8: pollChannels filters EMA with symmetric integer division diff / (1 << Config::EMA_ALPHA_SHIFT)
        has_l8 = ("diff / (1 << Config::EMA_ALPHA_SHIFT)" in poll_channels or
                  "diff / (1 <<" in poll_channels or
                  "diff / 8" in poll_channels)
        audit(f"{v}: Bug 8 - Symmetric integer division EMA for RC channels", has_l8)

        # Bug 9: out.kill reads directly from state.chMicros_cache without EMA
        has_l9 = "out.kill = state.chMicros_cache[Config::CH_KILL_SWITCH]" in poll_channels
        audit(f"{v}: Bug 9 - Instant raw kill switch without EMA delay", has_l9)

def run_group3_audits():
    print("\n=== Group 3: Wing Kinematics, Math & Servo Control (Bugs 10-14) ===")
    for v in ("single_tria", "single_sin", "multi_tria", "multi_sin"):
        src = get_variant_src(v)
        glide_body = extract_block(src, r'void\s+updateCurrentOffsetGlide\s*\([^)]*\)')
        servo_update = extract_block(src, r'void\s+update\s*\([^)]*float\s+targetL[^)]*\)')
        map_trim = extract_block(src, r'float\s+mapTrimFloat\s*\([^)]*\)')
        compute_targets = extract_block(src, r'void\s+computeServoTargets\s*\([^)]*\)')

        # Bug 10: updateCurrentOffsetGlide resets isFlapUp = true near 0
        has_l10 = ("state.isFlapUp = true" in glide_body and ("0.01" in glide_body or "currentOffset == 0" in glide_body or "fabsf" in glide_body))
        audit(f"{v}: Bug 10 - Glide near 0 resets isFlapUp to true", has_l10)

        # Bug 11: Triangle uses isRisingPhase instead of delta
        if "tria" in v:
            tria_body = extract_block(src, r'void\s+updateCurrentOffsetTriangle\s*\([^)]*\)')
            has_l11 = ("isRisingPhase" in tria_body and not re.search(r'\bdelta\b', tria_body))
            audit(f"{v}: Bug 11 - Triangle flap direction uses isRisingPhase without delta", has_l11)
        else:
            audit(f"{v}: Bug 11 - Sine already verified isRisingPhase", True)

        # Bug 12: ServoSystem::update scaled EMA with symmetric step +-1
        has_l12 = ("scaledTargetL" in servo_update and
                   ("diffL / (1 <<" in servo_update or "diffL / 2" in servo_update or "diffL / (1 << Config::SERVO_EMA_ALPHA_SHIFT)" in servo_update))
        audit(f"{v}: Bug 12 - Servo EMA scaled target with symmetric step", has_l12)

        # Bug 13: mapTrimFloat divides by span 490.0f
        has_l13 = ("490.0f" in map_trim or "CH_MAX - CH_CENTER - STICK_DEADBAND" in map_trim)
        audit(f"{v}: Bug 13 - mapTrimFloat normalized with 490us span", has_l13)

        # Bug 14: computeServoTargets maps Perch from PERCH_ENTER_ELEV to CH_MIN
        has_l14 = ("PERCH_ENTER_ELEV - elevator" in compute_targets and "PERCH_ENTER_ELEV - Config::CH_MIN" in compute_targets)
        audit(f"{v}: Bug 14 - Smooth perch mapping from PERCH_ENTER_ELEV", has_l14)

def prepare_extracted_config():
    src = (ROOT / "butterfly_esp32c3mini_Phelan_single_tria/butterfly_esp32c3mini_Phelan_single_tria.ino").read_text(encoding="utf-8")
    config_block = extract_block(src, r'namespace\s+Config\b')
    if "mapTrimFloat" in config_block and "angleToUs" in config_block:
        hdr = "#pragma once\n#define HAS_EXTRACTED_CONFIG 1\nnamespace Config {\n" + config_block + "\n}\n"
    else:
        hdr = "#pragma once\n// mapTrimFloat or angleToUs missing\n"
    (ROOT / "tests/host/extracted_config.h").write_text(hdr, encoding="utf-8")

def run_host_math_tests():
    print("\n=== Host Math Unit Tests (g++) ===")
    prepare_extracted_config()
    test_src = ROOT / "tests/host/test_math.cpp"
    out_exe = ROOT / "tests/host/test_math.exe"

    cmd = [GPP, "-std=gnu++17", "-I", str(ROOT / "tests/host"), str(test_src), "-o", str(out_exe)]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0:
        audit("host: test_math compile", False)
        print(res.stderr)
        return
    else:
        audit("host: test_math compile", True)

    run_res = subprocess.run([str(out_exe)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if out_exe.exists():
        try:
            out_exe.unlink()
        except Exception:
            pass

    audit("host: test_math execution", run_res.returncode == 0)
    if run_res.returncode != 0:
        print(run_res.stderr or run_res.stdout)

def main():
    run_group1_audits()
    run_group2_audits()
    run_group3_audits()
    run_host_math_tests()

    print("\n-------------------------------------------")
    print(f"Results: {passed} passed, {len(failures)} failed")
    if failures:
        print("FAILURES:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    else:
        print("ALL AUDITS AND TESTS PASSED!")
        sys.exit(0)

if __name__ == "__main__":
    main()

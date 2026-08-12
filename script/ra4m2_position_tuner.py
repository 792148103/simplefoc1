#!/usr/bin/env python3
"""
Conservative position-loop tuner for the RA4M2 SimpleFOC firmware.

The script talks to the MCU over the same text protocol used by
FOC_Dashboard_RA4M2.html. It only tunes the angle P term (NAP) by default,
while keeping the current angle-mode velocity PID values explicit.

Install dependency if needed:
    python -m pip install pyserial

Example:
    python script/ra4m2_position_tuner.py --port COM7 --run

Full-turn example:
    python script/ra4m2_position_tuner.py --port COM7 --run --full-turns 1 --turn-step-duration 3.0
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import random
import re
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

try:
    import serial
except ImportError:  # pragma: no cover - useful user message when run locally
    serial = None


TWO_PI = 2.0 * math.pi


class TeeStream:
    def __init__(self, *streams) -> None:
        self.streams = streams

    def write(self, text: str) -> int:
        for stream in self.streams:
            stream.write(text)
            stream.flush()
        return len(text)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


@dataclass
class AnglePidState:
    nap: float = 28.0
    nvp: float = 0.025
    nvi: float = 0.5
    nvf: float = 0.025
    nvl: float = 300.0


@dataclass
class StepMetrics:
    target: float
    final_actual: float
    max_actual: float
    min_actual: float
    steady_error: float
    peak_error: float
    overshoot: float
    oscillation: float
    samples: int


class Ra4m2Serial:
    def __init__(self, port: str, baud: int, timeout: float = 0.05) -> None:
        if serial is not None:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=0.5)
            self.backend = "pyserial"
        elif os.name == "nt":
            self.ser = WinSerial(port=port, baud=baud, timeout=timeout)
            self.backend = "win32"
        else:
            raise RuntimeError("pyserial is not installed. Run: python -m pip install pyserial")

        print(f"Opened {port} at {baud} baud using {self.backend}")
        time.sleep(1.0)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def command(self, text: str, wait_ack: bool = True, ack_timeout: float = 0.8) -> bool:
        line = (text.strip() + "\n").encode("ascii")
        self.ser.write(line)
        self.ser.flush()
        if not wait_ack:
            return True

        deadline = time.monotonic() + ack_timeout
        wanted = "MCU_ACK: " + text.strip()
        while time.monotonic() < deadline:
            line_text = self.read_line()
            if not line_text:
                continue
            if line_text == wanted:
                return True
            if line_text.startswith("MCU_ACK:"):
                continue
        return False

    def read_line(self) -> str:
        try:
            raw = self.ser.readline()
        except Exception:
            return ""
        if not raw:
            return ""
        return raw.decode("ascii", errors="ignore").strip()

    def read_frame(self, timeout: float = 0.5) -> Optional[tuple[float, float]]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            text = self.read_line()
            if not text:
                continue
            parts = [item.strip() for item in text.split(",")]
            if len(parts) < 2:
                continue
            try:
                return float(parts[0]), float(parts[1])
            except ValueError:
                continue
        return None

    def collect_frames(self, duration: float) -> list[tuple[float, float]]:
        frames: list[tuple[float, float]] = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            frame = self.read_frame(timeout=0.1)
            if frame is not None:
                frames.append(frame)
        return frames


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", ctypes.c_uint32),
        ("BaudRate", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("wReserved", ctypes.c_uint16),
        ("XonLim", ctypes.c_uint16),
        ("XoffLim", ctypes.c_uint16),
        ("ByteSize", ctypes.c_uint8),
        ("Parity", ctypes.c_uint8),
        ("StopBits", ctypes.c_uint8),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", ctypes.c_uint16),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", ctypes.c_uint32),
        ("ReadTotalTimeoutMultiplier", ctypes.c_uint32),
        ("ReadTotalTimeoutConstant", ctypes.c_uint32),
        ("WriteTotalTimeoutMultiplier", ctypes.c_uint32),
        ("WriteTotalTimeoutConstant", ctypes.c_uint32),
    ]


class WinSerial:
    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
    PURGE_RXCLEAR = 0x0008
    PURGE_TXCLEAR = 0x0004

    def __init__(self, port: str, baud: int, timeout: float) -> None:
        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self.timeout = timeout
        device = port if port.startswith("\\\\.\\") else "\\\\.\\" + port
        self.handle = self.kernel32.CreateFileW(
            ctypes.c_wchar_p(device),
            self.GENERIC_READ | self.GENERIC_WRITE,
            0,
            None,
            self.OPEN_EXISTING,
            0,
            None,
        )
        if self.handle == self.INVALID_HANDLE_VALUE:
            raise OSError(ctypes.get_last_error(), f"Cannot open {port}")

        try:
            self._configure(baud)
            self.reset_input_buffer()
        except Exception:
            self.close()
            raise

    def _configure(self, baud: int) -> None:
        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not self.kernel32.GetCommState(self.handle, ctypes.byref(dcb)):
            raise OSError(ctypes.get_last_error(), "GetCommState failed")

        config = f"baud={baud} parity=N data=8 stop=1"
        if not self.kernel32.BuildCommDCBW(ctypes.c_wchar_p(config), ctypes.byref(dcb)):
            raise OSError(ctypes.get_last_error(), "BuildCommDCB failed")
        if not self.kernel32.SetCommState(self.handle, ctypes.byref(dcb)):
            raise OSError(ctypes.get_last_error(), "SetCommState failed")

        timeout_ms = max(1, int(self.timeout * 1000))
        timeouts = COMMTIMEOUTS(
            ReadIntervalTimeout=timeout_ms,
            ReadTotalTimeoutMultiplier=0,
            ReadTotalTimeoutConstant=timeout_ms,
            WriteTotalTimeoutMultiplier=0,
            WriteTotalTimeoutConstant=500,
        )
        if not self.kernel32.SetCommTimeouts(self.handle, ctypes.byref(timeouts)):
            raise OSError(ctypes.get_last_error(), "SetCommTimeouts failed")

    def reset_input_buffer(self) -> None:
        self.kernel32.PurgeComm(self.handle, self.PURGE_RXCLEAR | self.PURGE_TXCLEAR)

    def write(self, data: bytes) -> int:
        written = ctypes.c_uint32(0)
        buffer = ctypes.create_string_buffer(data)
        ok = self.kernel32.WriteFile(self.handle, buffer, len(data), ctypes.byref(written), None)
        if not ok:
            raise OSError(ctypes.get_last_error(), "WriteFile failed")
        return int(written.value)

    def flush(self) -> None:
        self.kernel32.FlushFileBuffers(self.handle)

    def readline(self) -> bytes:
        data = bytearray()
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            byte = ctypes.create_string_buffer(1)
            read = ctypes.c_uint32(0)
            ok = self.kernel32.ReadFile(self.handle, byte, 1, ctypes.byref(read), None)
            if not ok:
                raise OSError(ctypes.get_last_error(), "ReadFile failed")
            if read.value == 0:
                continue
            data.extend(byte.raw[:1])
            if data[-1:] == b"\n":
                break
        return bytes(data)

    def close(self) -> None:
        if getattr(self, "handle", None) not in (None, self.INVALID_HANDLE_VALUE):
            self.kernel32.CloseHandle(self.handle)
            self.handle = None


def send_pid_state(link: Ra4m2Serial, state: AnglePidState) -> None:
    commands = [
        "NMD0",
        f"NAP{state.nap:.4f}",
        f"NVP{state.nvp:.4f}",
        f"NVI{state.nvi:.4f}",
        f"NVF{state.nvf:.4f}",
        f"NVL{state.nvl:.4f}",
    ]
    for cmd in commands:
        ok = link.command(cmd)
        print(f"{cmd:<12} {'ACK' if ok else 'NO_ACK'}")
        time.sleep(0.06)


def estimate_current_position(link: Ra4m2Serial, settle_time: float) -> float:
    frames = link.collect_frames(settle_time)
    actual_values = [actual for _, actual in frames]
    if not actual_values:
        raise RuntimeError("No telemetry frames received. Check NDS0, baud rate, and serial port.")
    return statistics.median(actual_values[-min(10, len(actual_values)):])


def run_step(link: Ra4m2Serial, target: float, duration: float) -> StepMetrics:
    ok = link.command(f"NTG{target:.4f}")
    print(f"NTG{target:.4f}  {'ACK' if ok else 'NO_ACK'}  wait={duration:.2f}s")
    frames = link.collect_frames(duration)
    if len(frames) < 8:
        raise RuntimeError(f"Too few telemetry frames for target {target:.3f}: {len(frames)}")

    actual_values = [actual for _, actual in frames]
    tail = actual_values[-max(5, len(actual_values) // 5):]
    final_actual = statistics.mean(tail)
    errors = [target - value for value in actual_values]
    tail_errors = [target - value for value in tail]
    steady_error = statistics.mean(tail_errors)
    peak_error = max(abs(error) for error in errors)

    if target >= actual_values[0]:
        overshoot = max(0.0, max(actual_values) - target)
    else:
        overshoot = max(0.0, target - min(actual_values))

    oscillation = statistics.pstdev(tail) if len(tail) >= 2 else 0.0
    return StepMetrics(
        target=target,
        final_actual=final_actual,
        max_actual=max(actual_values),
        min_actual=min(actual_values),
        steady_error=steady_error,
        peak_error=peak_error,
        overshoot=overshoot,
        oscillation=oscillation,
        samples=len(frames),
    )


def score(metrics: Iterable[StepMetrics]) -> float:
    total = 0.0
    for item in metrics:
        total += abs(item.steady_error) * 2.0
        total += item.overshoot * 3.0
        total += item.oscillation * 8.0
        total += max(0.0, item.peak_error - 1.0) * 0.5
    return total


def safe_metrics(metrics: Iterable[StepMetrics], max_overshoot: float, max_oscillation: float) -> bool:
    for item in metrics:
        if item.overshoot > max_overshoot:
            return False
        if item.oscillation > max_oscillation:
            return False
    return True


def print_metrics(label: str, metrics: list[StepMetrics]) -> None:
    print(f"\n[{label}]")
    for item in metrics:
        print(
            "target={:.3f} final={:.3f} steady_err={:+.3f} "
            "overshoot={:.3f} osc={:.4f} samples={}".format(
                item.target,
                item.final_actual,
                item.steady_error,
                item.overshoot,
                item.oscillation,
                item.samples,
            )
        )
    print(f"score={score(metrics):.5f}")


def load_pid_state_from_log(path: str) -> Optional[AnglePidState]:
    if not path or not os.path.exists(path):
        return None

    text = open(path, "r", encoding="utf-8", errors="ignore").read()
    patterns = {
        "nap": r"NAP angle P\s*=\s*([-+]?\d+(?:\.\d+)?)",
        "nvp": r"NVP velocity P\s*=\s*([-+]?\d+(?:\.\d+)?)",
        "nvi": r"NVI velocity I\s*=\s*([-+]?\d+(?:\.\d+)?)",
        "nvf": r"NVF velocity Tf\s*=\s*([-+]?\d+(?:\.\d+)?)",
        "nvl": r"NVL velocity lim\s*=\s*([-+]?\d+(?:\.\d+)?)",
    }
    values: dict[str, float] = {}
    for key, pattern in patterns.items():
        matches = re.findall(pattern, text)
        if not matches:
            return None
        values[key] = float(matches[-1])
    return AnglePidState(**values)


def make_targets(args: argparse.Namespace, center: float) -> list[float]:
    targets = [center + args.step_rad, center - args.step_rad, center]

    if args.extra_steps:
        for scale in (0.5, 1.5, 2.0):
            value = min(args.max_test_rad, args.step_rad * scale)
            if value > 0.0:
                targets.extend([center + value, center - value])
        targets.append(center)

    turn_request = effective_turn_request(args)
    if turn_request:
        turn_count = min(abs(turn_request), args.max_turns)
        direction = 1.0 if turn_request >= 0 else -1.0
        waypoint_count = max(1, args.turn_waypoints)

        for sign in (1.0, -1.0):
            for waypoint_index in range(1, waypoint_count + 1):
                waypoint_ratio = waypoint_index / waypoint_count
                targets.append(center + sign * direction * turn_count * TWO_PI * waypoint_ratio)
            targets.append(center)

    if args.random_targets > 0:
        rng = random.Random(args.seed)
        span = args.random_span_rad
        for _ in range(args.random_targets):
            targets.append(center + rng.uniform(-span, span))
        targets.append(center)

    filtered: list[float] = []
    max_delta = max(args.max_test_rad, abs(turn_request) * TWO_PI if turn_request else 0.0)
    for target in targets:
        if abs(target - center) <= max_delta + 1e-6:
            filtered.append(target)

    return filtered


def effective_turn_request(args: argparse.Namespace) -> float:
    return args.full_turns if args.full_turns is not None else args.turns


def move_duration(args: argparse.Namespace, previous_target: float, next_target: float) -> float:
    displacement = abs(next_target - previous_target)
    if displacement > args.max_test_rad + 1e-6:
        return max(args.step_duration, args.turn_step_duration * max(1.0, displacement / TWO_PI))
    return args.step_duration


def test_candidate(
    link: Ra4m2Serial,
    state: AnglePidState,
    targets: list[float],
    args: argparse.Namespace,
    start_position: float,
) -> list[StepMetrics]:
    send_pid_state(link, state)
    metrics: list[StepMetrics] = []
    previous_target = start_position
    for target in targets:
        duration = move_duration(args, previous_target, target)
        metrics.append(run_step(link, target, duration))
        previous_target = target
    return metrics


def tune_position(args: argparse.Namespace) -> AnglePidState:
    state = AnglePidState(
        nap=args.nap,
        nvp=args.nvp,
        nvi=args.nvi,
        nvf=args.nvf,
        nvl=args.nvl,
    )
    if args.start_from_last:
        loaded = getattr(args, "loaded_previous_state", None)
        if loaded is None:
            print(f"Previous PID state not found in {args.previous_log_file}; using command-line defaults.")
        else:
            state = loaded
            print("Loaded previous PID state:")
            print(f"  NAP={state.nap:.4f}, NVP={state.nvp:.4f}, NVI={state.nvi:.4f}, NVF={state.nvf:.4f}, NVL={state.nvl:.4f}")

    link = Ra4m2Serial(args.port, args.baud)
    try:
        print("Enabling compact telemetry and entering angle mode...")
        link.command("NDS0")
        time.sleep(0.15)
        link.command("NMD0")
        time.sleep(0.2)

        center = estimate_current_position(link, args.settle)
        print(f"Measured current position: {center:.4f} rad")

        targets = make_targets(args, center)
        print("Test targets:")
        for target in targets:
            print(f"  {target:.4f} rad ({(target - center) / TWO_PI:+.3f} turns from center)")

        base_metrics = test_candidate(link, state, targets, args, center)
        print_metrics("baseline", base_metrics)
        best_state = AnglePidState(**state.__dict__)
        best_score = score(base_metrics)

        for round_index in range(args.rounds):
            factors = [0.85, 1.15]
            improved = False
            for factor in factors:
                candidate = AnglePidState(**best_state.__dict__)
                candidate.nap = max(args.min_nap, min(args.max_nap, best_state.nap * factor))
                if abs(candidate.nap - best_state.nap) < 1e-5:
                    continue

                print(f"\nRound {round_index + 1}: testing NAP={candidate.nap:.4f}")
                metrics = test_candidate(link, candidate, targets, args, center)
                print_metrics(f"candidate NAP={candidate.nap:.4f}", metrics)

                candidate_score = score(metrics)
                if safe_metrics(metrics, args.max_overshoot, args.max_oscillation) and candidate_score < best_score:
                    best_state = candidate
                    best_score = candidate_score
                    improved = True
                    print(f"Accepted NAP={best_state.nap:.4f}")

            if not improved:
                print("\nNo safer score improvement in this round; stopping.")
                break

        print("\nApplying final PID state...")
        send_pid_state(link, best_state)
        link.command(f"NTG{center:.4f}")
        time.sleep(0.2)

        if args.quiet_end:
            link.command("NDS1")

        return best_state
    finally:
        link.close()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RA4M2 SimpleFOC conservative angle-loop tuner")
    parser.add_argument("--port", required=True, help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--run", action="store_true", help="Actually send commands to the motor")

    parser.add_argument("--nap", type=float, default=28.0, help="Initial angle P")
    parser.add_argument("--nvp", type=float, default=0.025, help="Angle-mode velocity P")
    parser.add_argument("--nvi", type=float, default=0.5, help="Angle-mode velocity I")
    parser.add_argument("--nvf", type=float, default=0.025, help="Angle-mode velocity LPF Tf")
    parser.add_argument("--nvl", type=float, default=300.0, help="Angle-mode velocity limit")

    parser.add_argument("--min-nap", type=float, default=2.0)
    parser.add_argument("--max-nap", type=float, default=60.0)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--step-rad", type=float, default=0.35, help="Small angle step around current position")
    parser.add_argument("--extra-steps", action="store_true", help="Also test multiple deterministic angle steps")
    parser.add_argument("--random-targets", type=int, default=0, help="Number of random targets around current position")
    parser.add_argument("--random-span-rad", type=float, default=1.0, help="Random target span around center")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for repeatable tests")
    parser.add_argument("--turns", type=float, default=0.0, help="Also test +/- this many full turns from current position")
    parser.add_argument("--full-turns", type=float, default=None, help="Clearer alias for --turns; enables full-circle position tests")
    parser.add_argument("--max-turns", type=float, default=2.0, help="Safety clamp for --turns")
    parser.add_argument("--turn-waypoints", type=int, default=1, help="Split each full-turn move into this many intermediate targets")
    parser.add_argument("--turn-step-duration", type=float, default=3.0, help="Wait time used for large full-turn moves")
    parser.add_argument("--max-test-rad", type=float, default=1.5, help="Safety clamp for non-turn test displacement")
    parser.add_argument("--step-duration", type=float, default=1.2)
    parser.add_argument("--settle", type=float, default=0.8)
    parser.add_argument("--max-overshoot", type=float, default=0.20)
    parser.add_argument("--max-oscillation", type=float, default=0.035)
    parser.add_argument("--quiet-end", action="store_true", help="Send NDS1 when done")
    parser.add_argument(
        "--log-file",
        default=os.path.join(os.path.dirname(__file__), "position_tuner_last_result.txt"),
        help="Path for a copy of the tuner output",
    )
    parser.add_argument(
        "--previous-log-file",
        default=os.path.join(os.path.dirname(__file__), "position_tuner_last_result.txt"),
        help="Log file to read when --start-from-last is enabled",
    )
    parser.add_argument("--start-from-last", action="store_true", help="Continue from the last final PID state in the previous log")
    args = parser.parse_args(argv)

    if not args.run:
        parser.error("This script moves the motor. Add --run after confirming the shaft can move safely.")
    if args.step_rad <= 0.0 or args.step_rad > 1.0:
        parser.error("--step-rad must be > 0 and <= 1.0 for this conservative tuner")
    if args.random_targets < 0:
        parser.error("--random-targets must be >= 0")
    if args.random_span_rad <= 0.0 or args.random_span_rad > args.max_test_rad:
        parser.error("--random-span-rad must be > 0 and <= --max-test-rad")
    if args.max_test_rad <= 0.0 or args.max_test_rad > 6.0:
        parser.error("--max-test-rad must be > 0 and <= 6.0")
    if args.full_turns is not None and abs(args.turns) > 1e-9:
        parser.error("Use either --turns or --full-turns, not both")
    if args.turn_waypoints < 1 or args.turn_waypoints > 32:
        parser.error("--turn-waypoints must be between 1 and 32")
    if args.turn_step_duration <= 0.0:
        parser.error("--turn-step-duration must be > 0")
    turn_request = effective_turn_request(args)
    if abs(turn_request) > args.max_turns:
        print(f"Requested full-turn test {turn_request} exceeds --max-turns {args.max_turns}; clamping in target generation.")
    return args


def main(argv: list[str]) -> int:
    log_handle = None
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    try:
        args = parse_args(argv)
        if args.start_from_last:
            args.loaded_previous_state = load_pid_state_from_log(args.previous_log_file)
        else:
            args.loaded_previous_state = None

        log_handle = open(args.log_file, "w", encoding="utf-8")
        sys.stdout = TeeStream(original_stdout, log_handle)
        sys.stderr = TeeStream(original_stderr, log_handle)

        print("RA4M2 position tuner log")
        print(time.strftime("Started: %Y-%m-%d %H:%M:%S"))
        print(f"Log file: {os.path.abspath(args.log_file)}\n")

        final_state = tune_position(args)
    except KeyboardInterrupt:
        print("\nInterrupted by user. If the motor is unsafe, press P000 or cut power.", file=sys.stderr)
        if log_handle is not None:
            sys.stdout = original_stdout
            sys.stderr = original_stderr
            log_handle.close()
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        print("If the motor is unsafe, press P000 or cut power.", file=sys.stderr)
        if log_handle is not None:
            sys.stdout = original_stdout
            sys.stderr = original_stderr
            log_handle.close()
        return 1

    print("\nFinal angle-mode PID state sent to MCU:")
    print(f"NAP angle P       = {final_state.nap:.4f}")
    print(f"NVP velocity P    = {final_state.nvp:.4f}")
    print(f"NVI velocity I    = {final_state.nvi:.4f}")
    print(f"NVF velocity Tf   = {final_state.nvf:.4f}")
    print(f"NVL velocity lim  = {final_state.nvl:.4f}")
    print("\nNote: the firmware does not persist these values across reset unless you add flash storage.")
    if log_handle is not None:
        print(f"Log saved to: {os.path.abspath(log_handle.name)}")
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        log_handle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

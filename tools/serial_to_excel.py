#!/usr/bin/env python3
"""Log DiscoveryPID telemetry to Excel and CSV files.

The workbook contains:
  1. Telemetry  - the complete serial data table.
  2. JMP Data   - only P, I, and target-minus-output error.
  3. Summary    - run-level statistics and charts.
  4. Metadata   - setup and file information.

The logger also creates two CSV files:
  DiscoveryPID_YYYYMMDD_HHMMSS.csv
  DiscoveryPID_YYYYMMDD_HHMMSS_JMP.csv

Examples:
  py tools/serial_to_excel.py --port COM5 --p 1.25 --i 2.75 --target 100 --duration 30
  py tools/serial_to_excel.py --port COM5 --target 100 --duration 60
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import serial
from serial import SerialException
from serial.tools import list_ports
import xlsxwriter

BAUD_RATE = 115200
SCRIPT_VERSION = "1.1-jmp-autotune"

PAIR_PATTERN = re.compile(
    r"([A-Za-z][A-Za-z0-9_]*)=(-?(?:\d+(?:\.\d*)?|\.\d+))"
)

REQUIRED_FIELDS = {
    "P",
    "I",
    "Target",
    "OutRPM",
    "MotorRPM",
    "Integral",
    "Counts",
    "dC",
    "PWM",
}

TELEMETRY_HEADERS = [
    "PC Timestamp",
    "Elapsed Time (s)",
    "P",
    "I",
    "Target RPM",
    "Output RPM",
    "Motor RPM",
    "Error (Target - Output)",
    "Integral",
    "Encoder Counts",
    "Delta Counts",
    "PWM Command",
    "Raw Serial Line",
]

JMP_HEADERS = ["P", "I", "Error"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save DiscoveryPID telemetry to Excel, full CSV, and JMP CSV files."
    )
    parser.add_argument(
        "--port",
        help="Serial port, such as COM5. If omitted, the script tries to detect it.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=BAUD_RATE,
        help=f"Serial baud rate. Default: {BAUD_RATE}",
    )
    parser.add_argument(
        "--p",
        type=float,
        default=2.0,
        help="Proportional gain sent to the autotuning firmware. Default: 2.0",
    )
    parser.add_argument(
        "--i",
        type=float,
        default=5.0,
        help="Integral gain sent to the autotuning firmware. Default: 5.0",
    )
    parser.add_argument(
        "--target",
        type=float,
        help="Optional target output RPM sent after P and I are configured.",
    )
    parser.add_argument(
        "--trial",
        type=int,
        default=1,
        help="Trial number included in firmware telemetry. Default: 1",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Optional logging duration in seconds. Otherwise press Ctrl+C to stop.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional XLSX output path. CSV files use the same file stem.",
    )
    return parser.parse_args()


def select_serial_port(requested_port: Optional[str]) -> str:
    if requested_port:
        return requested_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports were detected.")

    likely_keywords = ("arduino", "mega", "ch340", "wch", "usb serial")
    likely = [
        port
        for port in ports
        if any(keyword in (port.description or "").lower() for keyword in likely_keywords)
    ]

    if len(likely) == 1:
        return likely[0].device

    if len(ports) == 1:
        return ports[0].device

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")

    selected = input("Enter the Arduino port, such as COM5: ").strip()
    if not selected:
        raise RuntimeError("No serial port was selected.")
    return selected


def parse_telemetry(line: str) -> Optional[Dict[str, float]]:
    values = {key: float(value) for key, value in PAIR_PATTERN.findall(line)}
    if not REQUIRED_FIELDS.issubset(values):
        return None
    return values


def build_output_paths(output: Optional[Path]) -> tuple[Path, Path, Path]:
    if output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        xlsx_path = Path("data") / "logs" / f"DiscoveryPID_{timestamp}.xlsx"
    else:
        xlsx_path = output
        if xlsx_path.suffix.lower() != ".xlsx":
            xlsx_path = xlsx_path.with_suffix(".xlsx")

    xlsx_path.parent.mkdir(parents=True, exist_ok=True)
    full_csv_path = xlsx_path.with_suffix(".csv")
    jmp_csv_path = xlsx_path.with_name(f"{xlsx_path.stem}_JMP.csv")
    return xlsx_path, full_csv_path, jmp_csv_path


def send_command(serial_port: serial.Serial, command: str) -> None:
    serial_port.write(f"{command}\n".encode("ascii"))
    serial_port.flush()
    time.sleep(0.05)
    print(f"Sent: {command}")


def main() -> int:
    args = parse_args()

    if not 0.0 <= args.p <= 10.0:
        print("P must be between 0 and 10.", file=sys.stderr)
        return 2
    if not 0.0 <= args.i <= 50.0:
        print("I must be between 0 and 50.", file=sys.stderr)
        return 2
    if args.duration is not None and args.duration <= 0:
        print("Duration must be greater than zero.", file=sys.stderr)
        return 2

    try:
        port_name = select_serial_port(args.port)
    except RuntimeError as exc:
        print(f"Port selection error: {exc}", file=sys.stderr)
        return 2

    xlsx_path, full_csv_path, jmp_csv_path = build_output_paths(args.output)

    print(f"Opening {port_name} at {args.baud} baud...")
    try:
        serial_port = serial.Serial(
            port=port_name,
            baudrate=args.baud,
            timeout=1.0,
            write_timeout=1.0,
        )
    except SerialException as exc:
        print(f"Could not open {port_name}: {exc}", file=sys.stderr)
        print("Close the PlatformIO serial monitor and try again.", file=sys.stderr)
        return 3

    # Opening the Mega serial port normally resets it.
    time.sleep(2.0)

    send_command(serial_port, f"TRIAL={args.trial}")
    send_command(serial_port, f"P={args.p}")
    send_command(serial_port, f"I={args.i}")
    if args.target is not None:
        send_command(serial_port, f"RPM={args.target}")

    workbook = xlsxwriter.Workbook(str(xlsx_path))
    telemetry_sheet = workbook.add_worksheet("Telemetry")
    jmp_sheet = workbook.add_worksheet("JMP Data")
    summary_sheet = workbook.add_worksheet("Summary")
    metadata_sheet = workbook.add_worksheet("Metadata")

    header_format = workbook.add_format(
        {
            "bold": True,
            "font_color": "white",
            "bg_color": "#1F4E78",
            "border": 1,
            "align": "center",
            "valign": "vcenter",
        }
    )
    timestamp_format = workbook.add_format({"num_format": "yyyy-mm-dd hh:mm:ss.000"})
    number_format = workbook.add_format({"num_format": "0.000"})
    integer_format = workbook.add_format({"num_format": "0"})
    title_format = workbook.add_format(
        {"bold": True, "font_size": 16, "font_color": "#1F4E78"}
    )
    label_format = workbook.add_format({"bold": True, "bg_color": "#D9EAF7"})

    for column, header in enumerate(TELEMETRY_HEADERS):
        telemetry_sheet.write(0, column, header, header_format)
    for column, header in enumerate(JMP_HEADERS):
        jmp_sheet.write(0, column, header, header_format)

    telemetry_sheet.freeze_panes(1, 0)
    telemetry_sheet.set_column("A:A", 24)
    telemetry_sheet.set_column("B:B", 16)
    telemetry_sheet.set_column("C:L", 16)
    telemetry_sheet.set_column("M:M", 100)

    jmp_sheet.freeze_panes(1, 0)
    jmp_sheet.set_column("A:C", 20)

    metadata_sheet.set_column("A:A", 26)
    metadata_sheet.set_column("B:B", 70)
    metadata_sheet.write("A1", "DiscoveryPID Log Metadata", title_format)
    metadata_rows = [
        ("Logger version", SCRIPT_VERSION),
        ("Serial port", port_name),
        ("Baud rate", args.baud),
        ("Logging started", datetime.now().isoformat(timespec="seconds")),
        ("Trial", args.trial),
        ("Requested P", args.p),
        ("Requested I", args.i),
        ("Requested target RPM", args.target if args.target is not None else "Not sent"),
        ("XLSX path", str(xlsx_path.resolve())),
        ("Full CSV path", str(full_csv_path.resolve())),
        ("JMP CSV path", str(jmp_csv_path.resolve())),
    ]
    for metadata_row, (label, value) in enumerate(metadata_rows, start=2):
        metadata_sheet.write(metadata_row, 0, label, label_format)
        metadata_sheet.write(metadata_row, 1, value)

    full_csv_file = full_csv_path.open("w", newline="", encoding="utf-8")
    full_csv_writer = csv.writer(full_csv_file)
    full_csv_writer.writerow(TELEMETRY_HEADERS)
    full_csv_file.flush()

    jmp_csv_file = jmp_csv_path.open("w", newline="", encoding="utf-8")
    jmp_csv_writer = csv.writer(jmp_csv_file)
    jmp_csv_writer.writerow(JMP_HEADERS)
    jmp_csv_file.flush()

    start_monotonic = time.monotonic()
    start_wall_clock = datetime.now()
    row_index = 1
    output_rpm_values: list[float] = []
    error_values: list[float] = []
    pwm_values: list[float] = []
    p_values: list[float] = []
    i_values: list[float] = []
    firmware_line = "Not detected"
    startup_messages: list[str] = []

    print("Logging telemetry. Press Ctrl+C to stop and finalize the files.")
    print(f"XLSX:     {xlsx_path.resolve()}")
    print(f"Full CSV: {full_csv_path.resolve()}")
    print(f"JMP CSV:  {jmp_csv_path.resolve()}")

    try:
        while True:
            if args.duration is not None:
                if time.monotonic() - start_monotonic >= args.duration:
                    print("Requested duration reached.")
                    break

            raw_line = serial_port.readline()
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            print(line)

            if line.startswith("Firmware:"):
                firmware_line = line.partition(":")[2].strip()

            telemetry = parse_telemetry(line)
            if telemetry is None:
                if len(startup_messages) < 100:
                    startup_messages.append(line)
                continue

            now = datetime.now()
            elapsed_seconds = time.monotonic() - start_monotonic
            p_gain = telemetry["P"]
            i_gain = telemetry["I"]
            target_rpm = telemetry["Target"]
            output_rpm = telemetry["OutRPM"]
            error = target_rpm - output_rpm

            telemetry_row = [
                now,
                elapsed_seconds,
                p_gain,
                i_gain,
                target_rpm,
                output_rpm,
                telemetry["MotorRPM"],
                error,
                telemetry["Integral"],
                int(telemetry["Counts"]),
                int(telemetry["dC"]),
                telemetry["PWM"],
                line,
            ]

            telemetry_sheet.write_datetime(row_index, 0, now, timestamp_format)
            for column in range(1, 9):
                telemetry_sheet.write_number(
                    row_index, column, float(telemetry_row[column]), number_format
                )
            telemetry_sheet.write_number(row_index, 9, telemetry_row[9], integer_format)
            telemetry_sheet.write_number(row_index, 10, telemetry_row[10], integer_format)
            telemetry_sheet.write_number(row_index, 11, telemetry_row[11], number_format)
            telemetry_sheet.write_string(row_index, 12, line)

            jmp_sheet.write_number(row_index, 0, p_gain, number_format)
            jmp_sheet.write_number(row_index, 1, i_gain, number_format)
            jmp_sheet.write_number(row_index, 2, error, number_format)

            full_csv_writer.writerow(
                [
                    now.isoformat(timespec="milliseconds"),
                    f"{elapsed_seconds:.3f}",
                    p_gain,
                    i_gain,
                    target_rpm,
                    output_rpm,
                    telemetry["MotorRPM"],
                    error,
                    telemetry["Integral"],
                    int(telemetry["Counts"]),
                    int(telemetry["dC"]),
                    telemetry["PWM"],
                    line,
                ]
            )
            full_csv_file.flush()

            jmp_csv_writer.writerow([p_gain, i_gain, error])
            jmp_csv_file.flush()

            p_values.append(p_gain)
            i_values.append(i_gain)
            output_rpm_values.append(output_rpm)
            error_values.append(error)
            pwm_values.append(telemetry["PWM"])
            row_index += 1

    except KeyboardInterrupt:
        print("\nStopping logger...")
    finally:
        try:
            send_command(serial_port, "STOP")
        except SerialException:
            pass

        serial_port.close()
        full_csv_file.close()
        jmp_csv_file.close()

        sample_count = row_index - 1
        duration_seconds = max(0.0, time.monotonic() - start_monotonic)

        metadata_sheet.write("A15", "Detected firmware", label_format)
        metadata_sheet.write("B15", firmware_line)
        metadata_sheet.write("A16", "Samples recorded", label_format)
        metadata_sheet.write("B16", sample_count)
        metadata_sheet.write("A17", "Logging duration (s)", label_format)
        metadata_sheet.write("B17", duration_seconds)
        metadata_sheet.write("A19", "Startup / status messages", label_format)
        for offset, message in enumerate(startup_messages, start=19):
            metadata_sheet.write(offset, 0, message)

        summary_sheet.set_column("A:A", 32)
        summary_sheet.set_column("B:B", 20)
        summary_sheet.write("A1", "DiscoveryPID Test Summary", title_format)

        summary_values: list[tuple[str, object]] = [
            ("Start time", start_wall_clock.isoformat(timespec="seconds")),
            ("Trial", args.trial),
            ("Samples", sample_count),
            ("Duration (s)", duration_seconds),
        ]

        if sample_count > 0:
            summary_values.extend(
                [
                    ("P", statistics.fmean(p_values)),
                    ("I", statistics.fmean(i_values)),
                    ("Mean output RPM", statistics.fmean(output_rpm_values)),
                    ("Minimum output RPM", min(output_rpm_values)),
                    ("Maximum output RPM", max(output_rpm_values)),
                    (
                        "Output RPM standard deviation",
                        statistics.pstdev(output_rpm_values)
                        if len(output_rpm_values) > 1
                        else 0.0,
                    ),
                    (
                        "Mean absolute RPM error",
                        statistics.fmean(abs(value) for value in error_values),
                    ),
                    ("Mean signed RPM error", statistics.fmean(error_values)),
                    ("Mean PWM command", statistics.fmean(pwm_values)),
                ]
            )

        for summary_row, (label, value) in enumerate(summary_values, start=2):
            summary_sheet.write(summary_row, 0, label, label_format)
            if isinstance(value, (int, float)):
                summary_sheet.write_number(summary_row, 1, float(value), number_format)
            else:
                summary_sheet.write(summary_row, 1, value)

        if sample_count > 0:
            last_excel_row = sample_count
            telemetry_sheet.autofilter(
                0, 0, last_excel_row, len(TELEMETRY_HEADERS) - 1
            )
            jmp_sheet.autofilter(0, 0, last_excel_row, len(JMP_HEADERS) - 1)

            speed_chart = workbook.add_chart({"type": "line"})
            speed_chart.add_series(
                {
                    "name": "Target RPM",
                    "categories": ["Telemetry", 1, 1, last_excel_row, 1],
                    "values": ["Telemetry", 1, 4, last_excel_row, 4],
                    "line": {"width": 1.5},
                }
            )
            speed_chart.add_series(
                {
                    "name": "Output RPM",
                    "categories": ["Telemetry", 1, 1, last_excel_row, 1],
                    "values": ["Telemetry", 1, 5, last_excel_row, 5],
                    "line": {"width": 1.5},
                }
            )
            speed_chart.set_title({"name": "Target vs. Measured Output RPM"})
            speed_chart.set_x_axis({"name": "Elapsed Time (s)"})
            speed_chart.set_y_axis({"name": "RPM"})
            speed_chart.set_legend({"position": "bottom"})
            speed_chart.set_size({"width": 760, "height": 420})
            summary_sheet.insert_chart("D2", speed_chart)

            pwm_chart = workbook.add_chart({"type": "line"})
            pwm_chart.add_series(
                {
                    "name": "PWM Command",
                    "categories": ["Telemetry", 1, 1, last_excel_row, 1],
                    "values": ["Telemetry", 1, 11, last_excel_row, 11],
                    "line": {"width": 1.5},
                }
            )
            pwm_chart.set_title({"name": "PWM Command"})
            pwm_chart.set_x_axis({"name": "Elapsed Time (s)"})
            pwm_chart.set_y_axis({"name": "PWM", "min": -255, "max": 255})
            pwm_chart.set_legend({"none": True})
            pwm_chart.set_size({"width": 760, "height": 360})
            summary_sheet.insert_chart("D24", pwm_chart)

        workbook.close()

    print(f"Saved Excel workbook: {xlsx_path.resolve()}")
    print(f"Saved full CSV:       {full_csv_path.resolve()}")
    print(f"Saved JMP CSV:        {jmp_csv_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

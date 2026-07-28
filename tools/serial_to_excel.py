#!/usr/bin/env python3
"""Log DiscoveryPID serial telemetry to Excel and CSV on the connected PC.

The Arduino Mega cannot directly create an Excel workbook on the computer.
This script runs on the connected Windows PC, reads the Mega's USB serial
output, and saves every telemetry row into:

  data/logs/DiscoveryPID_YYYYMMDD_HHMMSS.xlsx
  data/logs/DiscoveryPID_YYYYMMDD_HHMMSS.csv

The CSV file is flushed after every sample, making it a useful recovery file
and a convenient import format for JMP. The XLSX workbook is finalized when
logging ends normally or when Ctrl+C is pressed.

Examples:
  python tools/serial_to_excel.py --port COM5 --target 100
  python tools/serial_to_excel.py --port COM5 --target 100 --duration 60
  python tools/serial_to_excel.py
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
SCRIPT_VERSION = "1.0"

# Parses the firmware's key=value telemetry format, including negative values.
PAIR_PATTERN = re.compile(
    r"([A-Za-z][A-Za-z0-9_]*)=(-?(?:\d+(?:\.\d*)?|\.\d+))"
)

REQUIRED_FIELDS = {
    "Target",
    "OutRPM",
    "MotorRPM",
    "Error",
    "Integral",
    "Counts",
    "dC",
    "PWM",
}

HEADERS = [
    "PC Timestamp",
    "Elapsed Time (s)",
    "Target RPM",
    "Output RPM",
    "Motor RPM",
    "Error (RPM)",
    "Integral",
    "Encoder Counts",
    "Delta Counts",
    "PWM Command",
    "Raw Serial Line",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save DiscoveryPID serial telemetry to XLSX and CSV files."
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
        "--target",
        type=float,
        help="Optional target output RPM sent to the controller after connection.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Optional logging duration in seconds. Otherwise press Ctrl+C to stop.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional XLSX output path. The CSV backup uses the same file stem.",
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


def build_output_paths(output: Optional[Path]) -> tuple[Path, Path]:
    if output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        xlsx_path = Path("data") / "logs" / f"DiscoveryPID_{timestamp}.xlsx"
    else:
        xlsx_path = output
        if xlsx_path.suffix.lower() != ".xlsx":
            xlsx_path = xlsx_path.with_suffix(".xlsx")

    xlsx_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path = xlsx_path.with_suffix(".csv")
    return xlsx_path, csv_path


def main() -> int:
    args = parse_args()

    try:
        port_name = select_serial_port(args.port)
    except RuntimeError as exc:
        print(f"Port selection error: {exc}", file=sys.stderr)
        return 2

    xlsx_path, csv_path = build_output_paths(args.output)

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

    # Opening an Arduino serial port commonly resets the board. Give setup()
    # time to run before flushing stale data and optionally sending a target.
    time.sleep(2.0)
    serial_port.reset_input_buffer()

    if args.target is not None:
        serial_port.write(f"{args.target}\n".encode("ascii"))
        serial_port.flush()
        print(f"Sent target RPM: {args.target}")

    workbook = xlsxwriter.Workbook(str(xlsx_path))
    telemetry_sheet = workbook.add_worksheet("Telemetry")
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
    number_format = workbook.add_format({"num_format": "0.00"})
    integer_format = workbook.add_format({"num_format": "0"})
    title_format = workbook.add_format(
        {"bold": True, "font_size": 16, "font_color": "#1F4E78"}
    )
    label_format = workbook.add_format({"bold": True, "bg_color": "#D9EAF7"})

    for column, header in enumerate(HEADERS):
        telemetry_sheet.write(0, column, header, header_format)

    telemetry_sheet.freeze_panes(1, 0)
    telemetry_sheet.set_column("A:A", 24)
    telemetry_sheet.set_column("B:B", 16)
    telemetry_sheet.set_column("C:G", 14)
    telemetry_sheet.set_column("H:I", 15)
    telemetry_sheet.set_column("J:J", 14)
    telemetry_sheet.set_column("K:K", 90)

    metadata_sheet.set_column("A:A", 24)
    metadata_sheet.set_column("B:B", 60)
    metadata_sheet.write("A1", "DiscoveryPID Log Metadata", title_format)
    metadata_rows = [
        ("Logger version", SCRIPT_VERSION),
        ("Serial port", port_name),
        ("Baud rate", args.baud),
        ("Logging started", datetime.now().isoformat(timespec="seconds")),
        ("Requested target RPM", args.target if args.target is not None else "Not sent"),
        ("XLSX path", str(xlsx_path.resolve())),
        ("CSV backup path", str(csv_path.resolve())),
    ]
    for row_index, (label, value) in enumerate(metadata_rows, start=2):
        metadata_sheet.write(row_index, 0, label, label_format)
        metadata_sheet.write(row_index, 1, value)

    csv_file = csv_path.open("w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(HEADERS)
    csv_file.flush()

    start_monotonic = time.monotonic()
    start_wall_clock = datetime.now()
    row_index = 1
    output_rpm_values: list[float] = []
    error_values: list[float] = []
    pwm_values: list[float] = []
    firmware_line = "Not detected"
    startup_messages: list[str] = []

    print("Logging telemetry. Press Ctrl+C to stop and finalize the Excel file.")
    print(f"XLSX: {xlsx_path.resolve()}")
    print(f"CSV:  {csv_path.resolve()}")

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

            row = [
                now,
                elapsed_seconds,
                telemetry["Target"],
                telemetry["OutRPM"],
                telemetry["MotorRPM"],
                telemetry["Error"],
                telemetry["Integral"],
                int(telemetry["Counts"]),
                int(telemetry["dC"]),
                telemetry["PWM"],
                line,
            ]

            telemetry_sheet.write_datetime(row_index, 0, now, timestamp_format)
            telemetry_sheet.write_number(row_index, 1, elapsed_seconds, number_format)
            for column in range(2, 7):
                telemetry_sheet.write_number(row_index, column, float(row[column]), number_format)
            telemetry_sheet.write_number(row_index, 7, int(row[7]), integer_format)
            telemetry_sheet.write_number(row_index, 8, int(row[8]), integer_format)
            telemetry_sheet.write_number(row_index, 9, float(row[9]), number_format)
            telemetry_sheet.write_string(row_index, 10, line)

            csv_writer.writerow(
                [
                    now.isoformat(timespec="milliseconds"),
                    f"{elapsed_seconds:.3f}",
                    telemetry["Target"],
                    telemetry["OutRPM"],
                    telemetry["MotorRPM"],
                    telemetry["Error"],
                    telemetry["Integral"],
                    int(telemetry["Counts"]),
                    int(telemetry["dC"]),
                    telemetry["PWM"],
                    line,
                ]
            )
            csv_file.flush()

            output_rpm_values.append(telemetry["OutRPM"])
            error_values.append(telemetry["Error"])
            pwm_values.append(telemetry["PWM"])
            row_index += 1

    except KeyboardInterrupt:
        print("\nStopping logger...")
    finally:
        # Stop the motor before releasing the serial port.
        try:
            serial_port.write(b"0\n")
            serial_port.flush()
            time.sleep(0.1)
        except SerialException:
            pass

        serial_port.close()
        csv_file.close()

        sample_count = row_index - 1
        duration_seconds = max(0.0, time.monotonic() - start_monotonic)

        metadata_sheet.write("A11", "Detected firmware", label_format)
        metadata_sheet.write("B11", firmware_line)
        metadata_sheet.write("A12", "Samples recorded", label_format)
        metadata_sheet.write("B12", sample_count)
        metadata_sheet.write("A13", "Logging duration (s)", label_format)
        metadata_sheet.write("B13", duration_seconds)
        metadata_sheet.write("A15", "Startup / status messages", label_format)
        for offset, message in enumerate(startup_messages, start=15):
            metadata_sheet.write(offset, 0, message)

        summary_sheet.set_column("A:A", 28)
        summary_sheet.set_column("B:B", 18)
        summary_sheet.write("A1", "DiscoveryPID Test Summary", title_format)

        summary_values = [
            ("Start time", start_wall_clock.isoformat(timespec="seconds")),
            ("Samples", sample_count),
            ("Duration (s)", duration_seconds),
        ]

        if sample_count > 0:
            summary_values.extend(
                [
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
            telemetry_sheet.autofilter(0, 0, last_excel_row, len(HEADERS) - 1)

            speed_chart = workbook.add_chart({"type": "line"})
            speed_chart.add_series(
                {
                    "name": "Target RPM",
                    "categories": ["Telemetry", 1, 1, last_excel_row, 1],
                    "values": ["Telemetry", 1, 2, last_excel_row, 2],
                    "line": {"width": 1.5},
                }
            )
            speed_chart.add_series(
                {
                    "name": "Output RPM",
                    "categories": ["Telemetry", 1, 1, last_excel_row, 1],
                    "values": ["Telemetry", 1, 3, last_excel_row, 3],
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
                    "values": ["Telemetry", 1, 9, last_excel_row, 9],
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
    print(f"Saved CSV backup:     {csv_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

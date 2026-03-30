"""Capture Arduino Ohm's law sweeps, export session .dat files, and generate plots.

This script listens to the serial protocol emitted by the Arduino sketch,
keeps only useful sweeps, exports all accepted sweeps of the current session to
a single .dat file, and updates a single scientific plot while the program is
running.

Design choices:
    * Sweeps in the return direction of the potentiometer are discarded.
    * A single session-level .dat file is created per program execution.
    * A single session-level .png plot is updated after each accepted sweep.

Dependencies:
    pip install pyserial matplotlib numpy
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
import serial
from serial import SerialException


@dataclass(slots=True)
class DataPoint:
    """Single measurement point captured from the Arduino."""

    timestamp_ms: int
    sweep_id: int
    v_component_v: float
    i_component_ma: float
    v_total_v: float
    v_shunt_v: float


@dataclass(slots=True)
class SweepBuffer:
    """In-memory buffer for a single sweep."""

    sweep_id: int
    start_timestamp_ms: int
    points: list[DataPoint]


@dataclass(slots=True)
class SessionStore:
    """Data persisted during a single Python execution."""

    session_label: str
    dat_path: Path
    plot_path: Path
    accepted_sweeps: list[SweepBuffer]


@dataclass(slots=True)
class FitResult:
    """Linear fit result for V(I)."""

    current_sorted_ma: np.ndarray
    voltage_sorted: np.ndarray
    fitted_voltage_v: np.ndarray
    slope_v_per_ma: float
    intercept_v: float
    r_squared: float
    estimated_resistance_ohms: float


class SerialPortOpenError(RuntimeError):
    """Raised when the serial port cannot be opened."""


def build_argument_parser() -> argparse.ArgumentParser:
    """Create the command-line interface parser."""

    parser = argparse.ArgumentParser(
        description='Capture Arduino Ohm\'s law data, export .dat files, and plot them.'
    )
    parser.add_argument(
        '--port',
        required=True,
        help='Serial port name, for example COM3 on Windows or /dev/ttyACM0 on Linux.',
    )
    parser.add_argument(
        '--baudrate',
        type=int,
        default=115200,
        help='Serial baud rate. Default: 115200.',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path(__file__).resolve().parent / 'outputs',
        help='Directory in which .dat files and plots will be saved.',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=1.0,
        help='Serial read timeout in seconds. Default: 1.0.',
    )
    parser.add_argument(
        '--direction-threshold-v',
        type=float,
        default=0.15,
        help=(
            'Minimum net voltage increase required to accept a sweep as a forward '
            'measurement. Default: 0.15 V.'
        ),
    )
    return parser


def ensure_output_dir(output_dir: Path) -> Path:
    """Create the output directory if necessary and return it."""

    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def open_serial_connection(port: str, baudrate: int, timeout: float) -> serial.Serial:
    """Open the serial port with grounded error handling."""

    try:
        connection = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(2.0)
        connection.reset_input_buffer()
        return connection
    except SerialException as exc:
        message = str(exc).lower()

        if 'permissionerror' in message or 'access is denied' in message or 'acesso negado' in message:
            raise SerialPortOpenError(
                'open_serial_connection() could not open the serial port because it is already '
                'in use. Feche o Monitor Serial, o Serial Plotter e qualquer outro programa que '
                f'esteja usando a porta {port!r}, e tente novamente. Erro original: {exc}'
            ) from exc

        raise SerialPortOpenError(
            'open_serial_connection() failed to open the serial port. '
            f'Port={port!r}, baudrate={baudrate}. Original error: {exc}'
        ) from exc


def parse_data_line(parts: list[str]) -> DataPoint:
    """Parse a structured DATA line emitted by the Arduino sketch."""

    if len(parts) != 7:
        raise ValueError(
            'parse_data_line() expected 7 fields in a DATA line, '
            f'but received {len(parts)} fields: {parts!r}'
        )

    return DataPoint(
        timestamp_ms=int(parts[1]),
        sweep_id=int(parts[2]),
        v_component_v=float(parts[3]),
        i_component_ma=float(parts[4]),
        v_total_v=float(parts[5]),
        v_shunt_v=float(parts[6]),
    )


def create_session_store(output_dir: Path) -> SessionStore:
    """Create the session-level file paths and initialize the .dat file header."""

    session_label = datetime.now().strftime('%Y%m%d_%H%M%S')
    dat_path = output_dir / f'sessao_ohm_{session_label}.dat'
    plot_path = output_dir / f'sessao_ohm_{session_label}.png'

    with dat_path.open('w', encoding='utf-8') as file:
        file.write('# Sessao de medidas da Lei de Ohm capturada via serial do Arduino\n')
        file.write(f'# session_label={session_label}\n')
        file.write('# columns: timestamp_ms sweep_id V_component_V I_component_mA V_total_V V_shunt_V\n')

    return SessionStore(
        session_label=session_label,
        dat_path=dat_path,
        plot_path=plot_path,
        accepted_sweeps=[],
    )


def append_sweep_to_dat_file(dat_path: Path, sweep: SweepBuffer) -> None:
    """Append one accepted sweep to the session .dat file."""

    with dat_path.open('a', encoding='utf-8') as file:
        file.write(f'\n# sweep_id={sweep.sweep_id}\n')
        file.write(f'# start_timestamp_ms={sweep.start_timestamp_ms}\n')

        for point in sweep.points:
            file.write(
                f'{point.timestamp_ms}\t{point.sweep_id}\t'
                f'{point.v_component_v:.6f}\t{point.i_component_ma:.6f}\t'
                f'{point.v_total_v:.6f}\t{point.v_shunt_v:.6f}\n'
            )


def compute_linear_fit(points: Iterable[DataPoint]) -> FitResult:
    """Compute a first-degree fit for V(I) and return fit-related arrays and metrics."""

    points_list = list(points)

    if len(points_list) < 2:
        raise ValueError(
            'compute_linear_fit() requires at least two points to compute a linear fit.'
        )

    voltage = np.array([point.v_component_v for point in points_list], dtype=float)
    current_ma = np.array([point.i_component_ma for point in points_list], dtype=float)

    sort_indices = np.argsort(current_ma)
    current_sorted_ma = current_ma[sort_indices]
    voltage_sorted = voltage[sort_indices]

    coefficients = np.polyfit(current_sorted_ma, voltage_sorted, deg=1)
    fitted_voltage_v = np.polyval(coefficients, current_sorted_ma)

    residual_sum_of_squares = float(np.sum((voltage_sorted - fitted_voltage_v) ** 2))
    total_sum_of_squares = float(np.sum((voltage_sorted - np.mean(voltage_sorted)) ** 2))
    r_squared = 1.0 - (residual_sum_of_squares / total_sum_of_squares) if total_sum_of_squares > 0.0 else 1.0

    slope_v_per_ma = float(coefficients[0])
    intercept_v = float(coefficients[1])

    estimated_resistance_ohms = slope_v_per_ma * 1000.0

    return FitResult(
        current_sorted_ma=current_sorted_ma,
        voltage_sorted=voltage_sorted,
        fitted_voltage_v=fitted_voltage_v,
        slope_v_per_ma=slope_v_per_ma,
        intercept_v=intercept_v,
        r_squared=r_squared,
        estimated_resistance_ohms=estimated_resistance_ohms,
    )


def estimate_direction_delta_v(points: list[DataPoint]) -> float:
    """Estimate the net sweep direction using averaged endpoints."""

    if len(points) < 2:
        raise ValueError(
            'estimate_direction_delta_v() requires at least two points.'
        )

    window_size = max(1, min(5, len(points) // 4 if len(points) >= 4 else 1))
    start_mean_v = float(np.mean([point.v_component_v for point in points[:window_size]]))
    end_mean_v = float(np.mean([point.v_component_v for point in points[-window_size:]]))
    return end_mean_v - start_mean_v



def is_forward_sweep(sweep: SweepBuffer, direction_threshold_v: float) -> tuple[bool, float]:
    """Return whether the sweep is a valid forward sweep and its net voltage delta."""

    delta_v = estimate_direction_delta_v(sweep.points)
    return delta_v >= direction_threshold_v, delta_v



def flatten_points(sweeps: Iterable[SweepBuffer]) -> list[DataPoint]:
    """Flatten all accepted sweeps into a single point list."""

    points: list[DataPoint] = []

    for sweep in sweeps:
        points.extend(sweep.points)

    return points



def update_session_plot(session_store: SessionStore) -> Path:
    """Regenerate the session plot with all accepted sweeps overlaid."""

    all_points = flatten_points(session_store.accepted_sweeps)

    if len(all_points) < 2:
        raise ValueError(
            'update_session_plot() requires at least two accepted data points.'
        )

    fit_result = compute_linear_fit(all_points)

    figure, axis = plt.subplots(figsize=(8.5, 5.5), dpi=160)

    for sweep_index, sweep in enumerate(session_store.accepted_sweeps, start=1):
        sweep_fit = compute_linear_fit(sweep.points)
        axis.plot(
            sweep_fit.current_sorted_ma,
            sweep_fit.voltage_sorted,
            marker='o',
            linestyle='-',
            label=f'Medida {sweep_index}',
        )

    axis.plot(
        fit_result.current_sorted_ma,
        fit_result.fitted_voltage_v,
        linestyle='--',
        linewidth=2.0,
        label='Ajuste linear global',
    )
    axis.set_title('Curvas características I-V')
    axis.set_xlabel('Intensidade de corrente elétrica (mA)')
    axis.set_ylabel('Tensão no componente (V)')
    axis.grid(True)
    axis.legend()

    text_lines = [
        f'Medidas aceitas = {len(session_store.accepted_sweeps)}',
        f'R² global = {fit_result.r_squared:.6f}',
        f'Intercepto global = {fit_result.intercept_v:.6f} V',
    ]

    if np.isfinite(fit_result.estimated_resistance_ohms):
        text_lines.append(f'Resistência estimada = {fit_result.estimated_resistance_ohms:.3f} Ω')
    else:
        text_lines.append('Resistência estimada = indefinida')

    axis.text(
        0.02,
        0.98,
        '\n'.join(text_lines),
        transform=axis.transAxes,
        verticalalignment='top',
        bbox={'boxstyle': 'round', 'alpha': 0.15},
    )

    figure.tight_layout()
    figure.savefig(session_store.plot_path)
    plt.close(figure)

    return session_store.plot_path



def accept_and_persist_sweep(
    session_store: SessionStore,
    sweep: SweepBuffer,
    direction_threshold_v: float,
) -> tuple[bool, str]:
    """Validate a sweep, persist it if accepted, and update the session plot."""

    if len(sweep.points) < 2:
        return False, (
            'Varredura descartada porque possui poucos pontos para uma medida útil. '
            f'sweep_id={sweep.sweep_id}, pontos={len(sweep.points)}'
        )

    is_valid_forward_sweep, delta_v = is_forward_sweep(
        sweep=sweep,
        direction_threshold_v=direction_threshold_v,
    )

    if not is_valid_forward_sweep:
        return False, (
            'Varredura descartada por parecer ser um retorno do potenciômetro '
            f'ou uma variação insuficiente. sweep_id={sweep.sweep_id}, ΔV={delta_v:.4f} V'
        )

    append_sweep_to_dat_file(dat_path=session_store.dat_path, sweep=sweep)
    session_store.accepted_sweeps.append(sweep)
    update_session_plot(session_store=session_store)

    return True, (
        f'Varredura aceita. sweep_id={sweep.sweep_id}, '
        f'ΔV={delta_v:.4f} V, medidas acumuladas={len(session_store.accepted_sweeps)}'
    )



def run_capture_loop(port: str, baudrate: int, timeout: float, output_dir: Path, direction_threshold_v: float) -> None:
    """Listen to the serial port, capture sweeps, and save them to disk."""

    output_dir = ensure_output_dir(output_dir)
    current_sweep: SweepBuffer | None = None
    session_store = create_session_store(output_dir=output_dir)

    with open_serial_connection(port=port, baudrate=baudrate, timeout=timeout) as serial_connection:
        print(f'Conectado a {port} em {baudrate} baud.')
        print(f'Diretório de saída: {output_dir}')
        print(f'Arquivo .dat da sessão: {session_store.dat_path}')
        print(f'Gráfico da sessão: {session_store.plot_path}')
        print('Aguardando movimento do potenciômetro... Pressione Ctrl+C para encerrar.')

        while True:
            raw_line = serial_connection.readline()

            if not raw_line:
                continue

            try:
                line = raw_line.decode('utf-8', errors='replace').strip()
            except UnicodeDecodeError as exc:
                raise RuntimeError(
                    'run_capture_loop() failed to decode a serial line as UTF-8.'
                ) from exc

            if not line:
                continue

            if line.startswith('#'):
                print(line)
                continue

            parts = line.split('\t')
            message_type = parts[0]

            if message_type == 'START':
                if len(parts) != 3:
                    print(f'Ignorando linha START malformada: {line}')
                    continue

                current_sweep = SweepBuffer(
                    sweep_id=int(parts[2]),
                    start_timestamp_ms=int(parts[1]),
                    points=[],
                )
                print(f'Varredura {current_sweep.sweep_id} iniciada.')
                continue

            if message_type == 'DATA':
                if current_sweep is None:
                    print(f'Ignorando linha DATA recebida antes de START: {line}')
                    continue

                point = parse_data_line(parts)

                if point.sweep_id != current_sweep.sweep_id:
                    print(
                        'Ignorando linha DATA com identificador incompatível: '
                        f'atual={current_sweep.sweep_id}, recebido={point.sweep_id}'
                    )
                    continue

                current_sweep.points.append(point)
                continue

            if message_type == 'END':
                if current_sweep is None:
                    print(f'Ignorando linha END recebida antes de START: {line}')
                    continue

                if len(parts) != 4:
                    print(f'Ignorando linha END malformada: {line}')
                    continue

                received_sweep_id = int(parts[2])

                if received_sweep_id != current_sweep.sweep_id:
                    print(
                        'Ignorando linha END com identificador incompatível: '
                        f'atual={current_sweep.sweep_id}, recebido={received_sweep_id}'
                    )
                    continue

                accepted, message = accept_and_persist_sweep(
                    session_store=session_store,
                    sweep=current_sweep,
                    direction_threshold_v=direction_threshold_v,
                )
                print(message)

                if accepted:
                    print(f'Dados acumulados em: {session_store.dat_path}')
                    print(f'Gráfico atualizado em: {session_store.plot_path}')

                current_sweep = None
                continue

            print(f'Ignorando linha desconhecida: {line}')



def main() -> int:
    """Program entry point."""

    parser = build_argument_parser()
    arguments = parser.parse_args()

    try:
        run_capture_loop(
            port=arguments.port,
            baudrate=arguments.baudrate,
            timeout=arguments.timeout,
            output_dir=arguments.output_dir,
            direction_threshold_v=arguments.direction_threshold_v,
        )
    except KeyboardInterrupt:
        print('\nCaptura interrompida pelo usuário.')
        return 0
    except Exception as exc:  # noqa: BLE001 - deliberate top-level boundary.
        print(f'Erro: {exc}', file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    raise SystemExit(main())

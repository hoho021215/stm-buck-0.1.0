"""Run the 0.1.0 host controller against the switched RL-C model."""
import csv
import ctypes as c
import json
from pathlib import Path
import sys

from plant import Plant


# 실측값은 이 구역에 입력하세요. 기본값은 예시 회로 조건입니다.
PLANT_PARAMETERS = {
    'L': 470e-6,      # H, 인덕턴스 (470 µH)
    'C': 220e-6,      # F, 출력에 병렬로 연결된 전체 유효 정전용량 (220 µF)
    'dcr': 0.10,      # Ω, 인덕터 직류저항
    'esr': 0.04,      # Ω, 출력 커패시터 뱅크의 등가 직렬저항
    'load': 18.75,    # Ω, 출력 저항 부하
}
VIN_BEFORE_V = 29.0  # V, 계단 입력 전 전압
VIN_AFTER_V = 30.0   # V, 계단 입력 후 전압


class Capture(c.Structure):
    _fields_ = [
        (name, c.c_uint32) for name in ('status', 'samples', 'active_features', 'rejected_requests')
    ] + [
        (name, c.c_float)
        for name in ('vout_min_v', 'vout_max_v', 'mae_v', 'peak_abs_il_a', 'duty_min', 'duty_max')
    ]


def main():
    here = Path(__file__).resolve().parent
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else here / 'out'
    out.mkdir(parents=True, exist_ok=True)
    products = ('waveforms.csv', 'summary.json', 'parameters.json', 'buck_line_step.pdf', 'buck_line_step.png')
    if any((out / name).exists() for name in products):
        print(f'기존 결과가 {out}에 있습니다. 이번 실행이 실패하면 이전 결과가 남을 수 있습니다. '
              '조건별 결과를 보관하려면 새 출력 폴더를 지정하세요.', file=sys.stderr)

    lib = c.CDLL(str(here / 'libbuck_sim.so'))
    lib.sim_init.argtypes = [c.c_uint32]
    lib.sim_init.restype = None
    lib.sim_feed.argtypes = [c.c_float] * 3 + [c.c_uint32] * 2
    lib.sim_feed.restype = c.c_uint32
    measurement = Capture.in_dll(lib, 'buck_measurement')
    frame_step = 150000
    trials = {}
    rows = []
    for mask in (7, 6):
        lib.sim_init(mask)
        plant = Plant(**PLANT_PARAMETERS)
        ticks = 0
        enabled = False
        before_step = []
        for n in range(frame_step + 2502):
            vin = VIN_BEFORE_V if n < frame_step else VIN_AFTER_V
            duty = ticks / 1440.0
            vs, il = plant.cycle(vin, duty, enabled)
            if n == 2048 and not lib.sim_calibrated():
                raise RuntimeError(f'zero calibration failed (mask={mask})')
            if n == frame_step and not (lib.sim_monitoring() and lib.sim_active() == mask):
                raise RuntimeError(f'run qualification failed (mask={mask})')
            ticks = lib.sim_feed(vin, vs, il, int(n == 2048), int(n == frame_step))
            enabled = lib.sim_gate() != 0
            faults = lib.sim_fault()
            if faults:
                raise RuntimeError(f'fault {faults} at t={n / 50000:.6f}s mask={mask}, '
                                   f'Vout={vs:.6f}, IL={il:.6f}')
            if n >= frame_step - 250:
                rows.append((mask, (n - frame_step) * 20 + 5, vin, vs, il, duty))
                if n < frame_step:
                    before_step.append(vs)
        result = {name: getattr(measurement, name) for name, _ in measurement._fields_}
        progress = tuple(result[key] for key in ('samples', 'status', 'active_features', 'rejected_requests'))
        if progress != (2500, 2, mask, 0):
            raise RuntimeError(f'incomplete firmware measurement (mask={mask}): {result}')
        result['before_step_mean_model_v'] = sum(before_step) / len(before_step)
        result['final_faults'] = lib.sim_fault()
        trials[mask] = result

    conditions = {
        'plant': PLANT_PARAMETERS,
        'vin_before_v': VIN_BEFORE_V,
        'vin_after_v': VIN_AFTER_V,
        'pwm_period_s': Plant.PERIOD,
        'sample_at_s': Plant.SAMPLE_AT,
    }
    with (out / 'waveforms.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['feature_mask', 'relative_sample_time_us', 'vin_V', 'vout_model_V',
                         'iL_model_A', 'applied_pwm_duty'])
        writer.writerows(rows)
    (out / 'summary.json').write_text(json.dumps(trials, indent=2) + '\n')
    (out / 'parameters.json').write_text(json.dumps(conditions, indent=2) + '\n')
    print(json.dumps(trials, indent=2))


if __name__ == '__main__':
    main()

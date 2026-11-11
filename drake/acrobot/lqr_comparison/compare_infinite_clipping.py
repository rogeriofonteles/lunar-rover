"""Reproduce six nonlinear Drake runs and an infinite-horizon LQR PDF report."""
import argparse
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.systems.analysis import Simulator

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from acrobot_test import simulation_options
from lqr import wrapped_state_error, _linear_model, infinite_horizon_lqr, control_torque

EQUILIBRIUM = np.array([math.pi, 0., 0., 0.])


def errors(states):
    return wrapped_state_error(states, EQUILIBRIUM)


def simulate(gain, error, clipped, dt, steps, limit):
    plant = AcrobotPlant()
    simulator = Simulator(plant)
    simulator.get_mutable_integrator().set_target_accuracy(1e-8)
    context = simulator.get_mutable_context()
    context.SetContinuousState(EQUILIBRIUM + [-error, 0., 0., 0.])
    port = plant.get_input_port(0)
    port.FixValue(context, [0.])
    simulator.Initialize()
    times = np.arange(steps + 1) * dt
    states = [plant.get_output_port(0).Eval(context).copy()]
    requested, applied = [], []
    for time in times[1:]:
        deviation = errors(states[-1])
        requested.append(control_torque(gain, deviation))
        applied.append(control_torque(gain, deviation, limit if clipped else None))
        port.FixValue(context, [applied[-1]])
        simulator.AdvanceTo(time)
        states.append(plant.get_output_port(0).Eval(context).copy())
    states = np.asarray(states)
    if not np.all(np.isfinite(states)):
        raise RuntimeError('Simulation produced nonfinite states')
    return times, states, errors(states), np.asarray(requested), np.asarray(applied)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).parents[2] / 'output/pdf')
    parser.add_argument('--timestep', type=float)
    parser.add_argument('--steps', type=int)
    args = parser.parse_args()
    options = simulation_options(timestep=args.timestep, steps=args.steps)
    dt, steps, limit = (options[k] for k in ('timestep', 'steps', 'torque_limit_nm'))
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    plant = AcrobotPlant()
    gain = infinite_horizon_lqr(plant, timestep=dt)
    A, B, Q = _linear_model(plant, dt, None, 1., EQUILIBRIUM)
    design_poles = np.linalg.eigvals(A - B @ gain)
    augmented = np.zeros((5, 5))
    augmented[:4, :4] = (A - np.eye(4)) / dt
    augmented[:4, 4:] = B / dt
    values, vectors = np.linalg.eig(augmented)
    exact = np.real_if_close(vectors @ np.diag(np.exp(values * dt)) @ np.linalg.inv(vectors))
    sampled_poles = np.linalg.eigvals(exact[:4, :4] - exact[:4, 4:] @ gain)
    records = []
    all_results = {}
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False})
    for error in (0.05, 0.25, 0.6):
        for clipped in (True, False):
            result = simulate(gain, error, clipped, dt, steps, limit)
            all_results[error, clipped] = result
            times, states, deviations, requested, applied = result
            name = f'wrapped_shoulder_error_{error:g}_{"clipped" if clipped else "unclipped"}'
            np.savetxt(output / f'{name}.csv',
                       np.column_stack((times, states, deviations,
                                        np.r_[requested, np.nan], np.r_[applied, np.nan])),
                       delimiter=',', header='time_s,theta1_rad,theta2_rad,theta1dot_rad_s,theta2dot_rad_s,error_theta1_rad,error_theta2_rad,error_theta1dot_rad_s,error_theta2dot_rad_s,requested_torque_Nm,applied_torque_Nm', comments='')
            records.append(dict(error_rad=error, clipping=clipped, gain=gain.tolist(),
                                peak_requested_torque=float(np.max(np.abs(requested))),
                                peak_applied_torque=float(np.max(np.abs(applied))),
                                saturated_steps=int(np.count_nonzero(np.abs(requested) > limit)) if clipped else 0,
                                final_angle_error_rad=deviations[-1, :2].tolist(),
                                final_velocity_rad_s=deviations[-1, 2:].tolist()))
    def complex_pairs(values):
        return [[float(v.real), float(v.imag)] for v in values]
    metadata = dict(controller='infinite horizon', timestep_s=dt, steps=steps,
                    torque_limit_Nm=limit, Q=Q.tolist(), R=1.,
                    initial_state_convention='[pi-e, 0, 0, 0]',
                    angular_error_interval='[-pi, pi)',
                    design_poles=complex_pairs(design_poles),
                    exact_sampled_poles=complex_pairs(sampled_poles), runs=records)
    (output / 'infinite_lqr_wrapped_shoulder_results.json').write_text(json.dumps(metadata, indent=2))
    pdf_path = output / 'infinite_lqr_wrapped_shoulder_comparison.pdf'
    with PdfPages(pdf_path) as pdf:
        fig = plt.figure(figsize=(11.7, 8.3))
        fig.text(.07, .92, 'Acrobot | Infinite-horizon LQR', size=23, weight='bold')
        lines = [f'Six nonlinear Drake simulations | {steps*dt:g} s | control period {dt:g} s',
                 f'Initial state: [pi - e, 0, 0, 0], e = 0.05, 0.25, 0.6 rad.',
                 f'Clipped torque: +/-{limit:g} N m. Unclipped: no torque limit.',
                 'Q = diag(10, 10, 1, 1); R = 1; u = -K error (angles wrapped to [-pi, pi)).',
                 'One constant gain is used for every run; saturation does not redesign K.',
                 'K = ' + np.array2string(gain.ravel(), precision=6), '',
                 'Local closed-loop eigenvalues (discrete time):',
                 'Euler design A - BK: ' + ', '.join(f'{p:.6f}' for p in design_poles),
                 'Exact zero-order hold: ' + ', '.join(f'{p:.6f}' for p in sampled_poles),
                 f'Spectral radii: design {max(abs(design_poles)):.6f}; exact {max(abs(sampled_poles)):.6f}.',
                 'These poles apply near upright, where clipping is inactive.',
                 'Saturated nonlinear motion has no single fixed closed-loop pole set.']
        fig.text(.07, .85, '\n'.join(lines), va='top', linespacing=1.65, size=10)
        table_rows = [[f'{r["error_rad"]:g}', 'On' if r['clipping'] else 'Off',
                       f'{r["peak_applied_torque"]:.3f}', str(r['saturated_steps']),
                       f'{np.linalg.norm(r["final_angle_error_rad"]):.3g}',
                       f'{np.linalg.norm(r["final_velocity_rad_s"]):.3g}'] for r in records]
        ax = fig.add_axes([.07, .08, .86, .25]); ax.axis('off')
        table = ax.table(cellText=table_rows, colLabels=['e (rad)', 'Clipping', 'Peak |u| (N m)',
                        'Clipped steps', 'Final angle norm\n(rad)', 'Final speed norm\n(rad/s)'], loc='center', cellLoc='center')
        table.auto_set_font_size(False); table.set_fontsize(9); table.scale(1, 1.65)
        fig.text(.93, .025, '1 / 4', ha='right', size=9)
        pdf.savefig(fig); plt.close(fig)
        for page, error in enumerate((0.05, 0.25, 0.6), 2):
            fig, axes = plt.subplots(3, 2, figsize=(11.7, 8.3), layout='constrained')
            fig.suptitle(f'Initial error e = {error:g} rad | Infinite-horizon LQR', fontsize=18)
            for clipped, color, label in ((True, '#cc6b25', 'Clipped'), (False, '#236ea5', 'Unclipped')):
                times, states, deviations, requested, applied = all_results[error, clipped]
                for i, ax in enumerate(axes.flat[:4]):
                    ax.plot(times, deviations[:, i], color=color, label=label, lw=1.5)
                axes.flat[4].stairs(applied, times, color=color, label=label, lw=1.4)
                axes.flat[5].stairs(requested, times, color=color, label=label, lw=1.4)
            for ax, title, unit in zip(axes.flat,
                ['Shoulder angle error (wrapped)', 'Elbow angle error (wrapped)',
                 'Shoulder angular velocity', 'Elbow angular velocity', 'Applied torque', 'Requested torque before clipping'],
                ['rad', 'rad', 'rad/s', 'rad/s', 'N m', 'N m']):
                ax.set(title=title, xlabel='Time (s)', ylabel=unit, xlim=(0, steps*dt))
                ax.grid(alpha=.25); ax.legend(fontsize=8)
            for ax in axes.flat[4:]:
                for bound in (-limit, limit):
                    ax.axhline(bound, color='gray', ls='--', lw=.8)
            fig.savefig(output / f'wrapped_shoulder_error_{error:g}_comparison.png', dpi=160)
            pdf.savefig(fig); plt.close(fig)
    print(pdf_path)
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()

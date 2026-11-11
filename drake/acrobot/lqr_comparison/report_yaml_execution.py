"""Run current lqr.yaml and export threshold-highlighted plots and samples."""
import json
from pathlib import Path
import sys
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from pydrake.examples import AcrobotPlant
from pydrake.systems.analysis import Simulator
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from acrobot_test import simulation_options
from lqr import controller_gains, wrapped_state_error, control_torque


def main():
    options = simulation_options()
    output = Path(__file__).resolve().parents[2] / 'output/pdf'
    output.mkdir(parents=True, exist_ok=True)
    prefix = output / 'current_yaml_execution'
    source = Path(__file__).resolve().parents[1] / 'lqr.yaml'
    prefix.with_suffix('.yaml').write_text(source.read_text())
    plant = AcrobotPlant()
    simulator = Simulator(plant)
    context = simulator.get_mutable_context()
    context.SetContinuousState(options['initial_state'])
    port = plant.get_input_port(0)
    port.FixValue(context, [0.])
    dt = options['timestep']
    steps = min(options['steps'], options['horizon']) if options['mode'] == 'finite' else options['steps']
    gains = controller_gains(plant, mode=options['mode'], timestep=dt, horizon=options['horizon'])
    equilibrium = np.array([math.pi, 0., 0., 0.])
    simulator.Initialize()
    times = np.arange(steps + 1)*dt
    states = [plant.get_output_port(0).Eval(context).copy()]
    torques = []
    for step in range(steps):
        deviation = wrapped_state_error(states[-1], equilibrium)
        gain = gains[step] if options['mode'] == 'finite' else gains[0]
        torque = control_torque(gain, deviation, options['torque_limit_nm'] if options['clip_torque'] else None)
        port.FixValue(context, [torque])
        torques.append(torque)
        simulator.AdvanceTo(times[step+1])
        states.append(plant.get_output_port(0).Eval(context).copy())
    deviations = wrapped_state_error(states, equilibrium)
    torques = np.asarray(torques)
    assert np.all(np.isfinite(deviations)) and np.all(np.isfinite(torques))
    masks = np.abs(deviations) < [.05, .05, .1, .1]
    np.savetxt(prefix.with_suffix('.csv'), np.column_stack((times, deviations, np.r_[torques, np.nan], masks.astype(int))),
               delimiter=',', comments='', header='time_s,shoulder_error_rad,elbow_error_rad,shoulder_velocity_rad_s,elbow_velocity_rad_s,applied_elbow_torque_Nm,shoulder_error_below_0.05,elbow_error_below_0.05,shoulder_speed_below_0.1,elbow_speed_below_0.1')
    metadata = {k: v.tolist() if isinstance(v, np.ndarray) else v for k,v in options.items()}
    metadata['gains'] = gains.tolist()
    metadata['final_error_and_velocities'] = deviations[-1].tolist()
    metadata['peak_applied_torque_Nm'] = float(np.max(np.abs(torques)))
    prefix.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    with PdfPages(prefix.with_suffix('.pdf')) as pdf:
        for page, end in enumerate((times[-1], min(2., times[-1])), 1):
            fig = plt.figure(figsize=(11.7, 8.3), layout='constrained')
            grid = fig.add_gridspec(3, 2)
            axes = [fig.add_subplot(grid[i,j]) for i in range(2) for j in range(2)]
            torque_ax = fig.add_subplot(grid[2,:])
            fig.text(.5, .96, f'Acrobot | Current YAML execution | {"Full run" if page == 1 else "First 2 seconds"}', fontsize=18, ha='center')
            titles = ['Shoulder angle error', 'Elbow angle error', 'Shoulder angular velocity', 'Elbow angular velocity']
            for i, ax in enumerate(axes):
                threshold = .05 if i < 2 else .1
                ax.axhspan(-threshold, threshold, color='#2a9d63', alpha=.10)
                ax.plot(times, deviations[:,i], color='#506c8a', lw=1.3, label='Trajectory')
                ax.plot(times, np.ma.masked_where(~masks[:,i], deviations[:,i]), color='#008844', lw=2.6,
                        label=f'Absolute value < {threshold:g}')
                for bound in (-threshold, threshold): ax.axhline(bound, color='#008844', ls='--', lw=.8)
                ax.set(title=titles[i], ylabel='rad' if i<2 else 'rad/s', xlabel='Time (s)', xlim=(0,end))
                ax.grid(alpha=.2); ax.legend(fontsize=8, loc='best')
            torque_ax.stairs(torques, times, color='#b46029', lw=1.5, label='Applied elbow torque')
            if options['clip_torque']:
                for bound in (-options['torque_limit_nm'],options['torque_limit_nm']):
                    torque_ax.axhline(bound, color='gray', ls='--', lw=.8)
            torque_ax.set(title='Applied torque (shoulder is unactuated)', ylabel='N m', xlabel='Time (s)', xlim=(0,end))
            torque_ax.grid(alpha=.2); torque_ax.legend(fontsize=8)
            fig.get_layout_engine().set(rect=(0,.09,1,.83))
            config = f"Mode: {options['mode']} | dt: {dt:g} s | duration: {times[-1]:g} s | clipping: {options['clip_torque']} | limit: {options['torque_limit_nm']:g} N m"
            fig.text(.04,.061,config,fontsize=9)
            fig.text(.04,.038,f"Initial state: {options['initial_state'].tolist()} | Angular errors wrapped to [-pi, pi).",fontsize=9)
            fig.text(.04,.016,'Green curves mark qualifying samples; pale bands show tolerance. Signed velocity is plotted; speed = absolute velocity.',fontsize=8)
            fig.text(.97,.016,f'{page}/2',ha='right',fontsize=8)
            pdf.savefig(fig); plt.close(fig)
    print(prefix.with_suffix('.pdf'))
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()

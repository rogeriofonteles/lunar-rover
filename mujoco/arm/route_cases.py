"""Reproducible tracking experiments; disturbances belong only to the plant."""
from copy import deepcopy

import mujoco
import numpy as np

from .model import ArmModel


CASES = {
    'nominal': dict(label='01 · Nominal', speed=.65, acceleration=1.2,
        description='Referência suave, modelo conhecido e limites de torque folgados.',
        focus='Compare o erro em milímetros com o custo por comando. Curvas sobrepostas são um resultado: a tarefa pode não justificar um controlador mais caro.'),
    'fast': dict(label='02 · Movimento rápido', speed=1.8, acceleration=5.,
        description='Mesmo caminho geométrico, com limites de velocidade e aceleração maiores.',
        focus='Olhe os picos de erro nas acelerações e frenagens. Compare o erro articular, não apenas as posições quase sobrepostas.'),
    'torque': dict(label='03 · Torque limitado', speed=1.8, acceleration=5.,
        torque_limit=[3., 29., 20., 2.2, 2., .6], allow_infeasible=True,
        description='A referência rápida é mantida, mas os motores têm limites mais restritos. O torque nominal pode exceder a capacidade disponível.',
        focus='Compare com Movimento rápido para isolar os limites. Observe a utilização de torque, o tempo em saturação e o erro. MPC antecipa limites; LQR satura o comando depois de calculá-lo. Isso não garante vantagem em toda rota.'),
    'payload': dict(label='04 · Carga desconhecida', speed=.65, acceleration=1.2, payload_kg=.5,
        description='Massa adicional de 0,5 kg na ponta, presente desde o início e desconhecida pelos controladores.',
        focus='Procure erro persistente mesmo nas pausas. Todos continuam usando o braço sem carga em suas previsões; nenhum método ganha um estimador ou ação integral neste teste.'),
    'disturbance': dict(label='05 · Perturbação externa', speed=.65, acceleration=1.2,
        pulse=dict(start_s=6., duration_s=.3, torque_nm=[6., -10., 5., 0., 0., 0.]),
        description='Entre 6,0 e 6,3 s, torques externos de +6, −10 e +5 N·m atuam nas juntas 1, 2 e 3. A perturbação não é anunciada ao controlador.',
        focus='Use a janela do evento para comparar pico de erro e recuperação após 6,3 s. O tempo de recuperação exige erro abaixo de 5 mm por pelo menos 0,5 s.'),
}


def case_settings(name='nominal'):
    if name not in CASES:
        raise ValueError(f'Caso desconhecido: {name}; escolha {", ".join(CASES)}')
    return dict(key=name, dwell=.6, **deepcopy(CASES[name]))


def make_plant(config):
    """Independent model: unknown parameters must never leak into the controller."""
    plant=ArmModel(config['timestep'], config['physics_timestep'], config['torque_limit'])
    mass=config.get('experiment', {}).get('payload_kg', 0.)
    if mass:
        model=plant.model
        body=model.body('tool_roll').id
        old_mass=model.body_mass[body]
        old_center=model.body_ipos[body].copy()
        position=model.site_pos[model.site('button_tip').id].copy()
        center=(old_mass*old_center+mass*position)/(old_mass+mass)
        rotation=np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.body_iquat[body])
        rotation=rotation.reshape(3,3)
        inertia=rotation@np.diag(model.body_inertia[body])@rotation.T
        for m,delta in ((old_mass,old_center-center),(mass,position-center)):
            inertia+=m*((delta@delta)*np.eye(3)-np.outer(delta,delta))
        # Payload modeled as a 3 cm radius sphere, rigidly attached at the tip.
        inertia+=np.eye(3)*(.4*mass*.03**2)
        values,axes=np.linalg.eigh(inertia)
        if np.linalg.det(axes)<0:
            axes[:,0]*=-1
        model.body_mass[body]=old_mass+mass
        model.body_ipos[body]=center
        model.body_inertia[body]=values
        mujoco.mju_mat2Quat(model.body_iquat[body], axes.ravel())
        mujoco.mj_setConst(model, plant.data)
        mujoco.mj_resetData(model, plant.data)
    return plant


def external_torque(experiment, time_s):
    pulse=experiment.get('pulse')
    if pulse and pulse['start_s']-1e-10 <= time_s < pulse['start_s']+pulse['duration_s']-1e-10:
        return np.asarray(pulse['torque_nm'],float)
    return np.zeros(6)

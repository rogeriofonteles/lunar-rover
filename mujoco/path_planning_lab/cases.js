window.ROUTE_CASES = {
  "nominal": {
    "key": "nominal",
    "dwell": 0.6,
    "label": "01 · Nominal",
    "speed": 0.65,
    "acceleration": 1.2,
    "description": "Referência suave, modelo conhecido e limites de torque folgados.",
    "focus": "Compare o erro em milímetros com o custo por comando. Curvas sobrepostas são um resultado: a tarefa pode não justificar um controlador mais caro.",
    "data": "data.js?v=2",
    "folder": ""
  },
  "fast": {
    "key": "fast",
    "dwell": 0.6,
    "label": "02 · Movimento rápido",
    "speed": 1.8,
    "acceleration": 5.0,
    "description": "Mesmo caminho geométrico, com limites de velocidade e aceleração maiores.",
    "focus": "Olhe os picos de erro nas acelerações e frenagens. Compare o erro articular, não apenas as posições quase sobrepostas.",
    "data": "cases/fast/data.js?v=2",
    "folder": "cases/fast/"
  },
  "torque": {
    "key": "torque",
    "dwell": 0.6,
    "label": "03 · Torque limitado",
    "speed": 1.8,
    "acceleration": 5.0,
    "torque_limit": [
      3.0,
      29.0,
      20.0,
      2.2,
      2.0,
      0.6
    ],
    "allow_infeasible": true,
    "description": "A referência rápida é mantida, mas os motores têm limites mais restritos. O torque nominal pode exceder a capacidade disponível.",
    "focus": "Compare com Movimento rápido para isolar os limites. Observe a utilização de torque, o tempo em saturação e o erro. MPC antecipa limites; LQR satura o comando depois de calculá-lo. Isso não garante vantagem em toda rota.",
    "data": "cases/torque/data.js?v=2",
    "folder": "cases/torque/"
  },
  "payload": {
    "key": "payload",
    "dwell": 0.6,
    "label": "04 · Carga desconhecida",
    "speed": 0.65,
    "acceleration": 1.2,
    "payload_kg": 0.5,
    "description": "Massa adicional de 0,5 kg na ponta, presente desde o início e desconhecida pelos controladores.",
    "focus": "Procure erro persistente mesmo nas pausas. Todos continuam usando o braço sem carga em suas previsões; nenhum método ganha um estimador ou ação integral neste teste.",
    "data": "cases/payload/data.js?v=2",
    "folder": "cases/payload/"
  },
  "disturbance": {
    "key": "disturbance",
    "dwell": 0.6,
    "label": "05 · Perturbação externa",
    "speed": 0.65,
    "acceleration": 1.2,
    "pulse": {
      "start_s": 6.0,
      "duration_s": 0.3,
      "torque_nm": [
        6.0,
        -10.0,
        5.0,
        0.0,
        0.0,
        0.0
      ]
    },
    "description": "Entre 6,0 e 6,3 s, torques externos de +6, −10 e +5 N·m atuam nas juntas 1, 2 e 3. A perturbação não é anunciada ao controlador.",
    "focus": "Use a janela do evento para comparar pico de erro e recuperação após 6,3 s. O tempo de recuperação exige erro abaixo de 5 mm por pelo menos 0,5 s.",
    "data": "cases/disturbance/data.js?v=2",
    "folder": "cases/disturbance/"
  }
};

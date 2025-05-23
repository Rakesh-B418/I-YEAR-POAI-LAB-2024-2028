import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


temperature = ctrl.Antecedent(np.arange(0, 41, 1), 'temperature')  # 0 to 40°C
fan_speed = ctrl.Consequent(np.arange(0, 11, 1), 'fan_speed')      # 0 to 10 speed


temperature['low'] = fuzz.trimf(temperature.universe, [0, 0, 20])
temperature['medium'] = fuzz.trimf(temperature.universe, [15, 25, 35])
temperature['high'] = fuzz.trimf(temperature.universe, [30, 40, 40])


fan_speed['slow'] = fuzz.trimf(fan_speed.universe, [0, 0, 5])
fan_speed['medium'] = fuzz.trimf(fan_speed.universe, [3, 5, 7])
fan_speed['fast'] = fuzz.trimf(fan_speed.universe, [6, 10, 10])


rule1 = ctrl.Rule(temperature['low'], fan_speed['slow'])
rule2 = ctrl.Rule(temperature['medium'], fan_speed['medium'])
rule3 = ctrl.Rule(temperature['high'], fan_speed['fast'])


fan_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
fan_simulation = ctrl.ControlSystemSimulation(fan_ctrl)


fan_simulation.input['temperature'] = 28


fan_simulation.compute()
print(f"Temperature: 28°C")
print(f"Fuzzified fan speed: {fan_simulation.output['fan_speed']:.2f}") 10th
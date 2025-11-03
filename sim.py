import os
import sys
import traci



if 'SUMO_HOME' in os.environ:
    # Set up the system path for TraCI
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
    sumo_binary = os.path.join(os.environ['SUMO_HOME'], 'bin', 'sumo-gui.exe')
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")


sumo_cfg = 'simulation.sumocfg'


sumo_binary = "sumo-gui"
sumo_cmd = [sumo_binary, "-c", sumo_cfg]

#
try:
    traci.start(sumo_cmd)
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
  
        # vehicle_speed = traci.vehicle.getSpeed('vehicle_id')
        step += 1
finally:
    traci.close()
    sys.stdout.flush()
# Step 1: Add modules to provide access to specific libraries and functions
import os  # Module provides functions to handle file paths, directories, environment variables
import sys  # Module provides access to Python-specific system parameters and functions

from vehicle import Vehicle
import random

# Step 2: Establish path to SUMO (SUMO_HOME)
if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add Traci module to provide access to specific libraries and functions
import traci  # Static network information (such as reading and analyzing network files)

# Step 4: Define Sumo configuration
Sumo_config = [
    "sumo-gui",
    "-c",
    "highway/highway.sumocfg",
    "--step-length",
    "0.05",
    "--delay",
    "1000",
    "--lateral-resolution",
    "0.1",
]

# Step 5: Open connection between SUMO and Traci
traci.start(Sumo_config)

# Step 6: Define Variables
vehicle_speed = 0
total_speed = 0

# Step 7: Define Functions
vehicles = {}

# Step 8: Take simulation steps until there are no more vehicles in the network
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()  # Move simulation forward 1 step
    # Here you can decide what to do with simulation data at each step
    for newVeh in traci.simulation.getDepartedIDList():
        vehicles[newVeh] = Vehicle(
            newVeh,
            traci.vehicle.getSpeed(newVeh),
            traci.vehicle.getAcceleration(newVeh),
            traci.vehicle.getDecel(newVeh),
            traci.vehicle.getPosition(newVeh),
            traci.vehicle.getLaneIndex(newVeh),
            traci.vehicle.getLanePosition(newVeh),
            traci.vehicle.getLength(newVeh),
        )
        vehicles[newVeh].disableLaneSwitch(traci)

    for currVeh in traci.vehicle.getIDList():
        # see if ego vehcile is in a case to switch lanes
        if vehicles[currVeh].laneChagneTest(traci) !=0:
            # request lane change, changes vehicle state
            vehicles[currVeh].laneSwitchStart(vehicles[currVeh].laneChagneTest(traci))
        
        # update does actual lane change
        vehicles[currVeh].update(traci)

    for oldVeh in traci.simulation.getArrivedIDList():
        del vehicles[oldVeh]


# Step 9: Close connection between SUMO and Traci
traci.close()

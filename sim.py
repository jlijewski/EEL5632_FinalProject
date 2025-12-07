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
LYING_ENABLED = False  
LYING_CHANCE = 0.3   

# track a single vehicle throughout simulation. if None, a random car will be chosen
tracked_vehicle_id = "f_east.7"
highlighted_ids = ["f_east.9","f_east.15"]
# Metrics declaration
departure_times = {}
travel_times = []
colliding_vehicles = set()

# Step 7: Define Functions
vehicles = {}
# Step 8: Take simulation steps until there are no more vehicles in the network
try:
    '''Track a specific vehicle for debugging'''
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()  # Move simulation forward 1 step
        # This tracks the collisions
        colliding_ids = traci.simulation.getCollidingVehiclesIDList()
        for vid in colliding_ids:
            colliding_vehicles.add(vid)

        # Here you can decide what to do with simulation data at each step
        for newVeh in traci.simulation.getDepartedIDList():
            # Starts tracking vehicle time
            departure_times[newVeh] = traci.simulation.getTime()
            lying_factor = 0
            if LYING_ENABLED and random.random() < LYING_CHANCE:
                lying_factor = 1
            isHighlighted = False
            if newVeh in highlighted_ids:
                isHighlighted= True
            vehicles[newVeh] = Vehicle(
                newVeh,
                traci.vehicle.getSpeed(newVeh),
                traci.vehicle.getAcceleration(newVeh),
                traci.vehicle.getDecel(newVeh),
                traci.vehicle.getPosition(newVeh),
                traci.vehicle.getLaneIndex(newVeh),
                traci.vehicle.getLanePosition(newVeh),
                traci.vehicle.getLength(newVeh),
                isHighlighted=isHighlighted,
                lyingFactor=lying_factor
            )
            vehicles[newVeh].disableLaneSwitch(traci)
            # set tracking for this vehicle if its id is to be tracked
            if newVeh==tracked_vehicle_id: 
                vehicles[tracked_vehicle_id].isTracked = True
                print(f"\n+++++++++ NOW TRACKING VEHICLE: {tracked_vehicle_id} ++++++++++\n")

        for currVeh in traci.vehicle.getIDList():
            # update does actual lane change
            vehicles[currVeh].update(traci)

        for oldVeh in traci.simulation.getArrivedIDList():
            # Stop tracking old vehicles
            if vehicles[oldVeh].isTracked:
                tracked_vehicle_id = None
            #Instead of just deleting the vehicle we'll stop the timer there and then delete
            if oldVeh in departure_times:
                travel_time = traci.simulation.getTime() - departure_times[oldVeh]
                travel_times.append(travel_time)
                del departure_times[oldVeh]
            if oldVeh in vehicles:
                del vehicles[oldVeh]

        # choose a new vehicle to track if none or if tracked one left
        if tracked_vehicle_id is None:
            ids = traci.vehicle.getIDList()
            if ids:
                tracked_vehicle_id = random.choice(ids)
                vehicles[tracked_vehicle_id].isTracked = True
                print(f"\n+++++++++ NOW TRACKING VEHICLE: {tracked_vehicle_id} ++++++++++\n")
            else:
                tracked_vehicle_id = None
        

except traci.exceptions.TraCIException as e:
    print("GUI Closed")
finally:
    # Step 9: Close connection between SUMO and Traci
    traci.close()
    # Step 10: Print out the metrics (Theres probably a better way to do this but a print statement works right this second)
    print(f"Vehicles Collided: {len(colliding_vehicles)}")
    if travel_times:
        avg_time = sum(travel_times) / len(travel_times)
        print(f"Average Travel Time: {avg_time:.2f} seconds")
        print(f"Vehicles Arrived: {len(travel_times)}")
    else:
        #hopefully this never happens lol (unless you end simulation super early)
        print("No vehicles arrived.")

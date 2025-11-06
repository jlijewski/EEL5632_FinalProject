class Vehicle:


    def __init__(self, name, speed,accel,pos,lane,lanePos,length):
        self.name = name
        self.speed = speed
        self.accel = accel
        self.pos = pos
        self.lane = lane
        self.lanePos = lanePos
        self.length = length
        self.targetLane = -1
        print(f"Created: {name}")

    def __del__(self):
            print(f"Deleted: {self}")


    def disableLaneSwitch(self,traci):
        traci.vehicle.setLaneChangeMode(self.name,0)

    def update(self,traci):
        self.speed = traci.vehicle.getSpeed(self.name)
        self.accel = traci.vehicle.getAcceleration(self.name)
        self.pos = traci.vehicle.getPosition(self.name)
        self.lane = traci.vehicle.getLaneIndex(self.name)
        self.lanePos = traci.vehicle.getLanePosition(self.name)
        self.length = traci.vehicle.getLength(self.name)

        
class Vehicle:


    def __init__(self, name, speed,accel,pos,lane,lanePos,length):
        self.name = name
        self.speed = speed
        self.accel = accel
        self.pos = pos
        self.lane = lane
        self.lanePos = lanePos
        self.length = length
        print(f"Created: {name}")

    def __del__(self):
            print(f"Deleted: {self}")


    def disableLaneSwitch(self,traci):
        traci.vehicle.setLaneChangeMode(self.name,0)

        
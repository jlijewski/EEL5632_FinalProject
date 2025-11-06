from enum import Enum
from queue import Queue


class VehicleState(Enum):

    Idle = 1
    SendingRequest = 2
    WaitingOnAck = 3


# class Lanes(Enum):
#     Zero = "E1_0"
#     One = "E1_1"
#     Two = "E1_2"
#     Three = "E1_3"
#     Four = "E1_4"


class Vehicle:

    vehicle_requests: dict[str, Queue] = {}

    def __init__(self, name, speed, accel, pos, lane, lanePos, length):
        self.name = name
        self.speed = speed
        self.accel = accel
        self.pos = pos
        self.lane = lane
        self.lanePos = lanePos
        self.length = length
        self.targetLane = -1
        self.state = VehicleState.Idle
        self.vehicle_requests[self.name] = Queue()
        self.ackCount = 0
        # print(f"Created: {name}")

    def __del__(self):
        print(f"Deleted: {self}")

    def disableLaneSwitch(self, traci):
        traci.vehicle.setLaneChangeMode(self.name, 0)

    def update(self, traci):
        self.speed = traci.vehicle.getSpeed(self.name)
        self.accel = traci.vehicle.getAcceleration(self.name)
        self.pos = traci.vehicle.getPosition(self.name)
        self.lane = traci.vehicle.getLaneIndex(self.name)
        self.lanePos = traci.vehicle.getLanePosition(self.name)
        self.length = traci.vehicle.getLength(self.name)

        if self.state == VehicleState.SendingRequest:
            # iterate over all cars in target lane and send nearest ones a request
            for veh in traci.lane.getLastStepVehicleIDs("E1_" + str(self.targetLane)):
                checkLanePos = traci.vehicle.getLanePosition(veh)
                if checkLanePos < (self.lanePos + self.length) and checkLanePos > (
                    self.lanePos - self.length
                ):
                    self.vehicle_requests[veh].put(self.name + "/R")

                    self.ackCount += 1
            if self.ackCount == 0:
                self.laneSwitch(traci)
            else:
                self.state = VehicleState.WaitingOnAck
        elif self.state == VehicleState.WaitingOnAck:
            if self.ackCount == 0:
                self.laneSwitch(traci)
            else:
                self.state = VehicleState.WaitingOnAck

        while not self.vehicle_requests[self.name].empty():
            item = self.vehicle_requests[self.name].get_nowait()
            stringParts = item.split("/")
            if stringParts[1] == "R":
                self.vehicle_requests[stringParts[0]].put(self.name + "/A")
            else:
                self.ackCount -= 1

    def laneSwitchStart(self, target):
        self.state = VehicleState.SendingRequest
        print("started switch")
        self.targetLane = target

    def laneSwitch(self, traci):
        self.state = VehicleState.Idle

        print("CHANGED LANE")
        traci.vehicle.changeLane(
            vehID=self.name, laneIndex=self.targetLane, duration=2.0
        )
        self.targetLane = -1

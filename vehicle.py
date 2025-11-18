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

    def __init__(self, name, speed, accel, pos, lane, lanePos, length, cStyle):
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
        # if cStyle>1 then aggressive, if cStyle<1 then conservative
        # aggressive driver change with shorter distance
        self.cStyle = 1; # 1 for neutral driver. this is an attrivute of vehicle
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
                self.laneSwitchSimple(traci)
            else:
                self.state = VehicleState.WaitingOnAck
        elif self.state == VehicleState.WaitingOnAck:
            if self.ackCount == 0:
                self.laneSwitchSimple(traci)
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

    def laneSwitchSimple(self, traci):
        self.state = VehicleState.Idle

        print("CHANGED LANE")
        traci.vehicle.changeLane(
            vehID=self.name, laneIndex=self.targetLane, duration=2.0
        )
        self.targetLane = -1
    
    def laneSwitch2(self, traci):
        """
        Level 2 Lane Switch
        Based on: 
        R. Dang, J. Ding, B. Su, Q. Yao, Y. Tian and K. Li, 
        "A lane change warning system based on V2V communication" 
        """

        """
        Enviroment Detection
        Get data using V2V about: 
        relative position and 
        relative movement of surrounding vechicles
        """

        """
        Calculate minimum safe distance for lane chance
        Based on enviroment data and driving style index

        min safe distance analysis avoids collision and keep safe following distance
        rear vehicle - small deceleration or constant speed
        rear vehichle is most important

        Lane change is feasible if distance between ego vehicle and all 4 surrounding 
        vechicles is bigger than calculated minimum safe distance 
        """
        

        """
        Judgment of Lane change Feasibility
        Compare min safe distance to real distance
        """
        # isFeasible = False
        # if (self.loc - rear.loc)> safeDistRearTarget:
        #   isFeasible = True;
        """
        Take Action
        Either alert driver that lane change is good or bad
        or just do lane change based on if good or bad
        """

    def safeDistRearTarget(v_rd, v_k, c_style):
        """
        Find the minimum safe distance between ego-vehicle and rear vehicle in target lane
        rear target lane is most impactful vehicle
        From eq (16)
        v_rd rear vechicle in target lane speed before lane change
        v_k ego vechicles speed befor lane change
        somewhere between equation (5) and (6) v_h changes to v_k
        cStyle ego driving style
        """
        # avg paramters for drivers and vehicles
        # some of these could come from v2v
        t_driver = 1.35; # driver's reaction time
        t_brake = 0.15; # acting time of braking system
        t_rdsafe = 1.8; # rear vehicle time headway
        a_rdcon = 2; # max deceleration of rear vehicle in target lane
        t_reaction = t_driver + t_brake;
        
        # How can we include cStyle of non ego vehicle through v2v, how does that change equation
        
        # Find min safe distance between ego and rear
        if v_rd -v_k >=-5:
            t1 = c_style*(v_rd-v_k)*t_reaction +3*c_style*(v_rd - v_h)^2/(2*a_rdcon) + c_style*t_rd_safe*v_rd
            t2 = c_style*t_rdsafe*v_rd
            delta_d_mrdh = max(t1, t2)
        else:
            delta_d_mrdh = c_style*t_rdsafe*v_rd
        return delta_d_mrdh


    # def safeDistFrontTarget(v_rd, v_h, c_style):
    #     """
    #     Find the minimum safe distance between ego-vehicle and rear vehicle in target lane
    #     rear target lane is most impactful vehicle
    #     From eq (16)
    #     v_rd rear vechicle in target lane speed before lane change
    #     v_h ego vechicles speed befor lane change
    #     somewhere between equation (5) and (6) v_h changes to v_k
    #     cStyle ego driving style
    #     """
    #     # avg paramters for drivers and vehicles
    #     # some of these could come from v2v
    #     t_driver = 1.35; # driver's reaction time
    #     t_brake = 0.15; # acting time of braking system
    #     t_rdsafe = 1.8; # rear vehicle time headway
    #     a_rdcon = 2; # max deceleration of rear vehicle in target lane
    #     t_reaction = t_driver + t_brake;
    #     t1 = c_style*v_h*t_reaction+c_style*v_h^2/2*(a_hmax) -c_style*v_ld^2/(2*a_ldmax)
    #     t2 = c_style*v_h*t_ldsafe

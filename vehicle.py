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
class Packet:
    def __init__(self,sender,type,reportedSpeed, distance):
        self.sender = sender
        self.type = type
        self.speed = reportedSpeed
        self.distance = distance

class Vehicle:

    vehicle_requests: dict[str, Queue] = {}

    def __init__(self, name, speed, accel, pos, lane, lanePos, length, startTime):
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
        self.startTime = startTime
        # TODO : consider adding cstyle as attribute of veh
        # print(f"Created: {name}")

    def __del__(self):
        pass# print(f"Deleted: {self}")

    def disableLaneSwitch(self, traci):
        traci.vehicle.setLaneChangeMode(self.name, 0)

    def update(self, traci):
        self.speed = traci.vehicle.getSpeed(self.name)
        self.accel = traci.vehicle.getAcceleration(self.name)
        self.pos = traci.vehicle.getPosition(self.name)
        self.lane = traci.vehicle.getLaneIndex(self.name)
        self.lanePos = traci.vehicle.getLanePosition(self.name)
        self.length = traci.vehicle.getLength(self.name)

        """ 
        avg paramters for drivers and vehicles
        TODO:
            some of these could be taken from v2v instead
            these are used to set the gap and to calculate safe distance
            its silly to define them here and in the safe distance methods
        """
        t_driver = 1.35 # driver's reaction time
        t_brake = 0.15 # acting time of braking system
        t_rdsafe = 1.8 # rear vehicle time headway
        a_rdcon = 2 # max deceleration of rear vehicle in target lane
        t_reaction = t_driver + t_brake

        if self.state == VehicleState.SendingRequest:
            # iterate over all cars in target lane and send nearest rear and nearest leader a request
            traci.vehicle.highlight(self.name, color=(104, 171, 31)) #green
            # get target side Followers 
            # vehXX gives a tuple (veh_ID, dist) where distance is the distance between them and ego car
            vehRTlist = traci.vehicle.getLeftFollowers(self.name)
            vehRT=("",-1)
            if vehRTlist:
                vehRT = min(vehRTlist, key=lambda x: x[1])
            # get target side leader
            vehLTlist = traci.vehicle.getLeftLeaders(self.name)
            vehLT=("",-1)
            if vehLTlist:
                vehLT = min(vehLTlist, key=lambda x: x[1])
            #get follower
            vehRO = traci.vehicle.getFollower(self.name)
            # get leader
            vehLO = traci.vehicle.getLeader(self.name)
            
            # Highlight neighbors for fun
            if vehRT[0] != "": 
                #Note that these need to be double quotes or it will fail
                # theres some weird string stuff so watch for errors in these checks
                traci.vehicle.highlight(vehRT[0],color = (219, 99, 61))#orange
            if vehLT[0] !="":
                traci.vehicle.highlight(vehLT[0],color = (147, 83, 222))#purple
            if vehRO!=None and vehRO[0]!='':
                traci.vehicle.highlight(vehRO[0],color = (235, 198, 27))#yellow
            if vehLO!=None and vehLO[0]!='':
                traci.vehicle.highlight(vehLO[0],color = (100, 137, 242)) #blue
            if vehRT[0]!='' and vehRT[0] !="":

                """
                request a lane change from the closest vehicle. if RT vehicle agrees, set its max headway and decel
                TODO: need additional acks for other neighbor cars using the appropriate safe dist method. 
                each safe dist smaller than the distance between the ego car and the neighbor
                """
                # TODO: need additional acks 

                RTsafeDist = self.findRTsafeDist(traci.vehicle.getSpeed(vehRT[0]), self.speed, 1)
                requestPacket = Packet(type = "R", sender = self.name,reportedSpeed=self.speed,distance=vehRT[1])
                self.vehicle_requests[vehRT[0]].put(requestPacket)
                print("sent request from ", self.name, " to ",vehRT[0] )
                #if vehRT[1]> RTsafeDist:
                self.ackCount += 1
                if self.ackCount == 0:
                    # keep the RT vehicle at the expected headway and prevent random acceleration
                    if vehRT[0] !=  self.name:
                        traci.vehicle.openGap(vehID = vehRT[0],newTimeHeadway =  t_rdsafe, newSpaceHeadway= RTsafeDist, duration = 3.0, changeRate = a_rdcon, referenceVehID=self.name)
                    self.laneSwitch(traci)
                else:
                    self.state = VehicleState.WaitingOnAck
        elif self.state == VehicleState.WaitingOnAck:
            if self.ackCount == 0:
                self.laneSwitch(traci)
            else:
                # print(self.lane , ": " ,self.ackCount)
                self.state = VehicleState.WaitingOnAck
        #else:
            #traci.vehicle.highlight(self.name, color=(255, 255, 255)) 

        while not self.vehicle_requests[self.name].empty():
            item = self.vehicle_requests[self.name].get_nowait()
            #stringParts = item.split("/")
            print("received request from ", item.sender )
            print(item.type)
            if item.type == "R":
                safeDistCheck = self.findRTsafeDist(item.speed, self.speed, 1)
                print("comparing ",item.distance, "and ", safeDistCheck )
                if(item.distance >safeDistCheck ):
                    print("sending ack to ", item.sender)
                    ackPacket = Packet(type="A",sender=self.name,reportedSpeed=self.speed, distance=None)
                    self.vehicle_requests[item.sender].put(ackPacket)
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
    

    def findRTsafeDist(self,v_rd, v_k, c_style):
        """
        Find the minimum safe distance between ego-vehicle and rear vehicle in target lane
        Based on: 
        R. Dang, J. Ding, B. Su, Q. Yao, Y. Tian and K. Li, 
        "A lane change warning system based on V2V communication" 
        From eq (16)
        rear target lane is most impactful vehicle
        v_rd rear vechicle in target lane speed before lane change
        v_k ego vechicles speed befor lane change
        cStyle ego driving style
        """
        # TODO: see comment in update function on these, this double definition is silly
        t_driver = 1.35 # driver's reaction time
        t_brake = 0.15 # acting time of braking system
        t_rdsafe = 1.8 # rear vehicle time headway
        a_rdcon = 2 # max deceleration of rear vehicle in target lane
        t_reaction = t_driver + t_brake
        
        # TODO? can we include cStyle of non ego vehicle through v2v, how does that change equation
        
        # Find min safe distance between ego and rear
        if v_rd -v_k >=-5:
            t1 = c_style*(v_rd-v_k)*t_reaction +3*c_style*(v_rd - v_k)**2/(2*a_rdcon) + c_style*t_rdsafe*v_rd
            t2 = c_style*t_rdsafe*v_rd
            delta_d_mrdh = max(t1, t2)
        else:
            delta_d_mrdh = c_style*t_rdsafe*v_rd
        return delta_d_mrdh


    def findLTSafeDist(v_ld, v_h, c_style):
        """
        Find the minimum safe distance between ego-vehicle and leader vehicle in target lane
        From eq (16)
        v_ld rear vechicle in target lane speed before lane change
        v_h ego vechicles speed befor lane change
        cStyle ego driving style
        """
        # avg paramters for drivers and vehicles
        # some of these could come from v2v
        # TODO these may be able to be pulled from car info
        t_driver = 1.35 # driver's reaction time
        t_brake = 0.15 # acting time of braking system
        t_reaction = t_driver + t_brake

        # TODO: read through paper more to figure out what these calues should be
        t_ldsafe = 1.8 # rear vehicle time headway
        a_ldcon = 2 # max deceleration of rear vehicle in target lane
        # TODO!! IDK how to find this value, this is a place holder, need to examine paper
        a_hmax = 2.5 
        a_ldmax = 2.5

        t1 = c_style*v_h*t_reaction+c_style*v_h**2/2*(a_hmax) -c_style*v_ld**2/(2*a_ldmax)
        t2 = c_style*v_h*t_ldsafe
        delta_d_mldh = min(t1,t2)
        return delta_d_mldh
    
    def findXOSafeDist(v_h, t_xosafe, c_style):
        """
        Find the minimum safe distance between ego-vehicle and vehicle in original lane
        From eq (16)
        v_h ego vechicles speed befor lane change
        t_xosafe safe time headway between ego and front/rear vehicle in original lane
            this may or may not actually be a passed arg
        cStyle ego driving style
        """
        delta_d_xoh = c_style*v_h*t_xosafe
        return delta_d_xoh



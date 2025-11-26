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
        # TODO : consider adding cstyle as attribute of veh
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
            #get follower
            vehRO = traci.vehicle.getFollower(self.name)
            # get leader
            vehLO = traci.vehicle.getLeader(self.name)
            # Highlight neighbors for fun

            # determine if switching to left or right and get appropriate leader and follower (there is an easier way to do thise with bits)
            # original-lane neighbors
            vehRO = traci.vehicle.getFollower(self.name)
            vehLO = traci.vehicle.getLeader(self.name)

            # pick target-side followers/leaders based on desired target lane
            if self.targetLane > 0:
                # target is left
                vehRTList = traci.vehicle.getLeftFollowers(self.name) or []
                vehLTList = traci.vehicle.getLeftLeaders(self.name) or []
            else:
                # target is right (or default)
                vehRTList = traci.vehicle.getRightFollowers(self.name) or []
                vehLTList = traci.vehicle.getRightLeaders(self.name) or []

            vehRT = min(vehRTList, key=lambda x: x[1]) if vehRTList else ("", -1)
            vehLT = min(vehLTList, key=lambda x: x[1]) if vehLTList else ("", -1)

            """
            request a lane change from the closest vehicle. if RT vehicle agrees, set its max headway and decel
            TODO: need additional acks for other neighbor cars using the appropriate safe dist method. 
            each safe dist smaller than the distance between the ego car and the neighbor
            """
            # if there's a target-side follower, request it to make space
            if vehRT[0]:
                # send request to nearest follower on target side
                # NOTE: ensure vehicle queue exists for that vehicle (created on depart)
                self.vehicle_requests[vehRT[0]].put(self.name + "/R")
                RTsafeDist = self.findRTsafeDist(traci.vehicle.getSpeed(vehRT[0]), self.speed, 1)
                if vehRT[1] > RTsafeDist:
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
                self.state = VehicleState.WaitingOnAck

        while not self.vehicle_requests[self.name].empty():
            item = self.vehicle_requests[self.name].get_nowait()
            stringParts = item.split("/")
            if stringParts[1] == "R":
                self.vehicle_requests[stringParts[0]].put(self.name + "/A")
            else:
                self.ackCount -= 1
    
    def laneChagneTest (self, traci):
        """
        Return:
        1  -> attempt to change left
        -1  -> attempt to change right
        0  -> no change
        """
        maxLane = 4
        changeLane = 0

        # leader in current lane (veh tuple: (id, dist) or None)
        vehLO = traci.vehicle.getLeader(self.name)
        if vehLO is not None and vehLO[0] != "":
            # if leader ahead is slower and not accelerating more than ego, consider passing
            leader_id = vehLO[0]
            leader_speed = traci.vehicle.getSpeed(leader_id)
            leader_acc = traci.vehicle.getAcceleration(leader_id)
            if leader_speed < self.speed and (leader_acc - self.accel) < 0:
                # pass in left lane, making target lane left lane
                if self.lane < maxLane:
                    self.checkLeftLanePass(traci)
                else:
                    # If there is no left lane, try passing in right lane
                    if self.lane >= maxLane:
                        changeLane = self.checkRightLanePass(traci)
        '''
        # NEW: if in leftmost lane (0) and a vehicle on the right ahead is faster,
        # consider moving right (allow faster traffic to pass). Check right follower gap.
        if self.lane == maxLane:
            right_leaders = traci.vehicle.getRightLeaders(self.name) or []
            if right_leaders:
                right_leader = min(right_leaders, key=lambda x: x[1])
                if right_leader[0] != "" and traci.vehicle.getSpeed(right_leader[0]) > self.speed:
                    right_followers = traci.vehicle.getRightFollowers(self.name) or []
                    right_follower = min(right_followers, key=lambda x: x[1]) if right_followers else ("", float("inf"))
                    # if no right follower or follower gap is safe, request move right
                    if right_follower[0] == "":
                        changeLane = -1
                    else:
                        rd_safe = self.findRTsafeDist(traci.vehicle.getSpeed(right_follower[0]), self.speed, 1)
                        if right_follower[1] > rd_safe:
                            changeLane = -1
        '''
        if self.lane <= maxLane:
            vehRO = traci.vehicle.getFollower(self.name)

            if vehRO[0] != "":
                try:
                    rf_goal = traci.vehicle.getMaxSpeed(vehRO[0])*traci.vehicle.getSpeedFactor(vehRO[0])  # follower's goal / max speed
                except Exception:
                    rf_goal = traci.vehicle.getSpeed(vehRO[0])  # fallback

                try:
                    ego_goal = traci.vehicle.getMaxSpeed(self.name)*traci.vehicle.getSpeedFactor(self.name)
                except Exception:
                    ego_goal = self.speed

                # threshold to avoid tiny differences triggering lane changes
                SPEED_THRESHOLD = 0.5  # m/s

                # check follower wants to go noticeably faster and follower gap in target lane is safe
                if rf_goal > ego_goal + SPEED_THRESHOLD:
                    # compute safe distance required for follower in target lane
                    rd_safe = self.findRTsafeDist(traci.vehicle.getSpeed(vehRO[0]), self.speed, 1) if vehRO[0] else float("inf")
                    if vehRO[0] == "" or vehRO[1] > rd_safe:
                        changeLane = -1

        
        return changeLane
    
    def laneSwitchStart(self, target):
        self.state = VehicleState.SendingRequest
        print("started switch")
        self.targetLane = target+self.lane

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

    def checkRightLanePass(self, traci):
        changeLane =0
        right_leaders = traci.vehicle.getRightLeaders(self.name) or []
        right_followers = traci.vehicle.getRightFollowers(self.name) or []
        right_leader = min(right_leaders, key=lambda x: x[1]) if right_leaders else ("", float("inf"))
        right_follower = min(right_followers, key=lambda x: x[1]) if right_followers else ("", float("inf"))
        if right_leader[0] == "" and right_follower[0] == "":
            changeLane = -1
        elif right_follower[0] != "":
            rd_safe = self.findRTsafeDist(traci.vehicle.getSpeed(right_follower[0]), self.speed, 1)
            if right_follower[1] > rd_safe:
                changeLane = -1
        return changeLane

    def checkLeftLanePass(self,traci):
        changeLane =0
        # get nearest left leader and follower
        left_leaders = traci.vehicle.getLeftLeaders(self.name) or []
        left_followers = traci.vehicle.getLeftFollowers(self.name) or []
        left_leader = min(left_leaders, key=lambda x: x[1]) if left_leaders else ("", float("inf"))
        left_follower = min(left_followers, key=lambda x: x[1]) if left_followers else ("", float("inf"))

        # if no vehicles in left lane, then just pass
        if left_leader[0] == "" and left_follower[0] == "":
            changeLane = 1
        else:
            # check rear follower safe distance if present
            #TODO: add more checks
            if left_follower[0] != "":
                rd_safe = self.findRTsafeDist(traci.vehicle.getSpeed(left_follower[0]), self.speed, 1)
                if left_follower[1] > rd_safe:
                    changeLane = 1
        return changeLane
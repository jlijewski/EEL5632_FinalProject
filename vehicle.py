from enum import Enum
from queue import Queue


class VehicleState(Enum):

    Idle = 1
    SendingRequest = 2
    WaitingOnAck = 3


class Packet:
    def __init__(self, sender, type, reportedSpeed, distance, accel=None, pos=None, lane=None):
        self.sender = sender
        # Type is R for request, A for Ack
        self.type = type
        self.speed = reportedSpeed
        self.distance = distance
        self.accel = accel
        self.pos = pos
        self.lane = lane


# class Lanes(Enum):
#     Zero = "E1_0"
#     One = "E1_1"
#     Two = "E1_2"
#     Three = "E1_3"
#     Four = "E1_4"


class Vehicle:

    vehicle_requests: dict[str, Queue] = {}

    #I need this to keep track of who is chaning lanes
    active_lane_changes: dict[str, tuple[int, int]] = {}
    
    
    vehicle_lying_factors: dict[str, int] = {}

    def __init__(self, name, speed, accel, pos, lane, lanePos, length, lyingFactor=0):
        self.name = name
        self.speed = speed
        self.accel = accel
        self.pos = pos
        self.lane = lane
        self.lanePos = lanePos
        self.length = length
        self.targetLane = 0
        self.state = VehicleState.Idle
        self.vehicle_requests[self.name] = Queue()
        self.ackCount = 0
        self.requestsSent = 0
        self.lyingFactor = lyingFactor # 0 for honest, 1 for lying
        self.vehicle_lying_factors[self.name] = self.lyingFactor
        # TODO : consider adding cstyle as attribute of veh
        # print(f"Created: {name}")

    def __del__(self):

        if self.name in self.active_lane_changes:
            del self.active_lane_changes[self.name]
        if self.name in self.vehicle_lying_factors:
            del self.vehicle_lying_factors[self.name]
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

        # change lane if necessary and send request. if ego car is sending a request, then it wants to change lane
        if self.state == VehicleState.SendingRequest:

            if self.farLaneDetection(traci):
                traci.vehicle.highlight(self.name, color=(255, 0, 0))
                
                self.state = VehicleState.Idle
                if self.name in self.active_lane_changes:
                    del self.active_lane_changes[self.name]
                self.targetLane = 0
                print(f"{self.name} aborting: far lane conflict.")
                return

            # skip if no target lane was set
            if self.targetLane == 0:
                self.state = VehicleState.Idle
                return
            # Goal: send request to 4 neighbor cars, follower/leader in target lane and follower/leader in original lane
            # iterate over all cars in target lane and send nearest rear and nearest leader a request
            traci.vehicle.highlight(self.name, color=(104, 171, 31)) #green

            '''Get neighbor cars'''
            # vehXX gives a tuple (veh_ID, dist) where distance is the distance between them and ego car
            #get ego lane follower
            vehRO = traci.vehicle.getFollower(self.name) or ["",-1]
            # get ego lane leader
            vehLO = traci.vehicle.getLeader(self.name) or ["",-1]
            # determine if switching to left or right and get appropriate leader and follower (there is an easier way to do thise with bits)
            if self.targetLane > 0:
                # target is left
                vehRTList = traci.vehicle.getLeftFollowers(self.name) or []
                vehLTList = traci.vehicle.getLeftLeaders(self.name) or []
            elif self.targetLane < 0:
                # target is right lane
                vehRTList = traci.vehicle.getRightFollowers(self.name) or []
                vehLTList = traci.vehicle.getRightLeaders(self.name) or []
            # get closest follower/leader in target lane
            vehRT = ("",-1) if not vehRTList else min(vehRTList, key=lambda x: x[1])
            vehLT = ("",-1) if not vehLTList else min(vehLTList, key=lambda x: x[1])

            """
            Send Request a lane change from the closest vehicles. if RT vehicle agrees, set its max headway and decel
            TODO: need additional acks for other neighbor cars using the appropriate safe dist method. 
            each safe dist smaller than the distance between the ego car and the neighbor
            """
            self.requestsSent = 0
            
            if self.lyingFactor == 1:
                traci.vehicle.highlight(self.name, color=(255, 0, 0))

            def send_pkt(target_id, dist):
                if target_id:
                    reported_speed = self.speed - (self.lyingFactor * 20)
                    reported_dist = dist + (self.lyingFactor * 30)
                    reported_accel = self.accel - (self.lyingFactor * 10)
                    
                    pkt = Packet(self.name, "R", reported_speed, reported_dist, reported_accel, self.pos, self.lane)
                    self.vehicle_requests[target_id].put(pkt)
                    self.requestsSent += 1

            send_pkt(vehRT[0], vehRT[1])
            send_pkt(vehLT[0], vehLT[1])
            send_pkt(vehRO[0], vehRO[1])
            send_pkt(vehLO[0], vehLO[1])

            '''if all neighbors agree on request, do lane change'''
            # if there are no neighbor cars, do lane change
            if self.requestsSent == 0:
                self.laneSwitch(traci)
            # check that the number of requests sent is same as requests returned
            elif self.ackCount == self.requestsSent:
                # keep the RT vehicle at the expected headway and prevent random acceleration

                # This block had a divide by zero issue, so I added a check to make sure the speed is greater than 0
                if vehRT[0]:
                    RTsafeDist = self.findRTsafeDist(traci.vehicle.getSpeed(vehRT[0]), self.speed, 1)
                    
                    if traci.vehicle.getSpeed(vehRT[0]) >0:
                        thw = vehRT[1]/traci.vehicle.getSpeed(vehRT[0])
                        # check if the current headway is smaller than safe headway
                        # TODO: should I do this for all lanes? Probably just maintain gap for all, not change it 
                        if thw<t_rdsafe:
                            traci.vehicle.openGap(vehID = vehRT[0],newTimeHeadway =  t_rdsafe, newSpaceHeadway= RTsafeDist, duration = 5.0, changeRate = a_rdcon, referenceVehID=self.name)
                        else:
                            #maintain current headway
                            traci.vehicle.openGap(vehID = vehRT[0], newTimeHeadway=thw, newSpaceHeadway=vehRT[1],duration=5.0, changeRate=0.5)
                    else:
                        traci.vehicle.openGap(vehID = vehRT[0], newTimeHeadway=1000.0, newSpaceHeadway=vehRT[1],duration=5.0, changeRate=0.1)
                self.laneSwitch(traci)
            else:
                # if there are unreturned requests, continue to wait
                self.state = VehicleState.WaitingOnAck

        elif self.state == VehicleState.WaitingOnAck:
            # if all acks are in, do lane switch, otherwise keep waiting
            if self.ackCount == self.requestsSent:
                #TODO: extra safety check?
                self.laneSwitch(traci)
            else:
                self.state = VehicleState.WaitingOnAck

        # process incomming request and ack
        while not self.vehicle_requests[self.name].empty():
            packet = self.vehicle_requests[self.name].get_nowait()
            
            if packet.type == "R":
                # send back the needed info to check if the lane change is safe
                safeDistCheck = self.findRTsafeDist(packet.speed, self.speed, 1)
                if packet.distance > safeDistCheck and packet.accel <= self.accel:
                    ackPacket = Packet(
                        sender=self.name,
                        type="A",
                        reportedSpeed=self.speed,
                        distance=packet.distance,
                        accel=self.accel,
                        pos=self.pos,
                        lane=self.lane
                    )
                    self.vehicle_requests[packet.sender].put(ackPacket)
            
            elif packet.type == "A":
                self.ackCount += 1
            
            elif packet.type == "D":
                print(f"Lane change denied by {packet.sender}")
                self.state = VehicleState.Idle
                self.ackCount = 0
                if self.name in self.active_lane_changes:
                    del self.active_lane_changes[self.name]
    
    def lanechangeTest (self, traci):
        """
        Return:
        1  -> attempt to change left
        -1  -> attempt to change right
        0  -> no change
        """
        maxLane = 4
        changeLane = 0
        
        # pass slower veh: if leader ahead is slower and not accelerating more than ego, consider passing in left lane
        # vehicles in the left most lane cant pass to the left
        if self.lane < maxLane:
            # check if there is a leader vehicle
            vehLO = traci.vehicle.getLeader(self.name)
            if vehLO is not None and vehLO[0] != "":
                leaderID = vehLO[0]
                leaderSpeed = traci.vehicle.getSpeed(leaderID)
                leaderAcc = traci.vehicle.getAcceleration(leaderID)
                # pass to left if vehicle infront is slower then self 
                if leaderSpeed < self.speed and (leaderAcc - self.accel) < 0:
                    # pass if left lane is empty 
                    # or pass if gap is big enough
                    changeLane = self.checkLeftLanePass(traci)

        # Change to right lane, cant already be in right msot lane
        if self.lane > 0:
            # move for faster veh: if goal speed of following veh in same lane is faster, get over to right
            vehRO = traci.vehicle.getFollower(self.name)
            # confirmf there is  rear vehicle in the same lane
            if vehRO[0] != "":
                # get goal speed of following vehicle
                rf_goal = traci.vehicle.getMaxSpeed(vehRO[0])*traci.vehicle.getSpeedFactor(vehRO[0])  # follower's goal / max speed
                # get goal speed of ego vehicle
                ego_goal = traci.vehicle.getMaxSpeed(self.name)*traci.vehicle.getSpeedFactor(self.name)
                # threshold to avoid tiny differences triggering lane changes
                SPEED_THRESHOLD = 0.5  # m/s
                # compare goal speeds of ego and following vehicle (maxSpeed*speedFactor)
                if rf_goal > ego_goal + SPEED_THRESHOLD:
                    changeLane = self.checkRightLanePass(traci)
        return changeLane
    
    def laneSwitchStart(self, target):
        self.state = VehicleState.SendingRequest
        print("started switch")
        self.targetLane = target+self.lane

        self.active_lane_changes[self.name] = (self.targetLane, self.lyingFactor)

    def laneSwitch(self, traci):
        self.state = VehicleState.Idle

        print("CHANGED LANE")
        traci.vehicle.changeLane(
            vehID=self.name, laneIndex=self.targetLane, duration=2.0
        )

        if self.name in self.active_lane_changes:
            del self.active_lane_changes[self.name]

        self.targetLane = 0
    

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


    def findLTsafeDist(self, v_ld, v_h, c_style):
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
    
    def findXOsafeDist(self, v_h, t_xosafe, c_style):
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
    

    def farLaneDetection(self, traci):
        """
        This function checks the lane over from the target lane for vehicles that want to
        change lane, and if so we will yield to them.
        """
        
        direction = 1 if self.targetLane > self.lane else -1
        lane_index = self.targetLane + direction

        lane_id = traci.vehicle.getRoadID(self.name)
        try:
            num_lanes = traci.edge.getLaneNumber(lane_id)
        
        except traci.exceptions.TraCIException:
            return False
        
        if lane_index < 0 or lane_index >= num_lanes:
            return False
        
        far_lane_id = f"{lane_id}_{lane_index}"

        try:
            vehicle_index_far_lane = traci.lane.getLastStepVehicleIDs(far_lane_id)
        except traci.exceptions.TraCIException:
            return False
        
        for veh_id in vehicle_index_far_lane:
            try:
                if veh_id in self.active_lane_changes:
                    target_lane, lying_factor = self.active_lane_changes[veh_id]
                    
                    if target_lane == self.targetLane:
                        other_pos = traci.vehicle.getLanePosition(veh_id)
                        other_speed = traci.vehicle.getSpeed(veh_id)
                        
                  
                        other_speed -= (lying_factor * 20)
                        
                        dist = other_pos - self.lanePos
                        
                        if dist > 0:
                            dist += (lying_factor * 30)
                        else:
                            dist -= (lying_factor * 30)

                        is_conflict = False
                        
                        if dist < 0: 
                            safe_dist = self.findRTsafeDist(other_speed, self.speed, 1)
                            if abs(dist) < safe_dist:
                                is_conflict = True
                        else:
                            
                            
                            safe_dist = self.findLTsafeDist(other_speed, self.speed, 1) 
                            if abs(dist) < safe_dist:
                                is_conflict = True
                                
                        if is_conflict:
                            
                            #we could change this to be smarter but it just chooses the vehicle with the higher ID to yield
                            if self.name > veh_id:
                                return True

            except traci.exceptions.TraCIException:
                continue
        
        return False

    def checkRightLanePass(self, traci):
        changeLane =0
        right_leaders = traci.vehicle.getRightLeaders(self.name) or []
        right_followers = traci.vehicle.getRightFollowers(self.name) or []
        right_leader = min(right_leaders, key=lambda x: x[1]) if right_leaders else ("", float("inf"))
        right_follower = min(right_followers, key=lambda x: x[1]) if right_followers else ("", float("inf"))
        rd_safe = self.findRTsafeDist(traci.vehicle.getSpeed(right_follower[0]), self.speed, 1) if right_follower[0] else float("inf")
        if (right_follower[0] == "" and right_follower[0]=="") or right_follower[1] > rd_safe:
            changeLane = -1
        return changeLane

    def checkLeftLanePass(self,traci):
        changeLane =0
        # get nearest left leader and follower
        left_leaders = traci.vehicle.getLeftLeaders(self.name) or []
        left_followers = traci.vehicle.getLeftFollowers(self.name) or []
        left_leader = min(left_leaders, key=lambda x: x[1]) if left_leaders else ("", float("inf"))
        left_follower = min(left_followers, key=lambda x: x[1]) if left_followers else ("", float("inf"))
        rd_safe = self.findRTsafeDist(traci.vehicle.getSpeed(left_follower[0]), self.speed, 1) if left_follower[0] else float("inf")
        # change lane if there is no car in target lane or if gap is safe to pass in target lane
        if (left_follower[0] == "" and left_follower[0]=="") or left_follower[1] > rd_safe:
            changeLane = 1
        return changeLane
    
    def getTimeHeadway(self,traci,followerID, lookahead=100.0):
        leader_info = traci.vehicle.getLeader(followerID, lookahead)
        if leader_info is None:
            return None  # no leader ahead
        leaderID, gap = leader_info
        speed = traci.vehicle.getSpeed(followerID)
        return gap / speed if speed > 0 else float('inf')
from enum import Enum
from queue import Queue


class VehicleState(Enum):

    Idle = 1
    SendingRequest = 2
    WaitingOnAck = 3
    ChangingLane = 4


# class Lanes(Enum):
#     Zero = "E1_0"
#     One = "E1_1"
#     Two = "E1_2"
#     Three = "E1_3"
#     Four = "E1_4"
'''
Request Packet  
    - sent by car wanting to change lanes 
    - received by neighbor car 
    - payload is data of requesting car 
    sender  name of packet sender
    type    type of Packet, R for request
    neighbor    position of reciever packet compared to sender: 
                0 = RT, 1 = LT, 2 = RO, 3 = LO, 4 = RF, 5 = LF
'''
class RequestPacket:
    def __init__(self, sender, neighbor, state, reportedSpeed, reportedGap, accel=None, decel = None, pos=None, lane=None):
        self.sender = sender
        # Type is R for request, A for Ack
        self.type = 'R'
        self.neighbor = neighbor
        self.state = state
        self.speed = reportedSpeed
        self.reportedGap = reportedGap
        self.accel = accel
        self.decel = decel
        self.pos = pos
        self.lane = lane
    def __str__(self):
        return f"++++ Request Packet Sent to {self.neighbor}, reported at speed = {self.speed:.2f}, Gap = self.reportedGap ++++\n"

'''
Return Packet   -   sent by car that is neighbor to car changing lanes
                -   recieved by car wanting to change lane
                -   sent as response to Request Packet
                -   Payload gives minimum safe gap distance
'''
class ReturnPacket:
    def __init__(self, sender, neighbor, state, reportedGap, safeDistance, safeDistanceCheck):
        self.sender = sender
        self.type = "A"
        self.neighbor = neighbor
        self.state = state
        self.reportedGap = reportedGap
        self.safeDistance = safeDistance
        self.safeDistanceCheck = safeDistanceCheck
    def __str__(self):
        return f"==== Ack Packet sent from {self.neighbor}, reported Gap = {self.reportedGap:.2f} compare with safe Dis {self.safeDistance:.2f}  and the check is {self.safeDistanceCheck}\n"

class Vehicle:

    vehicle_requests: dict[str, Queue] = {}
    vehicle_lying_factors: dict[str, int] = {}

    def __init__(self, name, speed, accel, decel, pos, lane, lanePos, length, lyingFactor=0):
        self.name = name
        self.speed = speed
        self.accel = accel
        self.decel = decel
        self.pos = pos
        self.lane = lane
        self.lanePos = lanePos
        self.length = length
        self.targetLane = 0
        self.state = VehicleState.Idle
        self.vehicle_requests[self.name] = Queue()
        self.delta_d_rt = 0 
        self.ackCount = 0
        self.requestsSent = 0
        self.lyingFactor = lyingFactor # 0 for honest, 1 for lying
        self.vehicle_lying_factors[self.name] = self.lyingFactor
        self.isTracked = False
        # TODO : consider adding cstyle as attribute of veh
        # print(f"Created: {name}")

    def __del__(self):
        if self.name in self.vehicle_lying_factors:
            del self.vehicle_lying_factors[self.name]
        if self.isTracked: print(f"Deleted: {self}")

    def disableLaneSwitch(self, traci):
        traci.vehicle.setLaneChangeMode(self.name, 0)

    def tracker(self, traci):
        traci.vehicle.highlight(self.name, color=(215, 35, 168))
        print(f"======\nname = {self.name}, speed = {self.speed:.2f}, accel = {self.accel:.2f}, decel = {self.decel:.2f}, lane = {self.lane}\nState = {self.state}, targetLane = {self.targetLane}")
        print(f'---- Vehicle has {self.requestsSent} requests and {self.ackCount} acks in queue\n')
        if self.targetLane!=0:
            neighborList = self.getNeighbors(traci)
            for n in neighborList:
                if n[0]:
                    traci.vehicle.highlight(n[0], color=(39, 180, 190)) #magenta
                    print(f"--- Neighbor: {n[0]} at dist {n[1]} ---\n")

    def update(self, traci):
        self.speed = traci.vehicle.getSpeed(self.name)
        self.accel = traci.vehicle.getAcceleration(self.name)
        self.decel = traci.vehicle.getDecel(self.name)
        self.pos = traci.vehicle.getPosition(self.name)
        self.lane = traci.vehicle.getLaneIndex(self.name)
        self.lanePos = traci.vehicle.getLanePosition(self.name)
        self.length = traci.vehicle.getLength(self.name)
        if self.isTracked == True: self.tracker(traci)
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

        current_lane = traci.vehicle.getLaneIndex(self.name)
        ''' Update action is determined by current state
            Check if activley changing lane, 
            Send request to neighbor cars to ask if lane change is safe
            Wait on response packet from neighbor cars packet 
        '''
        if self.state == VehicleState.ChangingLane:
            # Update state when lane change is finished
            if current_lane == self.targetLane:
                if self.isTracked: print("FINISHED LANE CHANGE")
                self.state = VehicleState.Idle
                self.targetLane = 0
        elif self.state == VehicleState.SendingRequest:
            # change lane if necessary and send request. if ego car is sending a request, then it wants to change lane
            if self.targetLane == 0:
                # cancel lane change if no target lane was set
                self.state = VehicleState.Idle
                return
            # Goal: send request to 4 neighbor cars, follower/leader in target lane and follower/leader in original lane
            # iterate over all cars in target lane and send nearest rear and nearest leader a request
            #traci.vehicle.highlight(self.name, color=(104, 171, 31)) #green

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
            Send a request packet for lane change to the neighbor vehicles. 
            Request packet wil be processed by neighbor vehicle and contains data to determine if gap is safe
            if RT vehicle agrees, set its max headway and decel 
            """
            requestsSent = 0
            def send_pkt(target_id, dist, neighbor):
                # Send packet only if there is a neigbor car of that type
                if target_id:
                    reported_speed = self.speed #- (self.lyingFactor * 20)
                    reported_dist = dist #+ (self.lyingFactor * 30)
                    reported_accel = self.accel# - (self.lyingFactor * 10)

                    pkt = RequestPacket(self.name, neighbor, self.state, reported_speed, reported_dist, reported_accel, self.decel, self.pos, self.lane)
                    self.vehicle_requests[target_id].put(pkt)
                    if self.isTracked: print(pkt)
                    self.requestsSent += 1

            # Packet request to target rear car
            send_pkt(vehRT[0], vehRT[1], 0)
            send_pkt(vehLT[0], vehLT[1], 1)
            send_pkt(vehRO[0], vehRO[1], 2)
            send_pkt(vehLO[0], vehLO[1], 3)

            ''' 
            Ego car which requested to change lanes processes Acks 
            Check to see if lane change should occur and do lane change
            '''
            if requestsSent == 0:
                # No neighbor cars, free to change lanes
                self.laneSwitch(traci)
            elif self.ackCount == requestsSent:
                # all requests have been acknowledge, change lane
                # set gap for rear car in target lane
                if vehRT[0]:
                    traci.vehicle.openGap(vehID = vehRT[0],newTimeHeadway =  t_rdsafe, newSpaceHeadway= self.delta_d_rt, duration = 3.0, changeRate = a_rdcon, referenceVehID=self.name)
                self.laneSwitch(traci)
            else:
                # TODO check if this is redundant 
                # not all acks have been recieved
                self.state = VehicleState.WaitingOnAck
    
        elif self.state == VehicleState.WaitingOnAck:
            #keep waiting
            # TODO is this redundant?
            self.state = VehicleState.WaitingOnAck
        
        ''' 
        Process recieved packets in ego car queue. ego car is the reciever
        Requests    -   ego car is the neighbor to the sender car who wants to change lanes
                    -   payload is data of requesting car 
                    -   minimum safe gap between sender and reciever is calculated by reciever using the payload
                    -   Response is sent back to sender
        
        Responses   -   ego car is the car that requested lane change from neighbor sender car 
                    -   payload is minimum safe distance between sender and reciever 
                    -   safe distance is compared to current gap between sender and vehicle and ego car
        '''
        while not self.vehicle_requests[self.name].empty():
            # get the next packet in the queue
            packet = self.vehicle_requests[self.name].get_nowait()
            
            # respond to packets requesting safe minimum distance
            # packet.[data] is from the car trying to get over, self.[data] is from a neighbor car
            if packet.type == "R":
                # default safe distance is extremly large to prevent lane change in case of error
                safeDistance = float('inf')
                safeDistCheck = False #TODO: SHOULD THIS BE FALSE BY DEFAULT??
                # Determine which neighbor sent the request to determine which min safe gap calculation to use
                if packet.neighbor == 0 and self.state != VehicleState.ChangingLane:
                    # Response from RT neighbor
                    safeDistance = self.findRTsafeDist(self.speed, packet.speed, self.decel, 1)
                    self.delta_d_rt = safeDistance
                elif packet.neighbor == 1 and self.state != VehicleState.ChangingLane:
                    #safeDistance = self.findLTsafeDist(self.speed, packet.speed, self.decel, packet.decel, 1)
                    safeDistance = self.findLTsafeDist(self.speed, packet.speed, self.decel, 1)
                    # TODO : could prevent emergency breaking or cancel lane change in case of emergency breaking
                elif packet.neighbor == 2: 
                    # packet reponse is from original lane follower
                    headway = packet.reportedGap / packet.speed if packet.speed > 0 else float('inf')
                    safeDistance = self.findXOsafeDist(self.speed, packet.speed, self.decel, 1)
                elif packet.neighbor == 3:
                    # packet reponse is from original lane leader
                    headway = packet.reportedGap / packet.speed if packet.speed > 0 else float('inf')
                    safeDistance = self.findXOsafeDist(self.speed, packet.speed, self.decel, 1)
                if packet.reportedGap > safeDistance:
                    safeDistCheck = True

                infoPacket = ReturnPacket(
                    sender=self.name,
                    neighbor = packet.neighbor,
                    state = self.state,
                    reportedGap = packet.reportedGap,
                    safeDistance = safeDistance,
                    safeDistanceCheck = safeDistCheck
                )
                self.vehicle_requests[packet.sender].put(infoPacket)
                if self.isTracked: 
                    print(infoPacket)
            
            elif packet.type == "A":
                if packet.safeDistanceCheck:
                    self.ackCount += 1
                else: 
                    # if any gap is too small, then cancel requests
                    self.ackCount = 0
                    self.state = VehicleState.Idle
    
    def laneChagneTest (self, traci):
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
                    changeLane = 1

        # Change to right lane, cant already be in right most lane
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
                    changeLane = -1
        return changeLane
    
    def laneSwitchStart(self, target):
        self.state = VehicleState.SendingRequest
        if self.isTracked: print("REQUEST LANE CHANGE")
        self.targetLane = target+self.lane

    def laneSwitch(self, traci):
        self.state = VehicleState.Idle
        self.ackCount = 0
        self.state = VehicleState.ChangingLane  # Set to changing state
        if self.isTracked: print("STARTED LANE CHANGE ")
        traci.vehicle.changeLane(
            vehID=self.name, laneIndex=self.targetLane, duration=2.0
        )
        self.targetLane = 0
    

    def findRTsafeDist(self,v_rd, v_k, rt_decel,c_style):
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
        
        # Find min safe distance between ego and rear
        if v_rd -v_k >=-5:
            t1 = c_style*(v_rd-v_k)*t_reaction +3*c_style*(v_rd - v_k)**2/(2*rt_decel) + c_style*t_rdsafe*v_rd
            t2 = c_style*t_rdsafe*v_rd
            delta_d_mrdh = max(t1, t2)
        else:
            delta_d_mrdh = c_style*t_rdsafe*v_rd
        return delta_d_mrdh

    # d=vt
    def findLTsafeDist(self,v_rd, v_k, rt_decel,c_style):
        safeTime = 2
        # its safe to get over if velocity of ego car is faster then rear car
        relative_v = abs(v_k-v_rd)
        safeDist = relative_v*safeTime
        return safeTime
    def findXOsafeDist(self,v_rd, v_k, rt_decel,c_style):
        safeTime = 2
        # its safe to get over if velocity of ego car is faster then rear car
        relative_v = abs(v_k-v_rd)
        safeDist = relative_v*safeTime
        return safeDist
    # def findLTsafeDist(self, v_ld, v_h, lt_decel, ego_decel,c_style):
    #     """
    #     Find the minimum safe distance between ego-vehicle and leader vehicle in target lane
    #     From eq (16)
    #     v_ld rear vechicle in target lane speed before lane change
    #     v_h ego vechicles speed befor lane change
    #     cStyle ego driving style
    #     """
    #     # avg paramters for drivers and vehicles
    #     # some of these could come from v2v
    #     # TODO these may be able to be pulled from car info
    #     t_driver = 1.35 # driver's reaction time
    #     t_brake = 0.15 # acting time of braking system
    #     t_reaction = t_driver + t_brake

    #     # TODO: read through paper more to figure out what these calues should be
    #     t_ldsafe = 1.8 # rear vehicle time headway
        
    #     t1 = c_style*v_h*t_reaction+c_style*v_h**2/2*(ego_decel) -c_style*v_ld**2/(2*lt_decel)
    #     t2 = c_style*v_h*t_ldsafe
    #     delta_d_mldh = min(t1,t2)
    #     return delta_d_mldh
    
    # def findXOsafeDist(self, v_h, t_xosafe, c_style):
    #     """
    #     Find the minimum safe distance between ego-vehicle and vehicle in original lane
    #     From eq (16)
    #     v_h ego vechicles speed befor lane change
    #     t_xosafe safe time headway between ego and front/rear vehicle in original lane
    #         this may or may not actually be a passed arg
    #     cStyle ego driving style
    #     """

    #     delta_d_xoh = c_style*v_h*t_xosafe
    #     return delta_d_xoh
    def getNeighbors(self,traci):
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
        neghborList = [vehRT, vehLT, vehRO, vehLO]
        return neghborList

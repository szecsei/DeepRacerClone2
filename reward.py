def reward_function(params):

    '''
    @on_track (boolean) :: The vehicle is off-track if the front of the vehicle is outside of the white
    lines
    @x (float range: [0, 1]) :: Fraction of where the car is along the x-axis. 1 indicates
    max 'x' value in the coordinate system.
    @y (float range: [0, 1]) :: Fraction of where the car is along the y-axis. 1 indicates
    max 'y' value in the coordinate system.
    @distance_from_center (float [0, track_width/2]) :: Displacement from the center line of the track
    as defined by way points
    @heading (float: [-180, 180]) :: yaw of the car with respect to the car's x-axis in
    radians (-180 degrees, 180 degrees)
    @progress (float: [0,1]) :: % of track complete
    @steps (int) :: numbers of steps completed. One step is one move by the car
    @speed :: (float) 0 to c In m/s
    @steering_angle :: (float) -30 to 30 (-30 is right, 30 is left)
    @track_width (float) :: width of the track (> 0)
    @waypoints (ordered list) :: list of waypoint in order; each waypoint is a set of coordinates
    (x,y,yaw) that define a turning point. Defines center.
    @closest_waypoints (int,int) :: index of the closest waypoint (0-indexed) given the car's x,y
    position as measured by the eucliedean distance
    
    @is_left_of_center (bool) :: is left of center
    @@output: @reward (float [-1e5, 1e5])
    
    if we assume 15 frames per second, and our goal is under 10 seconds, and we assume that
    each frame corresponds to one step, our target is 150 frames total
    '''
    on_track = params['all_wheels_on_track']
    x = params["x"]
    y = params["y"]
    distance_from_center = params["distance_from_center"]
    heading = params["heading"]
    progress = params["progress"]/100
    steps = params["steps"]
    speed = params["speed"]
    steering = params["steering_angle"] /30
    track_width = params["track_width"]
    waypoints = params["waypoints"]
    is_left_of_center = params["is_left_of_center"]
    closest_waypoints = params["closest_waypoints"]
    '''
    Ideas:
    Incentivize efficiency using the progress and the number of steps
    Incentivize speed while going straight
    Incentivize being roughly in the center of the track
    '''
    import math
    from statistics import mean
    SPEED_MAX = 5
    CURVING_SPEED_MAX=3
    ##########
    # Settings
    ##########
    # Min / Max Reward
    REWARD_MIN = .01
    REWARD_MAX = 100000
    TARGET_STEPS = 150
    
    # Define the Area each side of the center that the card can use.
    # Later version might consider adjust this so that it can hug corners
    CENTER_LANE = track_width * .3


    ABS_STEERING_THRESHOLD = .85

    ####################
    # Locations on track
    ####################

    # Set Base Reward
    if not on_track: # Fail them if off Track
        return REWARD_MIN
    elif progress == 1:
        #the lap is complete.  if we use more steps than TARGET_STEPS, the reward is lower
        return REWARD_MAX * TARGET_STEPS / steps
    elif progress < .1:
        #assuming that the car starts on a straight track, set the base reward to be the maximum
        #we will adjust things later on
        reward = REWARD_MAX
    elif steps > 0:        # we want the vehicle to continue making progress
        reward = REWARD_MAX * max(progress, 0.4)
    
    #Check if the car is turning.  the reward will be computed differently for driving straight and turning.
    #this is Tanner's calculation for determining if the car is turning
    correction = 0
    next_next_point = waypoints[min(closest_waypoints[1]+1, len(waypoints)-1)]
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]  
    
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]) 
    # Convert to degree
    track_direction = math.degrees(track_direction)  
    
    track_direction_next = math.atan2(next_next_point[1] - next_point[1], next_next_point[0] - next_point[0]) 
    # Convert to degree
    track_direction_next = math.degrees(track_direction)
  
    if( abs(track_direction-track_direction_next) > 3):
        correction = True
    else:
        correction = False
    
    ##########
    # On straight
    ##########
    if not correction:
        #adjust reward based on speed
        if speed < SPEED_MAX*.5:
            #we are going straight, but not fast enough, so we will reduce the reward
            #this function will be a linear increase from reward/4 to reward/2
            reward *= 0.5 * (0.5 + speed/SPEED_MAX)
        else:
            #if the speed is .5 of SPEED_MAX or greater, this function will multiply the reward by a value between
            #0.8 and 1.2
            reward *= 16.0 * 4**(0.585 * speed/ SPEED_MAX)/30 
            
        #adjust reward based on track position
        if distance_from_center < track_width/4:
            #the position is good, so bump the reward up 30%
            reward = reward * 1.3
        elif distance_from_center < track_width/3:
            #getting closer to the edge, so only increase the reward a bit
            reward = reward * 1.10
        elif distance_from_center < track_width/2:
            #getting close to the edge, so decrease the reward a bit
            reward = reward * 0.9
        else:
            #too close to the edge?  really discourage this position
            reward = reward * .01
                
    ##########
    # Around Curve
    ##########
    else:
        #just adjust reward based on track position and speed
        if distance_from_center < track_width/2.25:
            #give it a boost for being in a good position, and a boost for going fast
            reward = reward*1.10 + 10000*(speed/CURVING_SPEED_MAX)**2
        else:
            #too close to the edge? drop the reward
            reward = reward * .01


    # make sure reward value returned is within the prescribed value range.
    reward = max(reward, REWARD_MIN)
    reward = min(reward, REWARD_MAX)

    return float(reward)

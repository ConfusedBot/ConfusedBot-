import math
import time
from Util import *
from States import *

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket


class ConfusedBot(BaseAgent):

    def initialize_agent(self):
        self.me = obj()
        self.ball = obj()
        self.players = []  # holds other players in match
        self.start = time.time()

        self.state = trtlShot()
        self.controller = calcController
		
    def checkState(self):
        if self.state.expired:
            if calcShot().available(self) == True:
                self.state = calcShot()
            elif trtlShot().available(self) == True:
                self.state = trtlShot()
            elif quickShot().available(self) == True:
                self.state = quickShot()
            elif wait().available(self) == True:
                self.state = wait()
            else:
                self.state = quickShot()

    def get_values(self, values):
        self.controller.boost = False
        if self.jump_time_end < self.game_time:
            self.controller.jump = False

        if self.stop_flick:
            self.controller.pitch = 0
            self.controller.roll = 0
            self.flick_time = 0
            self.stop_flick = False

        if self.flick_time < self.game_time and self.flick_time != 0:
            self.controller.jump = True
            self.stop_flick = True


def get_output(self, game: GameTickPacket) -> SimpleControllerState:
    self.preprocess(game)
    self.checkState()
    return self.state.execute(self)


def preprocess(self, game):
    self.players = []
    car = game.game_cars[self.index]
    self.me.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
    self.me.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
    self.me.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
    self.me.rvelocity.data = [car.physics.angular_velocity.x, car.physics.angular_velocity.y,
                              car.physics.angular_velocity.z]
    self.me.matrix = rotator_to_matrix(self.me)
    self.me.boost = car.boost

    ball = game.game_ball.physics
    self.ball.location.data = [ball.location.x, ball.location.y, ball.location.z]
    self.ball.velocity.data = [ball.velocity.x, ball.velocity.y, ball.velocity.z]
    self.ball.rotation.data = [ball.rotation.pitch, ball.rotation.yaw, ball.rotation.roll]
    self.ball.rvelocity.data = [ball.angular_velocity.x, ball.angular_velocity.y, ball.angular_velocity.z]

    self.ball.local_location = to_local(self.ball, self.me)

    # collects info for all other cars in match, updates objects in self.players accordingly
    for i in range(game.num_cars):
        if i != self.index:
            car = game.game_cars[i]
            temp = obj()
            temp.index = i
            temp.team = car.team
            temp.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
            temp.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
            temp.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
            temp.rvelocity.data = [car.physics.angular_velocity.x, car.physics.angular_velocity.y,
                                   car.physics.angular_velocity.z]
            self.me.boost = car.boost
            flag = False
            for item in self.players:
                if item.index == i:
                    item = temp
                    flag = True
                    break
            if flag:
                self.players.append(temp)
				
def predict_ball(Location,Velocity,Spin,time):
	gravity = 650
	ball_radius = 93
	max_ball_speed = 6000
	bounce_multiplier = 0.6
	air_resistance = 0.03
	surface_friction = 230
	side_wall = 4100
	backboard = 5140
	ceiling = 2050
	floor = 0
	goal_width = 900
	goal_height = 645
	Location,Velocity,Spin = a3(Location),a3(Velocity),a3(Spin)
	predicted_location = np.array([0,0,0])

	predicted_location = Location + Velocity*time
	predicted_velocity = Velocity

	# # # Bounces :
	if abs(predicted_location[0]) + ball_radius > side_wall and predicted_location[2]>200 :
		predicted_location[0] = RangeBounce(predicted_location[0],bounce_multiplier,ball_radius,side_wall)
		predicted_velocity[0] *= -bounce_multiplier
		angle_xy = math.atan2(abs(Velocity[0]),abs(Velocity[1]))/math.pi
		predicted_velocity[1] *=0.77
		predicted_velocity[2] *=0.77
	if abs(predicted_location[1]) + ball_radius > backboard and predicted_location[2]>200 and (abs(predicted_location[0])>goal_width or abs(predicted_location[2])>goal_height) :
		predicted_location[1] = RangeBounce(predicted_location[1],bounce_multiplier,ball_radius,backboard)
		predicted_velocity[1] *= -bounce_multiplier
		predicted_velocity[0] *=0.77
		predicted_velocity[2] *=0.77
	if predicted_location[2] + ball_radius > ceiling or predicted_location[2] - ball_radius - 1 < floor :
		predicted_location[2] = RangeBounce(predicted_location[2],bounce_multiplier,ball_radius,ceiling)
		predicted_velocity[2] *= -bounce_multiplier
		predicted_velocity[0] *=0.77
		predicted_velocity[1] *=0.77
		if predicted_location[2] -ball_radius < floor: predicted_location[2] = -(predicted_location[2]-ball_radius)*0.6 + ball_radius
		predicted_velocity[2] *= -bounce_multiplier

	if Location[2]>ball_radius+1 :
		# Gravity
		if abs(predicted_velocity[2])>0.1 : # if not floating
			predicted_velocity[2] -= gravity*time
	else :
		# Sliding Friction
		for i in range(2):
			if abs(predicted_velocity[i])>565:
				predicted_velocity[i] -= 230*math.copysign(1,predicted_velocity[i])

	# Air Resistance
	predicted_velocity *= (1-air_resistance*time)

	
	 # restricting abs Velocity to less than the maximum ball speed :
	predicted_velocity = Range(predicted_velocity,max_ball_speed)
	return predicted_location, predicted_velocity
def RangeBounce(value,multiplier,radius,R):
	value += math.copysign(radius,value)
	if abs(value)>R:
		value = math.copysign(R - abs(R-abs(value))*multiplier,value)
	value -= math.copysign(radius,value)
	return value

gravity = 655
ball_radius = 92.8
max_ball_speed = 6000
bounce_multiplier = 0.6
air_resistance = 0.03072
surface_friction = 230
side_wall = 4100
back_wall = 5140
ceiling = 2055
floor = 0
goal_width = 910
goal_height = 645

def _collision(Location,Velocity,excdim=3):
	global ball_radius,side_wall,back_wall,ceiling,floor,goal_width,goal_height
	
	c=[]
	c.append([predict_z_impact(Location[2],Velocity[2],ball_radius),ball_radius,2])
	c.append([predict_z_impact(Location[2],Velocity[2],ceiling-ball_radius),ceiling-ball_radius,2])
	c.append([predict_xy_impact(Location[0],Velocity[0],side_wall-ball_radius),side_wall-ball_radius,0])
	c.append([predict_xy_impact(Location[0],Velocity[0],-side_wall+ball_radius),-side_wall+ball_radius,0])
	c.append([predict_xy_impact(Location[1],Velocity[1],back_wall-ball_radius),back_wall-ball_radius,1])
	c.append([predict_xy_impact(Location[1],Velocity[1],-back_wall+ball_radius),-back_wall+ball_radius,1])

	for i in range(len(c)):
		if i == 0:
			for _ in range(len(c)):
				if c[_][0]!=excdim:
					impact_time = c[_][0]
					impact_point = c[_][1]
					idim = c[_][2]
					break

		if c[i][0]>0 and c[i][0]<impact_time and c[i][0]!=excdim :
			impact_time = c[i][0]
			impact_point = c[i][1]
			idim = c[i][2]

	return impact_time,impact_point,idim
def predict_equation(Location,Velocity,time):
	global ball_radius,max_ball_speed,bounce_multiplier,air_resistance,surface_friction,side_wall,back_wall,ceiling,floor,goal_width,goal_height

	gravity = 655
	time=time+0.0075*time
	g = np.array([0,0,1])

	if d3(Velocity,[0,0,0])==0 : 
		gravity = 0

	for i in range(30):

		impact_time, impact_point, idim = _collision(Location,Velocity)

		## Bounces 
		if impact_time<time and time!=0  and impact_time>=0 :
	
			impact_location = Location + (Velocity*(1-air_resistance*impact_time*(0.35)) - 0.5*gravity*impact_time*g)*impact_time
			impact_velocity = Velocity*(1-air_resistance*impact_time) - 0.5*gravity*impact_time*g

			if not at_goal(impact_location):
				
				time = time - impact_time

				impact_velocity[idim] *= -0.6

				if abs(impact_velocity[idim])>25:
					impact_velocity[(idim+1)%3] *= 0.725
					impact_velocity[(idim-1)%3] *= 0.725

				Location = impact_location
				Velocity = impact_velocity

				if abs(Velocity[2])<2 and Location[2]<ball_radius+1: 
					gravity=0
					Location[2]=ball_radius
					Velocity[2]=0
					break

				if time>=5e-3:
					Location = Location + (Velocity*(1-air_resistance*5e-3*(0.35)) - (0.5)*gravity*5e-3*g)*5e-3
					Velocity = Velocity*(1-air_resistance*5e-3) - gravity*5e-3*g
					time-=5e-3
			else:
				break
		else:
			break


	predicted_location = Location + (Velocity*(1-(10*air_resistance**2)*time) - (0.5)*gravity*time*g)*time
	predicted_velocity = (Velocity - gravity*time*g)*(1-air_resistance*time)

	# freezing the ball on goal line
	if at_goal(predicted_location):	
		impact_time+=0.06
		predicted_location = Location + (Velocity*(1-air_resistance*impact_time*(0.35)) - (0.5)*gravity*impact_time*g)*impact_time
		predicted_velocity = Velocity - gravity*impact_time*gravity

	return predicted_location, predicted_velocity

def checkState(self):
    if self.state.expired:
        if calcShot().available(self) == True:
            self.state = boost()
        elif trtlShot().available(self) == True:
            self.state = boost()
        elif quickShot().available(self) == True:
            self.state = boost()

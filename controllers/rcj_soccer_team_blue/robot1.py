# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
from cmath import atan
import math
from os import times_result  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from time import time,sleep
from PID import PID
from math import sqrt,atan2
class MyRobot1(RCJSoccerRobot):
    def run(self):
        #****************************mycode
        T=0.0010
        L=0.085
        R=0.02
        xd=-0.45
        yd=0.0
        vl=0.0
        vr=0.0
        e_r=[0,0] 
        e_p=[0,0]
        robot_pos = self.get_gps_coordinates()
        robot_com=self.get_compass_heading()
        C_r=PID(kd=0.05,kp=0.15,ki=0.01,T=T)     
        C_p=PID(kd=0.0,kp=0.05,ki=0.1,T=T)
        #****************************my code
        while self.robot.step(TIME_STEP) != -1:
            start=time()
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841

                self.left_motor.setVelocity(10) # self code
                #******************************************************************mycode
                #er=self.get_new_ball_data()
                robot_pos = self.get_gps_coordinates()
                robot_com=self.get_compass_heading()
                xd=ball_data['direction'][1]
                yd=ball_data['direction'][0]
                e_r[1]=sqrt((robot_pos[0]-yd)**2+(robot_pos[1]-xd)**2)
                e_p[1]=robot_com-atan2(yd-robot_pos[0],xd-robot_pos[1])
                u=C_r.run(e_r)
                w=C_p.run(e_p)
                vr_=(2*u[1]+L*w[1])/(2*R)
                vl_=(2*u[1]-L*w[1])/(2*R)
                print(ball_data)
                #print(f"xd-x :{robot_pos[1]-xd} ,yd-y :{robot_pos[0]-yd}")
                #****************dead zone:
                if(abs(vl_)>0.5):
                    vl=vl_
                else:
                    vl=vl*0.8
                if(abs(vr_)>0.5):
                    vr=vr_
                else:
                    vr=vr*0.8
                self.left_motor.setVelocity(vl)
                self.right_motor.setVelocity(vr)
                end=time()
                sleep(T-(end-start))
                #******************************************************************my code
                # Compute the speed for motors
                '''direction = utils.get_direction(ball_data["direction"])

                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise
                if direction == 0:
                    left_speed = 7
                    right_speed = 7
                else:
                    left_speed = direction * 4
                    right_speed = direction * -4

                # Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)

                # Send message to team robots
                self.send_data_to_team(self.player_id)'''
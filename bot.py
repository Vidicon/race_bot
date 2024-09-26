from typing import Tuple
from copy import deepcopy
import socket
import json 
import math
import os

from pygame import Vector2, Color, Surface
import pygame
from ...track import Track

from ...bot import Bot
from ...linear_math import Transform

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 5123
addr = (HOST, PORT)

TORAD = math.pi / 180



class BrumBot(Bot):
    def __init__(self, track: Track):
        self.track = track
        self.last_position = Transform()
        self.history = []
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.settimeout(0.01)
        self.font = pygame.font.SysFont(None, 24)

        self.dtrack, self.checkpoints, self.cpoints = self.upscale_path_with_bezier(self.track.lines) # 
        self.dtrack = self.smooth_path(self.dtrack, 10)
        self.angles, self.rel_angles = self.calculate_relative_angles(self.dtrack)

        self.next_waypoint = 0
        self.steerahead = 0

        self.lookahead = 20
        self.throttle = 0.0

        self.bull = pygame.image.load(
                os.path.dirname(__file__) + '/bull.png')

    def __del__(self):
        self.s.close()

    def plot(self, data):
        json_string = json.dumps(data, ensure_ascii=False)
        self.s.sendto(json_string.encode(), addr)
    
    def bezier3(self, p0, p1, p2, t):
        return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2
    
    def bezier4(self, p0, p1, p2, p3, t):
        return (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3


    # def upscale_path_with_bezier(self, points):
    #     if len(points) < 3:
    #         return points  # Not enough points for a curve
        
    #     upscaled_points = []
    #     cpoints = []

    #     # Iterate over consecutive triplets of points (P0, P1, P2)
    #     for i in range(len(points)):
    #         section = points[i] - points[(i - 1)%len(points)]
    #         section2 = points[(i + 1)%len(points)] -  points[i]
    #         section3 = points[(i + 2)%len(points)] -  points[(i + 1)%len(points)] 
    #         angle = math.atan2(section.y, section.x)
    #         angle2 = math.atan2(section3.y, section3.x)
    #         p0,  p2 = points[i],points[(i+1)%len(points)]
    #         p10 = p2 + Vector2(math.cos(angle2 - math.pi), math.sin(angle2 - math.pi)) * min(section2.length() * 0.4, 5000)
    #         p11 = p0 + Vector2(math.cos(angle), math.sin(angle)) * min(section2.length() * 0.4, 5000)
    #         px = p10.lerp(p11, 0.5)
    #         # px = p0 + 
    #         p1 = px
            
    #         cpoints.append(p10)
    #         cpoints.append(p10)
    #         # math.cos(section) 
    #         # angle
    #         steps = max(int(section2.length() / 10), 5)
    #         # print(f"-----------------")
    #         for t in range(steps):
    #             t /= steps
    #             # print(f"t: {t} steps: {steps}")
    #             # upscaled_points.append(self.bezier3(p0, p1, p2, t))
    #             upscaled_points.append(self.bezier4(p0, p11, p10, p2, t))
        
    #     return upscaled_points, cpoints
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle
    
    def upscale_path_with_bezier(self, points):
        if len(points) < 3:
            return points  # Not enough points for a curve
        
        upscaled_points = []
        checkpoints = []
        cpoints = []

        # Iterate over consecutive triplets of points (P0, P1, P2)
        for i in range(len(points)):
            section_before = points[i] - points[(i - 1)%len(points)]
            section_current = points[(i + 1)%len(points)] -  points[i]
            section_after = points[(i + 2)%len(points)] -  points[(i + 1)%len(points)] 
            angle_before = math.atan2(section_before.y, section_before.x)
            angle_curent = math.atan2(section_current.y, section_current.x)
            angle_after = math.atan2(section_after.y, section_after.x)

            angle_in  = ((angle_before + angle_curent) / 2)
            angle_out = ((angle_curent + angle_after) / 2)

            if abs(angle_before - angle_curent) > math.pi:
                angle_in += math.pi

            if abs(angle_curent - angle_after) > math.pi:
                angle_out += math.pi
            

            # print(f"angle: {angle_in} angleD: {angle_before - angle_curent} angle_before: {angle_before}, angle_curent: {angle_curent} ")

            p1 = points[i] 
            p2 = points[i] + Vector2(math.cos(angle_in), math.sin(angle_in)) * section_current.length() * 0.3

            p3 = points[(i+1)%len(points)] + Vector2(math.cos(angle_out+math.pi), math.sin(angle_out+math.pi)) * section_current.length() * 0.3
            p4 = points[(i+1)%len(points)]
          
            cpoints.append([p1, p2, p3, p4])
            
            # cpoints.append(p4)
            # math.cos(section) 
            # angle
            steps = max(int(section_current.length() / 10), 5)
            # print(f"-----------------")
            for t in range(steps):
                t /= steps
                upscaled_points.append(self.bezier4(p1, p2, p3, p4, t))
                checkpoints.append(t==0)
        
        return upscaled_points, checkpoints, cpoints
    
    def smooth_path(self, points, x = 10):
        path = []
        for i in range(len(points)):
            p1 = points[int(i-x/2)%len(points)]
            p2 = points[int(i+x/2)%len(points)]
            path.append(p1.lerp(p2, 0.5))
        return path

    def calculate_relative_angles(self, points):
        if len(points) < 3:
            return []  # Not enough points to calculate relative angles
        
        # Calculate angles for each segment
        angles = []
        for i in range(len(points)):
            direction =  points[(i + 1)%len(points)] - points[i]
            angle = math.atan2(direction.y, direction.x)
            x = {
            "angle": angle,
            "length": direction.length()
            }
            angles.append(x)
        
        # Calculate relative angles between consecutive segments
        relative_angles = []
        for i in range(len(angles)):
            relative_angle = angles[(i -1)%len(points)]["angle"] - angles[i]["angle"]
            if relative_angle < math.pi:
                relative_angle += 2*math.pi
            if relative_angle > math.pi:
                relative_angle -= 2*math.pi
            
            relative_angles.append(relative_angle)
            # print(f"angle: {angles[i]['angle']} rel: {relative_angle * 100}")
        
        
        return angles, relative_angles

    @property
    def name(self):
        return "BrumBot"

    @property
    def contributor(self):
        return "Brum"
    
    @property
    def color(self) -> Color:
        return Color('#c302d9')

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        v = velocity.length()
        track_size = len(self.track.lines)-1
        x, y = int(position.p[0]), int(position.p[1])
        self.last_position = position
        self.history.append(position.p)
        self.history = self.history[-100:]

        while (self.dtrack[self.next_waypoint] - position.p).length() < self.track.track_width:
            if self.checkpoints[(self.next_waypoint+1)% len(self.checkpoints)]:
                break
            self.next_waypoint = (self.next_waypoint + 1) % len(self.dtrack)

        i = 1
        last_dist = (self.dtrack[self.next_waypoint] - position.p).length()
        while (self.dtrack[(self.next_waypoint + i) % len(self.dtrack)] - position.p).length() < last_dist:
            if self.checkpoints[self.next_waypoint]:
                break
            self.next_waypoint = (self.next_waypoint + 1) % len(self.dtrack)
            i+=1
        
        
        
        
        # target = self.dtrack[self.next_waypoint]
        # target = position.inverse() * target
        # angle = target.as_polar()[1]

      
        # calculate the throttle
        # print(f" angle: {angle}")
        max_angle = 0.0
        anglex = 0.0

        ku = 0.13

        self.lookahead = max(int(v * ku),1)
        l = 0
        for i in range(self.lookahead):
            index = (self.next_waypoint + i) % len(self.dtrack)
            # target = self.dtrack[(self.next_waypoint + i) % len(self.dtrack)]
            lookangle = abs(self.rel_angles[index])
            if lookangle > 0.01:
                anglex += (lookangle*lookangle)
                l+=1
            max_angle = max(lookangle, max_angle)
        # print(f"l: {l}")
        upcoming_angle = 0.0
        if l:
            upcoming_angle = anglex / l#self.lookahead
        # print(f"upcoming_angle: {upcoming_angle}")
        
     
        # current_angle = abs(self.rel_angles[self.next_waypoint % len(self.dtrack)])
        anglex = 0.0
        self.current_angle_lookahead = 5
        for i in range(self.current_angle_lookahead):
            index = (self.next_waypoint + i) % len(self.dtrack)
            lookangle = abs(self.rel_angles[index])
            anglex += (lookangle*lookangle)
        
        current_angle = anglex / self.current_angle_lookahead
    
          
        target_speed = 150

        max_target_speed = 350
        
        safe_speed = 150
        ka = 400.0 # 280
        kb = 250

        # breakmultiplier = min(breakmultiplier, 1)

        breakmultiplier = upcoming_angle * max(velocity.length() - safe_speed, 0)

        #base speed
        # target_velocity = target_speed 

        # add speed if no upcoming angle
        # free_track_speed = (max_target_speed - target_speed) * (1 / (1+current_angle*ka))
        # target_velocity += free_track_speed

        target_velocity = 420 
        ka = 800.0 # 280
        
        free_track_speed = current_angle*ka* max(velocity.length() - safe_speed, 0)
        target_velocity -= free_track_speed

        # break if upcoming angle
        break_reduction = breakmultiplier * kb
        target_velocity -= break_reduction


        kp = 0.12
        throttle = (target_velocity - velocity.length()) * kp

        ##################################################################################
        # Steering
        ##################################################################################
        kas = 0.06
        self.steerahead = int(max(v-150, 0) * kas )
        # self.steerahead += int(max_angle*0.1)
        next_target = self.dtrack[(self.next_waypoint + self.steerahead) % len(self.dtrack)]
        next_target = position.inverse() * next_target
        next_angle = next_target.as_polar()[1]
        steer = next_angle

        throttle = self.clamp(throttle, -1, 1)
        steer = self.clamp(steer, -1, 1)

        dis = next_target.length()
        self.throttle = throttle
        # calculate the steering
        data = {
            "x": position.p[0],
            "y": -position.p[1],
            "path_dis": dis,
            "upcoming_angle": upcoming_angle,
            "free_track_speed": free_track_speed,
            "break_reduction": break_reduction,
            "throttle": throttle,
            "velocity": velocity.length(),
            "steer": steer,
            "target_velocity": target_velocity,
            "max_angle": max_angle,
            "breakmultiplier": breakmultiplier
        }
        self.plot(data)

        return throttle, steer
    
    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)
    
    def angle_to_color(self, angle, power = 80):
        s = int(angle*power)
        r = 255 - max(s,0)
        g = 255 + min(s,0)
        b = 255 - self.clamp(s, 0 , 255)
        return self.clamp(r, 0 , 255), self.clamp(g, 0 , 255), self.clamp(b, 0 , 255)
    
    def draw(self, map_scaled: Surface, zoom):
        
        # print(f" leng: {len(self.track.lines)}, len2: {len(angles)}")
        # print(angles)
        # for point in self.track.lines:
        #     pygame.draw.circle(map_scaled, (255,0,0), (point[0]* zoom, point[1]* zoom) ,self.track.track_width * zoom, 1)
       
        # pygame.draw.rect(map_scaled, Color('white'), (30, map_scaled.get_height() / 2, 40,  20))
        # pygame.draw.rect(map_scaled, Color('red'), (30, map_scaled.get_height() / 2 - self.throttle * 50, 40,  20))


        # target = self.dtrack[self.next_waypoint]
        # pygame.draw.circle(map_scaled, (255,100,200), (target[0]* zoom, target[1]* zoom) ,self.track.track_width * zoom, 1)

        # target = self.dtrack[(self.next_waypoint+ self.steerahead) % len(self.dtrack)]
        # pygame.draw.circle(map_scaled, (255,200,100), (target[0]* zoom, target[1]* zoom) ,10, 3)

        # target = self.dtrack[(self.next_waypoint+ self.lookahead) % len(self.dtrack)]
        # pygame.draw.circle(map_scaled, (100,200,255), (target[0]* zoom, target[1]* zoom) ,10, 3)
        # return
        # i = 0
        # for point in self.dtrack:
        #     angle = self.angles[i%len(self.angles)]["angle"]
        #     length = self.angles[i%len(self.angles)]["length"]
        #     rel_angle = self.rel_angles[i%len(self.rel_angles)]

        #     c = self.angle_to_color(rel_angle, 1000)
        #     pygame.draw.circle(map_scaled, c, (point[0]* zoom, point[1]* zoom) ,4)
            
            
        #     pygame.draw.line(map_scaled, c, (point[0]* zoom, point[1]* zoom), (point[0]* zoom + math.cos(angle)*length*0.2 * zoom, point[1]* zoom + math.sin(angle)*length*0.2*zoom), 2)

        #     # text = self.font.render(f'{angle:.2f}', True, (255, 255, 200))
        #     # map_scaled.blit(text, (point[0]* zoom - 20, point[1]* zoom - 15))

        #     # text = self.font.render(f'{rel_angle:.2f}', True, (0, 0, 0))
        #     # map_scaled.blit(text, (point[0]* zoom - 20, point[1]* zoom + 15))
            
            
        #     i+=1

        # i =0
        # for point in self.cpoints:
        #     i+=1
        #     # if i%2 != 0:
        #     #     continue
        #     pygame.draw.circle(map_scaled, (255,0,0),   (point[0][0]* zoom, point[0][1]* zoom) ,7)
        #     pygame.draw.circle(map_scaled, (0,255,0),   (point[1][0]* zoom, point[1][1]* zoom) ,7)
        #     pygame.draw.circle(map_scaled, (0,0,255), (point[2][0]* zoom, point[2][1]* zoom) ,7)
        #     pygame.draw.circle(map_scaled, (255,255,0), (point[3][0]* zoom, point[3][1]* zoom) ,7)
        #     pygame.draw.lines(map_scaled, (100, 200, 100), False, [zoom * p for p in point], 2)
        
        # pygame.draw.lines(map_scaled, (0, 0, 0), False, [zoom * p for p in self.track.lines], 2)
        if len(self.history) > 1:
            pygame.draw.lines(map_scaled, (100, 0, 0), False, [zoom * p for p in self.history], 2)


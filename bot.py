from typing import Tuple
from copy import deepcopy
import socket
import threading
import json
import math
from math import radians
import os

from pygame import Vector2, Color, Surface
import pygame
from ...track import Track

from ...bot import Bot
from ...linear_math import Transform, Rotation
from ...car_info import CarPhysics
from racer.constants import framerate

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 5123 # logger port
PORT2 = 6123 # parameter server port
addr = (HOST, PORT)

TORAD = math.pi / 180

class BotSim():
    def __init__(self, bot):
        self.bot = bot
        self.track = []
        # starting_angle = (self.bot.track.lines[1] - self.bot.track.lines[0]).as_polar()[1]

        # self.car_physics = CarPhysics(
        #     Transform(Rotation.fromangle(radians(starting_angle)), deepcopy(self.bot.track.lines[0])),
        #     Vector2())

    def reset(self):
        pass

    def run_sim(self, max_rounds = (60*60)):
        bot = BrumBot(deepcopy(self.bot.track))
        starting_angle = (self.bot.track.lines[1] - self.bot.track.lines[0]).as_polar()[1]
        car_physics = CarPhysics(
            Transform(Rotation.fromangle(radians(starting_angle)), deepcopy(self.bot.track.lines[0])),
            Vector2())

        results = []
        rounds = 0
        # starting_angle = (self.bot.track.lines[1] - self.bot.track.lines[0]).as_polar()[1]
        # position = Transform(Rotation.fromangle(radians(starting_angle)), deepcopy(self.bot.track.lines[0]))
        # velocity = Vector2(0,0)
        next_waypoint = 0
        while rounds < max_rounds:
            throttle, steer = bot.compute_commands(next_waypoint, deepcopy(car_physics.position), deepcopy(car_physics.velocity))
            car_physics.update(1.0/framerate, throttle, steer)
            if (self.bot.track.lines[next_waypoint] - car_physics.position.p).length() < self.bot.track.track_width:
                next_waypoint += 1
                if next_waypoint >= len(self.bot.track.lines):
                    next_waypoint = 0
                # self.round += 1
            result = {
                "throttle": throttle,
                "steer": steer,
                "position": deepcopy(car_physics.position.p),
                "velocity": car_physics.velocity.length()
            }
            rounds += 1
            results.append(result)

        return results

class RTrack():
    def __init__(self, lines):
        self.lines = lines
        self.track_width = 20
        self.angles = []
        self.rel_angles = []
        self.sharpness = []
        self.checkpoints = []
        self.control_points = []


class BrumBot(Bot):
    def __init__(self, track: Track, *args, **kwargs):
        self.track = track
        self.last_position = Transform()
        self.history = []
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.settimeout(0.01)
        self.font = pygame.font.SysFont(None, 24)
        self.draw_counter = 0

        self.atrack, self.checkpoints, self.cpoints = self.upscale_path_with_bezier(self.track.lines) #
        angles, rel_angles = self.calculate_relative_angles(self.atrack)
        self.btrack = self.track_offset(self.atrack, rel_angles)
        self.dtrack = self.smooth_path(self.btrack, 10)
        self.angles, self.rel_angles = self.calculate_relative_angles(self.dtrack)

        self.next_waypoint = 0
        self.steerahead = 0

        self.lookahead = 20
        self.throttle = 0.0
        self.nwp = 0

        self.sim = BotSim(self)

    def __del__(self):
        self.s.close()

    def plot(self, data):
        json_string = json.dumps(data, ensure_ascii=False)
        # self.s.sendto(json_string.encode(), addr)

    def bezier3(self, p0, p1, p2, t):
        return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2

    def bezier4(self, p0, p1, p2, p3, t):
        return (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def track_offset(self, points, angles):
        offset = 0
        new_points = []
        for i in range(len(points)):

            upcoming_angle = 0
            pased_angle = 0

            dis = 30
            for y in range(dis):
                upcoming_angle += angles[(i+y) % len(angles)]
                pased_angle += angles[(i-y) % len(angles)]
            upcoming_angle /= dis
            pased_angle /= dis



            offset = (upcoming_angle * 200) + (pased_angle * 100) + (pased_angle + upcoming_angle)/2 * 0

            section = points[i] - points[(i - 1)%len(points)]
            angle = math.atan2(section.y, section.x)
            new_points.append(points[i] + Vector2(math.cos(angle+math.pi/2), math.sin(angle+math.pi/2)) * offset)
        return new_points

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
            checkpoints.append(len(upscaled_points))
            for t in range(steps):
                t /= steps
                upscaled_points.append(self.bezier4(p1, p2, p3, p4, t))


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

    def closest_waypoint(self, position, current_waypoint):
        closest = 0
        smallest_distance = 1000000
        for i in range(1, 20):
            i_point = (current_waypoint + i) % len(self.dtrack)
            distance = (self.dtrack[i_point] - position.p).length()
            if distance < smallest_distance:
                smallest_distance = distance
                closest = i_point
            i_point = (current_waypoint - i) % len(self.dtrack)
            distance = (self.dtrack[i_point] - position.p).length()
            if distance < smallest_distance:
                smallest_distance = distance
                closest = i_point

        return closest

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
        self.nwp = next_waypoint

        # while (self.dtrack[self.next_waypoint] - position.p).length() < self.track.track_width:
        #     self.next_waypoint = (self.next_waypoint + 1) % len(self.dtrack)


        last_dist = (self.dtrack[self.next_waypoint] - position.p).length()
        i = 1
        while i < 20:

            i_waypoint = (self.next_waypoint + i)
            nextcheckpoint = self.checkpoints[next_waypoint]

            if (i_waypoint % len(self.dtrack))>=nextcheckpoint and nextcheckpoint != 0: # do not check past the next checkpoint
                # print(f"Skip: i: {i} i_waypoint: {i_waypoint} nextcheckpoint: {nextcheckpoint}")
                break

            distance = (self.dtrack[i_waypoint% len(self.dtrack)] - position.p).length()
            if distance < self.track.track_width*1.2:
                self.next_waypoint = i_waypoint % len(self.dtrack)
                last_dist = distance
                i = 0
            # print(f"i: {i} i_waypoint: {i_waypoint} nextcheckpoint: {nextcheckpoint}")
            i +=1

        #     distance = (self.dtrack[i_waypoint] - position.p).length()
        #     if distance < self.track.track_width: # go to waypoint if in reach
        #         self.next_waypoint = (self.next_waypoint + i) % len(self.dtrack)

        #     # if distance < last_dist: # skip to next waypoint if closer
        #     #     self.next_waypoint = (self.next_waypoint + i) % len(self.dtrack)

        #     last_dist = distance
        #     i +=1


                # last_dist = (self.dtrack[self.next_waypoint] - position.p).length()





        # target = self.dtrack[self.next_waypoint]
        # target = position.inverse() * target
        # angle = target.as_polar()[1]


        # calculate the throttle
        # print(f" angle: {angle}")

        i_close = self.closest_waypoint(position, self.next_waypoint)

        max_angle = 0.0
        anglex = 0.0

        ku = 0.12

        self.lookahead = max(int(v * ku),1)
        l = 0
        for i in range(self.lookahead):
            index = (i_close + i) % len(self.dtrack)
            # target = self.dtrack[(self.next_waypoint + i) % len(self.dtrack)]
            lookangle = abs(self.rel_angles[index])
            if lookangle > 0.05:
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
            index = (i_close + i) % len(self.dtrack)
            lookangle = abs(self.rel_angles[index])
            anglex += (lookangle*lookangle)

        current_angle = anglex / self.current_angle_lookahead


        target_velocity = 400

        # break if upcoming angle
        start_break_speed = 120 # start breaking above this speed
        start_break_angle = 0.0024 # start breaking above this angle
        break_force = 200 # force of breaking
        break_reduction = max(upcoming_angle - start_break_angle, 0) * max(velocity.length() - start_break_speed, 0) * break_force
        target_velocity -= min(break_reduction, target_velocity)



        # free_track_speed = (current_angle*current_angle)*ka* max(velocity.length() - safe_speed, 0)
        # target_velocity -= free_track_speed

        # break if upcoming angle
        # start_reduce_speed = 100 # start breaking above this speed
        start_reduce_angle = 0.000 # start breaking above this angle
        reduce_force = 200 # force of breaking
        reduce_reduction = max(current_angle - start_reduce_angle, 0) * reduce_force

        target_velocity -= min(reduce_reduction, target_velocity)

        kp = 10.22
        throttle = (target_velocity - velocity.length()) * kp
        # throttle -= break_reduction
        # throttle -= reduce_reduction

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
            "reduce_reduction": reduce_reduction,
            "break_reduction": break_reduction,
            "throttle": throttle,
            "velocity": velocity.length(),
            "steer": steer,
            "target_velocity": target_velocity,
            "max_angle": max_angle,
            "current_angle": current_angle
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


        target = self.dtrack[self.next_waypoint]
        pygame.draw.circle(map_scaled, (255,100,200), (target[0]* zoom, target[1]* zoom) ,self.track.track_width * zoom, 1)

        # target = self.dtrack[(self.next_waypoint+ self.steerahead) % len(self.dtrack)]
        # pygame.draw.circle(map_scaled, (255,200,100), (target[0]* zoom, target[1]* zoom) ,10, 3)

        # target = self.dtrack[(self.next_waypoint+ self.lookahead) % len(self.dtrack)]
        # pygame.draw.circle(map_scaled, (100,200,255), (target[0]* zoom, target[1]* zoom) ,10, 3)
        # return
        # i = 0
        # for point in self.dtrack:
        #     angle = self.angles[i%len(self.angles)]["angle"]
        #     length = self.angles[i%len(self.angles)]["length"]
            # rel_angle = self.rel_angles[i%len(self.rel_angles)]

            # c = self.angle_to_color(rel_angle, 1000)
            # pygame.draw.circle(map_scaled, c, (point[0]* zoom, point[1]* zoom) ,4)


        #     pygame.draw.line(map_scaled, c, (point[0]* zoom, point[1]* zoom), (point[0]* zoom + math.cos(angle)*length*0.2 * zoom, point[1]* zoom + math.sin(angle)*length*0.2*zoom), 2)

        #     # text = self.font.render(f'{angle:.2f}', True, (255, 255, 200))
        #     # map_scaled.blit(text, (point[0]* zoom - 20, point[1]* zoom - 15))

        #     # text = self.font.render(f'{rel_angle:.2f}', True, (0, 0, 0))
        #     # map_scaled.blit(text, (point[0]* zoom - 20, point[1]* zoom + 15))


            # i+=1

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


        nextcheckpoint = self.checkpoints[self.nwp]
        pygame.draw.circle(map_scaled, (0,255,0), (self.dtrack[nextcheckpoint][0]* zoom, self.dtrack[nextcheckpoint][1]* zoom) ,5)
        return
        self.sim.reset()
        results = self.sim.run_sim()
        # print(f"len: {len(results)}")
        self.draw_counter = (self.draw_counter +1) % 5
        mode = self.draw_counter < 2
        positions = []
        for rs in results:
            # print(r)
            pos = rs["position"]
            t = rs["throttle"]
            s = rs["steer"]
            v = rs["velocity"]
            # positions.append(pos)

            val = t*2
            if mode:
                val = v / 450
            r = 255 * -min(max(val, -1.0), 0.0)
            g = 255 * min(max(val, 0.0), 1.0)
            b = 0#127 + min(max(t, -1.0), 1.0) * 127
            pygame.draw.circle(map_scaled, (r, g,b),   (pos[0]* zoom, pos[1]* zoom) ,2)

            # print(f"pos: {pos}")
        # pygame.draw.lines(map_scaled, (100, 0, 0), False, [zoom * p for p in positions], 2)



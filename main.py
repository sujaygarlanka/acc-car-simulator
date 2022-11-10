import pygame
import sys
import pymunk
from controller import Controller
from typing import NamedTuple
import time


class Observation(NamedTuple):
    velocity: float
    target_velocity: float
    distance_to_lead: float
    
class Car:
    def __init__(self, space, start_x):
        self.radius = 20
        self.space = space
        self.start_x = start_x
        self.start_y = 70
        self.body = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
        self.body.position = (self.start_x, self.start_y)
        self.shape = pymunk.Circle(self.body, self.radius)
        self.shape.mass = 1
        self.space.add(self.body, self.shape)

    def apply_force(self, force):
        self.body.apply_force_at_local_point((force, 0))

    def get_location(self):
        return (self.shape.body.position.x, self.shape.body.position.y)

class Simulator:
    def __init__(self, width, target_velocity, distance_threshold):
        self.width = width
        self.height = 200
        self.target_velocity = target_velocity
        self.distance_threshold = distance_threshold
        self.space = pymunk.Space()
        self.bg_location_x = 0
        self.ego_car = Car(self.space, 40)
        self.lead_car = Car(self.space, 400)
        self.lead_car_speed = 100
        self.lead_car.apply_force(self.lead_car_speed * 10)
        self.time_start = time.perf_counter()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.init()

    def _pygame_convert_coord(self, coord):
        return (int(coord[0]), int(self.height - coord[1]))
        
    def _draw_graphics(self):
        # (x,y) = self.pygame_convert_coord(self.ego_car.get_location())
        # print(x)
        # os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,0)
        black = (0, 0, 0)
        red = (255, 0, 0)
        self.screen.fill((217, 217, 217))
        metrics = self._get_metrics()
        font = pygame.font.Font('freesansbold.ttf', 20)

        bg_surface = pygame.image.load('background.jpg')
        bg_surface = pygame.transform.scale(bg_surface, (393.53, 200))
        # bg_surface_rect = bg_surface.get_rect(center = (self.width/2, self.height/2))
        self.screen.blit(bg_surface, (self.bg_location_x, 0))
        self.screen.blit(bg_surface, (self.bg_location_x + bg_surface.get_width(), 0))
        self.screen.blit(bg_surface, (self.bg_location_x + bg_surface.get_width()*2, 0))
        self.screen.blit(bg_surface, (self.bg_location_x + bg_surface.get_width()*3, 0))
        if abs(self.bg_location_x) > bg_surface.get_width():
            self.bg_location_x += bg_surface.get_width()

        velocity_text = "Velocity: " + str(round(metrics.velocity, 3)) + "/" + str(metrics.target_velocity)
        v_text = font.render(velocity_text, True, black if metrics.velocity <= metrics.target_velocity else red)
        v_rect = v_text.get_rect()
        self.screen.blit(v_text, v_rect)

        distance_text = "Distance: " + str(round(metrics.distance_to_lead, 3)) + "/" + str(self.distance_threshold)
        d_text = font.render(distance_text, True, black if metrics.distance_to_lead >= self.distance_threshold else red)
        d_rect = d_text.get_rect()
        d_rect.center = (650, 10)
        self.screen.blit(d_text, d_rect)

        ego_car_surface = pygame.image.load('ego_car.png')
        ego_car_surface = pygame.transform.scale(ego_car_surface, (69,40))
        ego_car_rect = ego_car_surface.get_rect(center = self._pygame_convert_coord(self.ego_car.get_location()))
        self.screen.blit(ego_car_surface, ego_car_rect)
        
        lead_car_surface = pygame.image.load('lead_car.png')
        lead_car_surface = pygame.transform.scale(lead_car_surface, (69,40))
        lead_car_rect = lead_car_surface.get_rect(center = self._pygame_convert_coord(self.lead_car.get_location()))
        self.screen.blit(lead_car_surface, lead_car_rect)
        
        pygame.display.update()

    def _step(self, acc):
        if acc > 0:
            self.ego_car.apply_force(acc*5)
        else:
            brake = 1 + .01 * acc
            pymunk.Body.update_velocity(self.ego_car.body, (0,0), brake, 0.1)
        if time.perf_counter() - self.time_start > 10:
            pymunk.Body.update_velocity(self.lead_car.body, (0,0), 0.99, 0.1)
        self.space.step(0.1)
        keep_on_screen = self.ego_car.get_location()[0] - self.ego_car.start_x
        self.ego_car.shape.body.position =  (self.ego_car.get_location()[0] - keep_on_screen, self.ego_car.start_y)
        self.lead_car.shape.body.position = (self.lead_car.get_location()[0] - keep_on_screen, self.lead_car.start_y)
        self.bg_location_x -= keep_on_screen
        
    def _get_metrics(self):
        distance = self.lead_car.get_location()[0] - self.ego_car.get_location()[0]
        ego_velocity = self.ego_car.body.velocity
        return Observation(ego_velocity[0], self.target_velocity, distance)

    def step(self, ego_acc):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        self._step(ego_acc)
        self._draw_graphics()
        return self._get_metrics()


width = 800
target_velocity = 100
distance_threshold = 500
simulator = Simulator(width, target_velocity, distance_threshold)
controller = Controller(target_velocity, distance_threshold)
force = 0
while True:
    obs = simulator.step(force)
    force = controller.run_step(obs)
    time.sleep(0.1)
        
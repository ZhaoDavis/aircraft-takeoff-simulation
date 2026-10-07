import pygame
import numpy as np
from scipy.integrate import ode
import math
import matplotlib.pyplot as plt
from matplotlib import ticker

WIDTH, HEIGHT = 1200, 600

WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (59, 130, 68)
LIGHTBLUE = (166, 232, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GRAY = (200, 200, 200)
LIGHTGRAY = (230, 230, 230)


cam_offset = 10
zoom = 0.25 #scale factor for simulation coords to screen coords (4 meters to 1 pixel)
ax_size = 20

takeoff_distance = -1
takeoff_velocity = -1
takeoff_angle = -1

class Aircraft:
    def __init__(self, name, mass, thrust, wingspan, wing_area, length):
        self.name = name
        self.mass = mass                #Kg
        self.thrust = thrust            #Newtons
        self.wingspan = wingspan        #m
        self.wing_area = wing_area      #m^2   
        self.length = length            #m
        self.aspect_ratio = self.wingspan**2/self.wing_area     
        self.drag_coeff = 0.02
        self.lift_coeff = 0.3
        self.oswald = 0.8

        self.in_air = False
        self.flaps_lowered = True
        self.gear_lowered = True
        self.aoa = 0.0                  #radians
        self.max_aoa = 0.52             #radians
        self.elevator_deflection = 0.0  #radians
        self.elevator_effectiveness = self.length*0.5  
    def toggle_flaps(self):
        self.flaps_lowered = not self.flaps_lowered

    def toggle_gear(self):
        self.gear_lowered = not self.gear_lowered

    def change_pitch(self, input):
        if input == 'up':
            self.elevator_deflection += 0.1
        elif input == 'down':
            self.elevator_deflection -= 0.1
        self.elevator_deflection = min(0.5, self.elevator_deflection)
        self.elevator_deflection = max(-0.3, self.elevator_deflection)

class AircraftSimulation:
    def __init__(self, aircraft):
        self.aircraft = aircraft
    
        self.air_density = 1.225    #kg/m^3
        self.g = 9.81               #m/s^2
        self.stats = []

        self.state = np.array([0.0, 0.0, 
                               0.0, 0.0, 0.0]) #(x, y, vx, vy, theta, omega)

        self.solver = ode(self.f) 
        self.solver.set_integrator('dop853')  
        self.solver.set_initial_value(self.state, 0) 
        self.time = 0.0 
        self.step = 0
        self.latest = ()

    def f(self, t, state):
        x, y, vx, vy, theta = state

        if vy == 0 and y == 0:
            flight_vector = 0
            self.aircraft.aoa = 0
        else:
            flight_vector = math.atan2(vy, vx)
            #flight_vector = (flight_vector + math.pi) % 2*math.pi - math.pi
            self.aircraft.aoa = theta - flight_vector

        v = math.sqrt(abs(vx**2) + abs(vy**2))

        cl = self.get_lift_coeff()
        #https://www.grc.nasa.gov/WWW/k-12/VirtualAero/BottleRocket/airplane/lifteq.html
        lift = cl * self.air_density * 0.5 * v**2 * self.aircraft.wing_area

        cd = self.get_drag_coeff(cl)
        #https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/
        drag = cd * 0.5  * self.air_density * v**2 * self.aircraft.wing_area

        if(y > 0):
            rolling_drag = 0
        else:
            rolling_drag = max(0, 0.03 * (self.aircraft.mass*self.g - lift))

        elevator_cl = 0.15*self.aircraft.elevator_deflection
        omega = elevator_cl * (2/(1+np.exp(-0.1*(v-50)))-0.03)
        omega += -self.aircraft.aoa**3 - 0.2 * self.aircraft.aoa     #pitch damping from AOA
        
        ax = (-lift*math.sin(flight_vector) + self.aircraft.thrust*math.cos(theta) - drag*math.cos(flight_vector) - rolling_drag) / self.aircraft.mass 
        ay = (lift*math.cos(flight_vector) + self.aircraft.thrust*math.sin(theta) - drag*math.sin(flight_vector)) / self.aircraft.mass - self.g 

        if vy < 0 and y <= 0:  # prevent plane from sinking through ground
            vy = 0
            ay = max(0, ay)

        self.latest = (self.time, x, y, v, lift, rolling_drag, math.degrees(theta), math.degrees(omega), math.degrees(self.aircraft.aoa))

        return [vx, vy, ax, ay, omega]  #[dx, dy, dvx, dvy, dtheta, domega]

    def update(self, dt):
        self.solver.integrate(self.time + dt)
        self.state = self.solver.y
        self.time += dt

        #record stats every 100 ticks(1 second in simulation)
        #recording inside f() results creates issues with determining when to record
        if self.step % 25 == 0:
            self.stats.append(self.latest)

        self.step += 1

    def get_state(self):
        return self.state

    def get_drag_coeff(self, cl):
        #https://www.grc.nasa.gov/WWW/k-12/VirtualAero/BottleRocket/airplane/induced.html
        cdi = cl**2/(math.pi * self.aircraft.oswald * self.aircraft.aspect_ratio) #Prandtl lifting line theory
        drag_coeff = self.aircraft.drag_coeff + cdi
        if self.aircraft.flaps_lowered:
            drag_coeff += 0.03
        if self.aircraft.gear_lowered:
            drag_coeff += 0.08
        return drag_coeff
    
    def get_lift_coeff(self):
        #"For thin airfoils at subsonic speed, the lift coefficient is 2 x pi x angle" from https://www.grc.nasa.gov/WWW/k-12/VirtualAero/BottleRocket/airplane/incline.html
        
        aoa = self.aircraft.aoa
        max_aoa = self.aircraft.max_aoa

        lift_coeff = self.aircraft.lift_coeff + 2*math.pi * aoa
        if self.aircraft.flaps_lowered:
            lift_coeff += 0.3

        if aoa > max_aoa:
            lift_coeff = (self.aircraft.lift_coeff + 2*math.pi * max_aoa) * (math.e**(-4*(abs(aoa-max_aoa))))
        
        return lift_coeff

def get_aircraft():
    #MTOW mass, thrust, wingspan, wing_area, length
    A320 = Aircraft("A320", 93000*0.8, 280000, 35.8, 122.4, 37.57)
    B777 = Aircraft("B777", 352000*0.8, 1000000, 64.8, 437, 73.86)
    A318 = Aircraft("A318", 68000*0.8, 200000, 34.1, 122.4, 31.44)
    CJ4 = Aircraft("CJ4", 7700*0.8, 32000, 15.5, 30.66, 16.26)
    F16 = Aircraft("F16", 19000*0.8, 130000, 9.96, 28, 15.06)
    aircraft_list = [A320, B777, A318, CJ4, F16]

    selected_aircraft = int(input("Select Aircraft Type:\n1. A320\n2. B777\n3. A318\n4. CJ4\n5. F16\n"))

    return(aircraft_list[selected_aircraft-1])

def draw_grid(screen):
    for x in range(0, WIDTH, ax_size):
        pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT))
    for y in range(-(cam_offset), HEIGHT, ax_size):
        pygame.draw.line(screen, GRAY, (0, y), (WIDTH, y))

def main():
    selected_aircraft = get_aircraft()
    simulation = AircraftSimulation(selected_aircraft)

    pygame.init()

    # Set the dimensions of the screen
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Aircraft Takeoff Simulation")

    running = True
    clock = pygame.time.Clock()

    plane = pygame.image.load("./assets/plane.png")
    plane = pygame.transform.scale(plane, (100, 40)) #plane increased to 10x size for visibility
    rotated_plane = plane

    while running:
        screen.fill(LIGHTGRAY)  #Background color

        simulation.update(0.01) 

        x, y, vx, vy, theta = simulation.get_state()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    simulation.aircraft.change_pitch('down')
                elif event.key == pygame.K_s:
                    simulation.aircraft.change_pitch('up')
                elif event.key == pygame.K_f:
                    simulation.aircraft.toggle_flaps()
                elif event.key == pygame.K_g and y > 0:
                    simulation.aircraft.toggle_gear()
                elif event.key == pygame.K_ESCAPE:
                    running = False
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w or event.key == pygame.K_s:
                    simulation.aircraft.change_pitch('release') 

                

        if vy > 0 and not simulation.aircraft.in_air:
            simulation.aircraft.in_air = True
            takeoff_distance = x
            takeoff_velocity = vx
            takeoff_angle = math.degrees(theta)

        draw_grid(screen) #draw ax
        pygame.draw.rect(screen, BLACK, pygame.Rect(0, HEIGHT - cam_offset, WIDTH, 5)) #draw runway

        aircraft_rect = pygame.Rect(x*zoom - 50, HEIGHT - y*zoom - 40 - cam_offset, 50, 30)  
        rotated_plane = pygame.transform.rotate(plane, math.degrees(theta))

        for i in range(len(simulation.stats) - 1):
            start = simulation.stats[i]
            end = simulation.stats[i + 1]
            pygame.draw.line(screen, BLUE, (start[1] * zoom, HEIGHT - start[2] * zoom - cam_offset - 10), (end[1] * zoom, HEIGHT - end[2] * zoom - cam_offset - 10), 2) #draw trail behind plane

        screen.blit(rotated_plane, aircraft_rect.topleft)

        font = pygame.font.SysFont("Arial", 24)
        velocity_text = font.render(f"v: {math.sqrt(vx**2 + vy**2):.2f} m/s", True, BLACK)
        position_text = font.render(f"Horizontal Displacement: {x:.2f} m", True, BLACK)
        altitude_text = font.render(f"Altitude: {y:.2f} m", True, BLACK)
        degree_text = font.render(f"Pitch: {math.degrees(theta):.2f}°", True, BLACK)
        aoa_text = font.render(f"Angle of Attack: {math.degrees(simulation.aircraft.aoa):.2f}°", True, BLACK)
        flaps_text = font.render(f"Flaps Lowered: {simulation.aircraft.flaps_lowered}", True, BLACK)
        gear_text = font.render(f"Gear Lowered: {simulation.aircraft.gear_lowered}", True, BLACK)
        elevator_text = font.render(f"Elevator Deflection: {simulation.aircraft.elevator_deflection:.2f}", True, BLACK)

        screen.blit(velocity_text, (10, 10))
        screen.blit(position_text, (10, 40))
        screen.blit(altitude_text, (10, 70))
        screen.blit(degree_text, (10, 100))
        screen.blit(aoa_text, (10, 130))
        screen.blit(flaps_text, (10, 160))
        screen.blit(gear_text, (10, 190))
        screen.blit(elevator_text, (10, 220))

        if simulation.aircraft.in_air:
            takeoff_dist_text = font.render(f"Takeoff Distance: {takeoff_distance:.2f} m", True, BLACK)
            takeoff_vel_text = font.render(f"Takeoff Speed: {takeoff_velocity:.2f} m/s", True, BLACK)
            takeoff_angle_text = font.render(f"Takeoff Pitch: {takeoff_angle:.2f}°", True, BLACK)

            screen.blit(takeoff_dist_text, (800, 10))
            screen.blit(takeoff_vel_text, (800, 40))
            screen.blit(takeoff_angle_text, (800, 70))

        pygame.display.flip()  

        if y*zoom >= HEIGHT or x*zoom >= WIDTH:
            pygame.quit()
            running = False
            break

        clock.tick(60)  

    pygame.quit()
    running = False

    #self.time, x, y, self.aircraft.aoa, theta, lift, rolling_drag
    np_arr = np.array(simulation.stats)
    print(np_arr.shape)
    fig, ax = plt.subplots(6, 1, figsize=(10, 10))
    plt.subplots_adjust(hspace=1.5)
    ax = ax.ravel()
    titles = ["Speed vs Time", "Lift vs Time", "Rolling drag vs Time", "Pitch vs Time","Omega vs Time", "AOA vs Time"]
    ylabs = ["Speed (m/s)", "Lift (Newtons)", "Rolling Drag (Newtons)", "Pitch (degrees)", "Angular Velocity (degrees/s)", "Angle of Attack (degrees)"]
    for i in range(0, 6):
        ax[i].plot(np_arr[:,0], np_arr[:,i+3])
        ax[i].set_title(titles[i])
        ax[i].set_xlabel("Time (Seconds)")
        ax[i].set_ylabel(ylabs[i])
        ax[i].margins(x=0)
        ax[i].grid()
        ax[i].yaxis.set_major_locator(ticker.MaxNLocator(4))
    fig.suptitle(f"Takeoff Metrics for {simulation.aircraft.name}")
    fig.canvas.manager.set_window_title("Takeoff Metrics")
    plt.show()

if __name__ == '__main__':
    main()
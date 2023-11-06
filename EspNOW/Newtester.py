import pygame
import sys
from collections import deque

# Initialize pygame and font
pygame.init()
pygame.font.init()

# Set up display
WIDTH, HEIGHT = 800, 600
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Bouncing Ball with Graphs")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

# Ball setup
BALL_RADIUS = 20
ball_x = WIDTH // 4
ball_y = HEIGHT // 2

# Physics
gravity = 9.8  # Increased gravity for a more noticeable effect
bounce = -0.6  # Adjust bounce effect, you might need to tweak this value further
velocity = 0
accelerations = deque(maxlen=WIDTH//2)  # store recent accelerations
velocities = deque(maxlen=WIDTH//2)  # store recent velocities


# Time
clock = pygame.time.Clock()
time_interval = 0

# Font
font_big = pygame.font.Font(None, 72)
font_small = pygame.font.Font(None, 36)
# Clear initial data
accelerations.clear()
velocities.clear()
def draw_graphs():
    """Draw the acceleration and velocity graphs"""
    origin_x = WIDTH // 2
    origin_y = HEIGHT // 2  # Change origin to the middle for both graphs

    # Draw acceleration graph
    if len(accelerations) > 1:  # Ensure there are at least two points
        acceleration_points = [(origin_x + idx, origin_y - int(a*10)) for idx, a in enumerate(accelerations)]
        pygame.draw.lines(win, BLUE, False, acceleration_points, 2)

    # Draw velocity graph
    if len(velocities) > 1:  # Ensure there are at least two points
        velocity_points = [(origin_x + idx, origin_y - int(v)) for idx, v in enumerate(velocities)]
        pygame.draw.lines(win, GREEN, False, velocity_points, 2)

    # Draw labels
    label_acc = font_small.render("Acceleration (Blue)", True, BLUE)
    label_vel = font_small.render("Velocity (Green)", True, GREEN)
    win.blit(label_acc, (WIDTH - 200, 20))
    win.blit(label_vel, (WIDTH - 200, 50))
# New variable for the kick strength
kick_strength = 2  # Change this value to control the strength of the kick

# Main loop
running = True
while running:
    win.fill(WHITE)
    time_interval = clock.tick(60) / 1000.0  # update time interval, in seconds

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:  # scroll up, kick the ball upwards
                velocity -= kick_strength
            elif event.button == 5:  # scroll down, kick the ball downwards
                velocity += kick_strength


   # Physics calculations
    acceleration = gravity - (velocity)# Acceleration is affected by gravity and air resistance
    initial_velocity = velocity  # Store the initial velocity before updating
    velocity += acceleration * time_interval  # Update the velocity
    ball_y += velocity  # Update position based on velocity and time

    # Boundary checking for ball
    if ball_y >= HEIGHT - BALL_RADIUS:
        ball_y = HEIGHT - BALL_RADIUS
        velocity *= bounce
    elif ball_y <= BALL_RADIUS:
        ball_y = BALL_RADIUS
        velocity *= bounce

    # Update acceleration and velocity history
    accelerations.append(acceleration)
    velocities.append(velocity)

    # Drawing
    pygame.draw.circle(win, RED, (ball_x, int(ball_y)), BALL_RADIUS)
    draw_graphs()

    # Render the text with updated velocity, acceleration, and time_interval
    text = f"V = {initial_velocity:.2f} + {acceleration:.2f} * {time_interval:.2f} => {velocity:.2f}"
    text_surface = font_big.render(text, True, BLACK)
    win.blit(text_surface, (20,20))

    # Update the display
    pygame.display.flip()

# Clean up
pygame.font.quit()
pygame.quit()
sys.exit()

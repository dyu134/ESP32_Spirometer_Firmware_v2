import sys
import time
import threading
from typing import Optional

import pygame

from SpirometerAPI import Spirometer


"""
Example Game using ESP32 Spirometer (x-axis) as input

- Uses SpirometerAPI to stream BLE data and read the latest calibrated X value (cal_x)
- Maps cal_x to a "flap" action for a simple Flappy-Bird-like demo
- Keyboard SPACE also flaps (useful for testing without hardware)

Controls
- Breath/tilt to change x-axis magnetic field (cal_x). If |cal_x| > threshold, the bird flaps.
- SPACE to flap (fallback)
- ESC or window close to quit (device disconnects automatically)

Requirements
- pip install pygame bleak pandas
- Ensure your device is advertising with the configured UUIDs in SpirometerAPI.py
- Check device ID using e.g. nRF Connect app and change if necessary
"""

# ------------------------ BLE Client Setup ------------------------
api = Spirometer(
    device_id="9C:13:9E:9D:20:C1",
    auto_calibrate_on_connect=True,
    calibration_samples=100,
    stability_std_threshold=30.0,      # <-- Increased threshold
    stability_min_samples=10,          # <-- Fewer samples required
)

connect_state = {"state": "connecting", "error": None}


def _connect_worker():
    try:
        # Connect and start streaming (blocking until optional auto-calibration completes)
        api.connect()
        connect_state["state"] = "connected"
    except Exception as e:
        connect_state["state"] = "error"
        connect_state["error"] = str(e)


threading.Thread(target=_connect_worker, daemon=True).start()


# ------------------------ Game Setup ------------------------
pygame.init()
WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Spirometer Example Game (X-axis)")
clock = pygame.time.Clock()
FONT = pygame.font.SysFont("Arial", 24)

# Game world params
GRAVITY = 0.7
FLAP_VELOCITY = -12
PIPE_GAP = 160
PIPE_WIDTH = 70
PIPE_SPEED = 4
SPAWN_TIME = 1400  # ms

# Input mapping params
X_DEADBAND = 6.0     # ignore small noise around zero
FLAP_THRESHOLD = 10.0  # when |cal_x| exceeds this, trigger a flap
LPF_ALPHA = 0.25     # smoothing factor for cal_x


def text(surface, s, x, y, color=(255, 255, 255)):
    surface.blit(FONT.render(s, True, color), (x, y))


class Bird:
    def __init__(self):
        self.x = 180
        self.y = HEIGHT // 2
        self.vy = 0.0
        self.r = 18

    def update(self):
        self.vy += GRAVITY
        self.y += self.vy

    def flap(self):
        self.vy = FLAP_VELOCITY

    def draw(self, surf):
        pygame.draw.circle(surf, (255, 200, 0), (int(self.x), int(self.y)), self.r)

    def reset(self):
        self.y = HEIGHT // 2
        self.vy = 0.0


class PipePair:
    def __init__(self, x: int):
        import random
        gap_y = random.randint(140, HEIGHT - 140)
        self.x = x
        self.top_rect = pygame.Rect(self.x, 0, PIPE_WIDTH, gap_y - PIPE_GAP // 2)
        self.bottom_rect = pygame.Rect(self.x, gap_y + PIPE_GAP // 2, PIPE_WIDTH, HEIGHT - (gap_y + PIPE_GAP // 2))
        self.passed = False

    def update(self):
        self.x -= PIPE_SPEED
        self.top_rect.x = int(self.x)
        self.bottom_rect.x = int(self.x)

    def offscreen(self) -> bool:
        return self.x + PIPE_WIDTH < 0

    def draw(self, surf):
        pygame.draw.rect(surf, (100, 200, 100), self.top_rect)
        pygame.draw.rect(surf, (100, 200, 100), self.bottom_rect)

    def collides(self, bx: int, by: int, br: int) -> bool:
        return self.top_rect.collidepoint(bx, by) or self.bottom_rect.collidepoint(bx, by)


# State
bird = Bird()
pipes: list[PipePair] = []
score = 0

SPAWN_EVENT = pygame.USEREVENT + 1
pygame.time.set_timer(SPAWN_EVENT, SPAWN_TIME)

filtered_x = 0.0

battery_percent: Optional[int] = None
last_battery_read = 0.0
BATTERY_INTERVAL = 15.0  # seconds


def should_flap_from_x(raw_x: float) -> bool:
    # Simple deadband + threshold detection on x-axis
    if abs(raw_x) < X_DEADBAND:
        return False
    return abs(raw_x) >= FLAP_THRESHOLD


running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_SPACE:
                bird.flap()
        elif event.type == SPAWN_EVENT and connect_state["state"] == "connected":
            pipes.append(PipePair(WIDTH + 40))

    # Read latest calibrated x-axis
    raw_x = api.get_latest_calibrated_x(0.0)
    # Low-pass filter
    filtered_x = (1.0 - LPF_ALPHA) * filtered_x + LPF_ALPHA * raw_x

    # Trigger flap when x magnitude is large
    if should_flap_from_x(filtered_x):
        bird.flap()

    # Update game world
    if connect_state["state"] == "connected":
        bird.update()
        for p in list(pipes):
            p.update()
            if p.offscreen():
                pipes.remove(p)
            # Score when passed
            if not p.passed and p.x + PIPE_WIDTH < bird.x:
                p.passed = True
                score += 1

    # Collision & bounds
    if bird.y - bird.r < 0 or bird.y + bird.r > HEIGHT:
        bird.reset()
        pipes.clear()
        score = 0

    else:
        for p in pipes:
            if p.collides(int(bird.x), int(bird.y), bird.r):
                bird.reset()
                pipes.clear()
                score = 0
                break

    # Battery polling (non-blocking feel; API handles async)
    now = time.time()
    if connect_state["state"] == "connected" and (now - last_battery_read) > BATTERY_INTERVAL:
        try:
            battery_percent = api.get_battery_level()
        except Exception:
            battery_percent = None
        last_battery_read = now

    # Draw
    screen.fill((30, 30, 30))

    # Ground line
    pygame.draw.line(screen, (80, 80, 80), (0, HEIGHT - 60), (WIDTH, HEIGHT - 60), 2)

    # Pipes
    for p in pipes:
        p.draw(screen)

    # Bird
    bird.draw(screen)

    # HUD
    if connect_state["state"] == "connecting":
        text(screen, "Connecting & calibrating...", 20, 20)
    elif connect_state["state"] == "error":
        text(screen, f"BLE error: {connect_state['error']}", 20, 20, (255, 100, 100))
        text(screen, "Press SPACE to play without device.", 20, 50, (255, 180, 180))
    else:
        text(screen, f"Score: {score}", 20, 20)
        text(screen, f"cal_x: {filtered_x:6.1f}", 20, 50)
        if battery_percent is not None:
            text(screen, f"Battery: {battery_percent}%", 20, 80)

    pygame.display.flip()
    clock.tick(60)

# Cleanup
try:
    api.disconnect()
except Exception:
    pass

pygame.quit()
sys.exit()
"""Same task as game.py but driven by an agent that chases the droplet.

Toggle HEADLESS to switch between:
  - HEADLESS = False -> py5 sketch draws the game; agent runs via Agent.act() callback
  - HEADLESS = True  -> no visuals, no display required; agent drives via nk.step()
                       with a deterministic virtual clock

Same Game state and chase policy in both modes.
"""
import sys
HEADLESS = '--headless' in sys.argv

from pathlib import Path
import random
from neurokraken import Neurokraken, State
from neurokraken.configurators import Display, devices

serial_in  = {'LR': devices.analog_read(pin=14, keys=['left', 'right']),
              'UD': devices.analog_read(pin=15, keys=['down', 'up'])}

serial_out = {'reward': devices.timed_on(pin=40)}

#---------------------------------- AGENT ----------------------------------

def chase_action():
    """Move the joystick toward the current reward position. Returns LR/UD analog values
    (0..1023, with 512 as neutral) shaped like what analog_read would deliver."""
    game = get.blocks['block']['game']
    dx = game.reward_pos[0] - game.x
    dy = game.reward_pos[1] - game.y
    norm = max(abs(dx), abs(dy), 1)
    LR = int(512 + 512 * (dx / norm))
    UD = int(512 - 512 * (dy / norm))      # flip y because game inverts UD
    LR = max(0, min(1023, LR))
    UD = max(0, min(1023, UD))
    return {'LR': LR, 'UD': UD}

class Agent:
    """Used in visual (non-headless) mode where the main loop runs as a py5 sketch
    and we can't call nk.step() ourselves. Agent.act() fires from inside the
    Dummy_Networker every 1/act_freq seconds."""
    def __init__(self):
        self.act_freq = 60        # 60 Hz updates

    def act(self):
        for k, v in chase_action().items():
            get.serial_in[k]['value'] = v

#---------------------------------- NEUROKRAKEN INSTANCE ----------------------

if HEADLESS:
    nk = Neurokraken(serial_in, serial_out, mode='agent',
                     log_dir=None,
                     headless=True,
                     task_tick_hz=200,        # virtual clock: 5 ms per tick
                     action_hold_steps=4)     # each nk.step() = 20 ms simulated
else:
    nk = Neurokraken(serial_in, serial_out, mode='agent', agent=Agent(),
                     display=Display(size=(800, 600)))

#----------------------------------- TASK -----------------------------------

from neurokraken.controls import get
from neurokraken.tools import Millis

spawn_points = [[100, 100], [700, 100], [100, 500], [700, 500]]

def constrain(value, minimum, maximum):
    return min(max(value, minimum), maximum)

def distance(x0, y0, x1, y1):
    return ((x0 - x1) ** 2 + (y0 - y1) ** 2) ** 0.5

class Game(State):
    def pre_task(self, sketch):
        # headless mode passes sketch=None - skip texture loading
        if sketch is None:
            return
        assets_path = Path(__file__).parent / 'assets'
        self.text_droplet = sketch.load_image(str(assets_path / 'game_droplet.png'))
        self.text_world   = sketch.load_image(str(assets_path / 'game_world.png'))
        self.text_heroAr  = sketch.load_image(str(assets_path / 'game_heroAr.png'))
        self.text_HeroBr  = sketch.load_image(str(assets_path / 'game_heroBr.png'))

    def on_start(self):
        self.x = 400
        self.y = 300
        self.speed = 10
        self.delta_x = 0
        self.delta_y = 0
        self.reward_pos = spawn_points[0]
        # per-episode timer: created/zeroed at on_start so nk.reset() doesn't strand it
        # holding a stale t_start from the previous episode (which would silently disable
        # the 60 fps gate when the virtual clock rewinds to 0).
        self.timer = Millis()

    def loop_main(self):
        # gate game logic to ~60 fps. In headless mode get.time_ms is the virtual clock,
        # so this still works (one game frame every ~16 ms simulated).
        if self.timer() < 16:
            return False, 0
        self.timer.zero()

        self.delta_x = (get.read_in('LR') / 512) - 1.0
        self.delta_y = (get.read_in('UD') / 512) - 1.0

        self.delta_x *= self.speed
        self.delta_y *= self.speed * -1
        self.x += self.delta_x
        self.y += self.delta_y
        self.x = constrain(self.x, 0, 800)
        self.y = constrain(self.y, 0, 600)

        if distance(self.x, self.y, self.reward_pos[0], self.reward_pos[1]) < 130:
            get.send_out('reward', 100)
            spawn_options = [s for s in spawn_points if distance(self.x, self.y, s[0], s[1]) > 250]
            self.reward_pos = random.choice(spawn_options)

        return False, 0

    def loop_visual(self, sketch):
        sketch.background(0)
        sketch.image_mode(sketch.CORNERS)
        sketch.image(self.text_world, 0, 0, 800, 600)

        sketch.image_mode(sketch.CENTER)
        sketch.image(self.text_droplet, self.reward_pos[0], self.reward_pos[1], 200, 200)

        with sketch.push():
            sketch.translate(self.x, self.y)
            if self.delta_x < 0:
                sketch.scale(-1, 1)
            if get.time_ms % 1000 > 500:
                sketch.image(self.text_heroAr, 0, 0, 105, 126)
            else:
                sketch.image(self.text_HeroBr, 0, 0, 105, 126)

task = {'game': Game(next_state='game')}

nk.load_task(task)
nk.run()  # in headless mode this returns immediately

#----------------------------------- DRIVER ---------------------------------

if HEADLESS:
    def reward_count():
        return len(get.log['controls'].get('reward', []))

    EPISODES = 2
    STEPS_PER_EPISODE = 1500   # 1500 * 20ms = 30s simulated per episode

    for episode in range(EPISODES):
        nk.reset()
        rewards_at_start = reward_count()

        for step in range(STEPS_PER_EPISODE):
            obs, info = nk.step(chase_action())
            if info['quit']:
                break

        game = get.blocks['block']['game']
        rewards = reward_count() - rewards_at_start
        print(f'episode {episode}: {STEPS_PER_EPISODE} steps, t_ms={obs["_t_ms"]:.0f}, '
              f'final pos=({game.x:.0f}, {game.y:.0f}), rewards collected={rewards}')

    nk.close()
    print('done.')

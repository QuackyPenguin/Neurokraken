"""Headless RL training loop example.

Same task structure as agent_simple.py (touch left/right based on servo position),
but driven by nk.reset() / nk.step() instead of an Agent.act() callback.

Demonstrates:
- headless=True (no py5 visuals, no cameras, deterministic virtual clock)
- nk.step({'touch_left': value}) injects the action into serial_in
- nk.get_obs() returns vector + state/block names + t_ms
- nk.reset() between episodes restarts the state machine cleanly
- Reward is detected by diffing the reward_valve change-log (since timed_on
  has reset_after_send=True and decays before get_reward_dict can see it)
"""
import random
from neurokraken import Neurokraken, State
from neurokraken.configurators import devices

serial_in = {
    'touch_left':  devices.capacitive_touch(pins=[10, 11], keys=['a']),
    'touch_right': devices.capacitive_touch(pins=[29, 30], keys=['d']),
}

serial_out = {
    'servo':        devices.servo(pin=14),
    'reward_valve': devices.timed_on(pin=40),
}

nk = Neurokraken(
    serial_in=serial_in,
    serial_out=serial_out,
    mode='agent',
    log_dir=None,           # use a temp dir; cleaned up by nk.close()
    headless=True,
    task_tick_hz=200,       # virtual clock: 1 tick = 5 ms simulated
    action_hold_steps=4,    # each nk.step() advances 4 ticks (= 20 ms simulated)
)

#----------------------------------- TASK -----------------------------------

from neurokraken.controls import get

class Intertrial(State):
    def on_start(self):
        get.send_out('servo', 127)            # arm in middle position

class Choice(State):
    def on_start(self):
        self.servo_pos = random.choice([10, 245])
        get.send_out('servo', self.servo_pos)

    def loop_main(self):
        if self.servo_pos == 10 and get.read_in('touch_left') > 4000:
            get.send_out('reward_valve', 50)
            return True, 0
        elif self.servo_pos == 245 and get.read_in('touch_right') > 4000:
            get.send_out('reward_valve', 50)
            return True, 0
        return False, 0

task = {
    'wait':   Intertrial(max_time_s=3,  next_state='choice'),
    'choice': Choice    (max_time_s=15, next_state='wait'),
}

nk.load_task(task)
nk.run()                                       # returns immediately in headless mode

#----------------------------------- RL LOOP --------------------------------

def random_action():
    """Touch nothing, left, or right with equal probability."""
    choice = random.choice(['none', 'left', 'right'])
    return {
        'touch_left':  6000 if choice == 'left'  else 0,
        'touch_right': 6000 if choice == 'right' else 0,
    }

def reward_count():
    """How many times the reward valve has fired so far this episode."""
    return len(get.log['controls'].get('reward_valve', []))

EPISODES = 3
STEPS_PER_EPISODE = 2000

for episode in range(EPISODES):
    nk.reset()
    rewards_at_start = reward_count()
    last_state = None

    for step in range(STEPS_PER_EPISODE):
        _random_action = random_action()
        obs, info = nk.step(_random_action)

        # log on state transitions so we can see the task progressing
        state_name = obs['_state']
        # if state_name != last_state:
        if state_name == 'choice' or last_state == 'choice':  # log every step in choice state since it's more interesting
            print(f'  [ep {episode} step {step:3d}  t_ms={obs["_t_ms"]:6.0f}]  '
                  f'state={last_state!r:>11s} -> {state_name!r:<11s}  block={obs["_block"]!r}, action={'left' if _random_action["touch_left"] else "right" if _random_action["touch_right"] else "none":>5s}')
            last_state = state_name

        if info['quit']:
            break

    rewards_this_ep = reward_count() - rewards_at_start
    print(f'episode {episode}: {STEPS_PER_EPISODE} steps, '
          f'final t_ms={obs["_t_ms"]:.0f}, rewards delivered={rewards_this_ep}\n')

nk.close()
print('done.')

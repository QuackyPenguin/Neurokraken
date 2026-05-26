
import sys
from pathlib import Path
# # Add the root to the path for imported mode tests, so that modules can be found.
sys.path.insert(0, str(Path(__file__).parent.resolve()))

import os, shutil
from core.print0 import print0
from threading import Thread
import time
import importlib.util
import inspect
Sketch = None
# type hint imports
from typing import Callable, Container, Any
from core.state_machine import State

class _SerialReady(Exception):
    """Raised when serial dictionaries are ready for config2teensy"""
    pass

class _MockSketch:
    """Absorbs all sketch method calls in headless pre_task so states fully initialize without a display.
    Returns itself from attribute access and calls so chains like sketch.load_image(p).get_np_pixels()*scale
    don't raise AttributeError. __array__ lets numpy consume the mock (np.flip etc.) without crashing."""
    def __getattr__(self, name):
        return self
    def __call__(self, *args, **kwargs):
        return _MockSketch()
    def get_np_pixels(self):
        import numpy as _np
        return _np.zeros((1, 1, 4), dtype=_np.uint8)
    def __mul__(self, other): return _MockSketch()
    def __rmul__(self, other): return _MockSketch()
    def __add__(self, other): return _MockSketch()
    def __radd__(self, other): return _MockSketch()
    def __sub__(self, other): return _MockSketch()
    def __rsub__(self, other): return _MockSketch()
    def __truediv__(self, other): return _MockSketch()
    def __rtruediv__(self, other): return _MockSketch()

class Neurokraken:
    def __init__(self, serial_in:dict={}, serial_out:dict={}, log_dir:str|None='./', mode='teensy',
                 display:dict=None, cameras:list=(), microphones:list=(),
                 subject:dict|str={'ID': '_'}, serial_key:str='KRAKEN',
                 autostart=True, max_framerate=8_000, networker_mode='archivist', agent=None,
                 config:Container={}, task_path:Path=None, import_pre_run:str=None,
                 log_performance=False,
                 headless: bool=False,
                 sim_speed: float=1.0,
                 task_tick_hz: int=200,
                 render_hz: int=0,
                 action_hold_steps: int=1,
                 ):
        """Create a Neurokraken instance using the provided device configuration.
        This class manages communication with hardware components including serial
        interfaces, camera systems, and data logging. It handles task execution,
        real-time data streaming, and system configuration management.

        Args:
            serial_in (dict): Dictionary containing serial input configuration
            serial_out (dict): Dictionary containing serial output configuration
            log_dir (str|None, optional): Directory path for logging output. A log folder will be created at this location.
                                          Defaults to the current folder './'. None to not save a log.
            mode (str, optional): Operating mode ('teensy', 'keyboard' or 'agent'). Defaults to 'teensy'.
                                  In 'agent' mode there are two sub-flows depending on headless:
                                  with headless=False (visual) the main loop runs as a blocking py5 sketch and the
                                  agent object's act() method is called periodically from inside Dummy_Networker,
                                  writing actions into serial_in from within act().
                                  With headless=True (RL training) run() returns immediately and you drive the loop
                                  yourself via nk.step(action_dict); the agent argument is ignored.
            agent (class, optional): A class with a def act(self) method and an act_freq (Hz) attribute,
                                     called periodically from inside the main loop when mode='agent' and headless=False.
                                     Ignored in headless mode where actions are injected via nk.step() instead.
            display (dict, optional): A configurators.Display() to position the subject's task view among
                                             the computer's connected displays.
            cameras (list, optional): List of camera configurations using configurators.Camera()
            microphones (list, optional): List of microphone configurations using configurators.Microphone()
            subject (dict|str, optional): Subject identification information. Defaults to {"ID": "_"}
            serial_key (str, optional): Serial communication key identifier. Defaults to 'KRAKEN'
            autostart (bool, optional): Whether to automatically start the experiment or wait for get.start(). Defaults to True
            max_framerate (int, optional): Maximum frame rate for the main loop. Defaults to 8000
            log_performance (bool, optional): Set to True to have the main loop log iteration and networking times. Defautls to False.
            config (Container, optional): Useful in runner mode to develop config-dependent experiments.
                                          The provided container (i.e. config.py file) will be accessible as get.config
            task_path (Path, optional): Useful in runner mode, this folder (i.e. tasks/my_task) will be copied to the 
                                        log folder as a backup of the experiment run. This path will also be searched
                                        for a file launch.py to import and run before the experiment start.
            import_pre_run (str, optional): Useful in runner mode.
                                            Path to a .py file to import just before starting the run, i.e. to start a GUI.
            headless (bool, optional): Disable all visual output and cameras/microphones, and switch the Dummy_Networker
                                       to a deterministic virtual clock (1000/task_tick_hz ms per tick) instead of wall-clock.
                                       When True, run() returns immediately after setup so the caller can drive the loop
                                       via nk.step() and nk.reset() (the standard RL training entry points). Defaults to False.
            sim_speed (float, optional): Simulation speed multiplier (passed to Main, implementation pending). Defaults to 1.0.
            task_tick_hz (int, optional): Target tick rate for the task state machine. In headless mode this also sets
                                          the virtual clock step (1000/task_tick_hz ms per Main.draw()). Defaults to 200.
            render_hz (int, optional): Target render rate; 0 means no rendering (passed to Main, implementation pending). Defaults to 0.
            action_hold_steps (int, optional): Default number of state-machine ticks each headless nk.step() advances when
                                               n_ticks is not given. Unused in visual mode (the loop ticks freely). Defaults to 1.
        """
        self.running_config2teensy = False
        stack = inspect.stack()
        for frame_info in stack:
            if frame_info.filename.endswith('config2teensy.py'):
                self.running_config2teensy = True

        self.serial_in = serial_in
        self.serial_out = serial_out
        self.config = config
        self.display_config = display
        self.log_dir = log_dir
        self.max_framerate = max_framerate
        self.task_path = task_path
        self.import_pre_run = import_pre_run
        self.log_performance = log_performance
        
        self.mode = mode
        self.headless = headless
        self.sim_speed = sim_speed
        self.task_tick_hz = task_tick_hz
        self.render_hz = render_hz
        self.action_hold_steps = action_hold_steps
        
        if self.headless:
            cameras = ()
            microphones = ()

        #------------------------- CHECK CORE SERIAL ENTRIES -------------------------
        if not 't_ms' in self.serial_in.keys():
            self.serial_in = {'t_ms': {'value': 0, 'encoding': 'uint', 'byte_length': 4, 'logging': False},
                              **self.serial_in}
        
        if not 'start_stop' in self.serial_out.keys():
            self.serial_out = {'start_stop': {'value': 0, 'encoding': 'uint', 'byte_length': 1,
                               'default': 0, 'reset_after_send': True},
                               **self.serial_out}

        if self.log_performance:
            self.serial_in['t_ms']['logging'] = True

        if self.running_config2teensy:
            pass
            # raise _SerialReady

        #------------------------- LOGGING -------------------------

        from datetime import datetime

        # replace forbidden directory characters and trimm off milliseconds
        log_name_DT = ''.join(str(datetime.now()).replace(':', ';').replace(' ', '_').split('.')[:-1])
        if type(subject) == str:
            subject = {'ID': subject}
        log_name = (subject['ID']) + '_' + log_name_DT
        if subject['ID'] == '_':
            log_name = 'Neurokraken' + '_' + log_name_DT
        if self.log_dir is not None:
            self.log_dir = (Path(self.log_dir) / f'{log_name}')
        else:
            import tempfile
            self.tempdir = tempfile.TemporaryDirectory(prefix='neurokraken_tmp_')
            print(f'running without log saving. Only a temporary log will be maintained for the run duration in {self.tempdir.name}')
            self.log_dir = Path(self.tempdir.name)

        if not self.log_dir.exists():
            os.makedirs(self.log_dir)

        if task_path is not None:
            # save a backup of the current version of the task
            shutil.copytree(self.task_path, self.log_dir / 'task', ignore=shutil.ignore_patterns('__pycache__*'))

        #------------------------- PROCESS PRIORITY -------------------------

        import psutil
        if sys.platform == 'win32':
            p = psutil.Process(os.getpid())
            orig_priority = p.nice()
            p.nice(psutil.REALTIME_PRIORITY_CLASS)
            print0(f'changed process priority from {orig_priority} to {p.nice()}',
                    priority=3, color='blue', topic='configuration')
            # If python is not executed from an administrator console the priority 
            # can and will only be set to HIGH instead of REALTIME

        #------------------------- LOG -------------------------

        from collections import defaultdict
        self.log = defaultdict(list, {
                    'experiment_data': {'datetime': str(datetime.now()),
                                        **subject # content of subject dict or subject.json
                                        },
                    'events': [], # (time,str) entries
                    'trials': [],
                    'blocks': [],
                    'states': [],
                    'cameras (t_ms/#frame/vid_time)': {},
                    'microphones (t_ms/audio_time)': {},
                    'controls': {}, # serial_out changes
                    })

        #------------------------- RUN CONTROLS -------------------------
        from dataclasses import dataclass
        @dataclass
        class Run_Controls:
            """Controls for the task start and end.
            beginning (bool):
                Used by statemachine.start_state_machine(). Schedules the activation by the main loop
                as soon as the teensy has reset its clock.
                True if the task is starting. Starting state defined by autostart=True/False.
            active (bool):
                Can be read by elements like cameras or code to fit their activity.
            quitting (bool):
                True if a shutdown has been triggered
                - useful for shutting down self-developed parallel code"""
            
            beginning:bool = True
            active:bool = False
            quitting:bool = False

        self.run_controls = Run_Controls()

        #------------------------- NETWORKING -------------------------
        archivist_mode = True if networker_mode == 'archivist' else False
        if mode == 'keyboard' or mode == 'agent':
            archivist_mode = False

        from core import networker as netw       

        if mode=='keyboard':
            self.networker = netw.Dummy_Networker()
        elif mode =='agent':
            # Headless RL training drives a virtual clock instead of wall-clock so a tick is deterministic.
            virtual_time_step_ms = (1000.0 / self.task_tick_hz) if self.headless else None
            self.networker = netw.Dummy_Networker(mode='agent', agent=agent, virtual_time_step_ms=virtual_time_step_ms)
        else:
            try:
                self.networker = netw.Networker(serial_key=serial_key,
                                                archivist_mode=archivist_mode, serial_in_log=self.log,
                                                run_controls=self.run_controls)
            except Exception as e:
                print0('Unable to start teensy communication. Is the USB cable plugged in? ' +
                       'If you provided a COM port in config.py it may not be correct - Try removing it to use autodetection. ' +
                       'If you want to use keyboard controls, please set simulate_teensy = True', color='red')
                print(e)
                exit()

        #------------------------- STATE_MACHINE AND CONTROLS -------------------------

        from core import state_machine

        self.machine = state_machine.State_Machine(self.serial_in['t_ms'],
                                                   self.serial_out,
                                                   self.run_controls,
                                                   block_log=self.log['blocks'],
                                                   trial_log=self.log['trials'],
                                                   state_log=self.log['states'])

        if autostart == False:
            self.run_controls.beginning = False

        # sketch info for the UI
        self.threads_info = {'framerate_main': 0,
                             'framerate_visual': 0,
                             'framerate_cams': {}}

        #------------------------- CAMERAS -------------------------

        from core import cameras as kraken_cam, microphones as kraken_mic

        if not isinstance(cameras, (list, tuple)):
            cameras = [cameras]
        for cam in cameras:
            kraken_cam.cameras.append(kraken_cam.Cam_Sketch(cam, self.run_controls, self.log['cameras (t_ms/#frame/vid_time)'],
                                                            self.serial_in['t_ms'], log_dir=self.log_dir,
                                                            show_cv2_backends=False, threads_info=self.threads_info))
        [cam.run_sketch(block=False) for cam in kraken_cam.cameras]

        if not isinstance(microphones, (list, tuple)):
            microphones = [microphones]
        self.microphones = []
        for mic in microphones:
            self.microphones.append(kraken_mic.Microphone(mic, self.run_controls, self.log['microphones (t_ms/audio_time)'],
                                                           self.serial_in['t_ms'], log_dir=self.log_dir))
        [mic.run_sketch(block=False) for mic in self.microphones]

        # loading general configuration elements is now complete

        #------------------------- POPULATE GET CONTROLS FOR THE USER -------------------------

        from . import controls

        get = controls.Get(serial_in=self.serial_in, serial_out=self.serial_out, config=config,
                           state_machine=self.machine, log=self.log,
                           cameras=kraken_cam.cameras, camera=kraken_cam.get_camera,
                           threads_info=self.threads_info, log_dir=self.log_dir, mode=mode)
        
        # replace the content of get
        controls.get.__dict__.update(get.__dict__)

    def load_task(self, task:dict[str, State] | dict[str, dict[str, State]],
                  experiment:Container={}, start_block:str|None=None, permanent_states:list[Callable]=(),
                  run_at_start:Callable=lambda : None, run_at_quit:Callable=lambda : None,
                  run_post_trial:Callable=lambda : None,
                  run_at_visual_start:Callable[[Any], None]=lambda sketch : None,
                  main_as_sketch:bool=True):
        
        from . import controls

        # make the experiment accessible for the later loaded UI
        # (reimporting experiment.py as is would reexecute code like adding duplicate achievements)
        controls.get.experiment = experiment

        if isinstance(list(task.values())[0], dict):
            blocks = task
        else:
            # the task is just a dict/progression of states, add a minimal block around it
            blocks = {'block': task}

        # there hasn't been a communication yet, so the original t_ms of a block/trial/state will still be 0
        self.start_block=start_block
        self.machine.define_experiment(blocks, start_block=start_block)

        controls.get.permanent_states = permanent_states

        self.main_as_sketch = main_as_sketch

        #------------------------- MAIN LOOP -------------------------

        from core import main_loops
        
        main_loops.main = main_loops.Main(self.networker, self.serial_in, self.serial_out, 
                                          self.run_controls, self.log, self.log_dir, self.machine,
                                          max_framerate=self.max_framerate, permanent_states=permanent_states,
                                          threads_info=self.threads_info, 
                                          run_at_start=run_at_start, run_at_quit=run_at_quit, run_post_trial=run_post_trial,
                                          log_performance=self.log_performance,
                                          mode=self.mode, sim_speed=self.sim_speed, task_tick_hz=self.task_tick_hz,
                                          render_hz=self.render_hz, action_hold_steps=self.action_hold_steps,
        )
        
        #------------------------- TASK DISPLAY -------------------------

        if self.display_config is not None and not self.headless:
            main_loops.visual = main_loops.Visual(self.machine, display_config=self.display_config, 
                                                  run_controls=self.run_controls, threads_info=self.threads_info,
                                                  run_at_visual_start=run_at_visual_start)
        #------------------------- LOAD ASSETS -------------------------

        if not self.headless:
            from py5 import Sketch as _Sketch
            class Pre_Task(_Sketch):
                """This sketch merely acts as a py5 instance to run py5 depending code, i.e. loading textures
                before the main- and display loops that may depend on this data being loaded."""
                def __init__(self, blocks):
                    super().__init__()
                    self.blocks = blocks

                def settings(self):
                    self.size(120, 120, self.P3D)

                def setup(self):
                    # P3D doesn't work with .get_surface().set_visible(False) so the window will flicker up for a short moment during this step
                    # self.world = self.load_shape(r'C:\Users\q131aw\Desktop\temp\test3d\otherFolder\world.obj')
                    for block in self.blocks.values():
                        for state in block.values():
                            state.pre_task(self)

                def draw(self):
                    # setup and the included load_shape seems to behave slightly asynchronous
                    # - once the sketch ran for several frames assets should be loaded.
                    if self.frame_count == 5:
                        self.exit_sketch()

            pre_task = Pre_Task(self.machine.blocks)
            pre_task.run_sketch(block=True)

        else:
            # headless: use a mock sketch so states can call sketch methods without a display
            mock = _MockSketch()
            print('Running in headless mode: using a mock sketch to load assets without a display. If you see AttributeErrors related to the sketch, consider adding the relevant method to the _MockSketch class.', flush=True)
            for block in self.machine.blocks.values():
                for state in block.values():
                    state.pre_task(mock)

        #------------------------- GARBAGE COLLECTION -------------------------

        import platform
        if platform.system() == 'Windows' and (self.mode != 'agent') and (not self.headless):
            import gc

            gc_level = 0
            if hasattr(experiment, 'garbage_collect_level'):
                gc_level = experiment.garbage_collect_level

            if gc_level == 2:
                print0(f'using default python garbage collection - you may expect lag spikes in the main loop',
                    color='blue', topic='configuration', priority=3)
            elif gc_level == 1:
                # reduced garbage collection - only moderate impact on main loop
                def reduced_garbage_collect():
                    print0(print0(f'freezing {gc.get_count()} current objects to permanent memory', 
                        color='blue', topic='configuration', priority=3))
                    gc.freeze()
                    gc.disable()
                    while True:
                        gc.collect(0)
                        time.sleep(10.0)

                Thread(target=reduced_garbage_collect, daemon=True).start()
            else:
                print0('disabling garbage collection for highest performance. You can expect an additional increase of several' +
                    'hundred MB RAM/hour runtime. (generally this is bellow the impact of already existing data collection)',
                    color='blue', topic='configuration', priority=3)
                gc.disable()

        #------------------------- STARTSTOP KEY LISTENER -------------------------

        if platform.system() == 'Windows':
            import keyboard
            def keyboard_startstop(event):
                if keyboard.is_pressed('alt') and keyboard.is_pressed('ctrl'):
                    if event.name== 'q':
                        self.machine.quit()
                    elif event.name == 's':
                        self.machine.start_state_machine()
                    elif event.name == 'e':
                        self.machine.stop_state_machine()
            keyboard.on_release(keyboard_startstop)

    def run(self):
        """Start the experiment.

        In visual mode (the default) this blocks on the py5 main sketch loop until the experiment quits
        (Ctrl+Alt+Q or get.quit()). In headless mode it initializes the main loop and returns immediately
        so the caller can drive the state machine via nk.step() and nk.reset().
        """
        if self.running_config2teensy:
            return

        print0('Starting Neurokraken. Ctrl+Alt+Q to quit', color='cyan')
        if not self.run_controls.beginning:
            print('autostart was provided as False. Press Ctrl+ALT+S or call get.start() from python to start, (CTRL+ALT+E/get.stop() to end the run)')

        from core import main_loops
        #------------------------- VISUAL LOOP -------------------------

        if self.display_config is not None and not self.headless:
            main_loops.visual.run_sketch(block=False)
            # safety for the borderless window starting out at a higher size
            while not main_loops.visual.frame_count > 1:
                time.sleep(0.001)

        #------------------------- UI -------------------------
        # load the UI after the visuals in case they are to be copied/shown in the UI
        if self.import_pre_run is not None:
            spec = importlib.util.spec_from_file_location(name='', location=self.import_pre_run)
            module = importlib.util.module_from_spec(spec)
            # Execute the module to load it
            spec.loader.exec_module(module)

        #------------------------- MAIN LOOP -------------------------

        if self.headless:
            # headless/agent mode: initialize without starting a sketch; caller drives the loop via step()
            main_loops.main._init_state()
            return

        if self.main_as_sketch:
            # more priority/consistency amidst parallel processes like camera capturing
            main_loops.main.run_sketch(block=True)
        else:
            # faster but less consistent frame intervals amidst parallel processes
            self.networker.write_teensy_data(self.serial_out)
            while main_loops.main.running:
                main_loops.main.draw()

        self.close()

    def close(self):
        """Release temporary resources. Call this after the training loop ends when using headless mode."""
        if hasattr(self, 'tempdir'):
            while True:
                try:
                    self.tempdir.cleanup()
                    break
                except PermissionError:
                    # some process (likely video saving) is still utilizing the temp dir preventing deletion - try again later
                    time.sleep(0.1)
                    
                    
    # NEW: agent mode methods
    def get_obs(self, keys: list[str] | None = None) -> dict:
        """Observation for agents/RL. Returns the current serial_in values plus task context.

        Primarily used by the headless RL loop where the caller pulls an observation after each nk.step(),
        but also works in visual mode (e.g. inside Agent.act() if you want a single dict view of the inputs).

        Args:
            keys (list[str], optional): Subset of serial_in keys to include in the vector. Defaults to every
                                        serial_in entry except 't_ms'.

        Returns:
            dict with keys 'vector' (list of current values), 'keys' (matching key order),
            '_state' (current state name str), '_block' (current block name str), '_t_ms' (current sim time).
        """
        if keys is None:
            keys = [k for k in self.serial_in.keys() if k != 't_ms']
        obs_vec = [self.serial_in[k].get('value') for k in keys]
        current_state = self.machine.current_state
        return {
            'vector': obs_vec,
            'keys': keys,
            '_state': current_state.name if current_state is not None else None,
            '_block': self.machine.current_block,
            '_t_ms': self.serial_in['t_ms']['value'],
        }

    def get_reward_dict(self, include_input: bool = False) -> dict:
        """Get a dictionary of output values that would be considered a reward by the current task.
        Optionally include input values as well. This can be used as an interface to define a reward function
        for RL agents based on the task itself.

        Caveat: devices with reset_after_send=True (timed_on, tone, ...) have their value zeroed inside
        write_teensy_data during the same tick they fire. To detect such reward events reliably, diff the
        per-output change history in get.log['controls'][key] instead of relying on this dict's live value.

        Args:
            include_input (bool, optional): Also include current serial_in values. Defaults to False.

        Returns:
            dict of {serial_out key: current value} plus '_state' (current state name) and '_block' (current block name).
        """
        reward_dict = {k: v.get('value') for k, v in self.serial_out.items()}
        if include_input:
            reward_dict.update({k: v.get('value') for k, v in self.serial_in.items()})
        # add task context
        current_state = self.machine.current_state
        reward_dict['_state'] = current_state.name if current_state is not None else None
        reward_dict['_block'] = self.machine.current_block
        return reward_dict

    def reset(self, clear_log: bool = True, reset_io: bool = True):
        """Reset the experiment for the next RL episode.

        Intended for headless mode where you drive episodes from a Python loop. In visual mode the main loop
        runs continuously and the state machine just keeps looping through blocks/states; there's no concept
        of an "episode" to reset between, so calling this from a visual run is rarely useful.

        Args:
            clear_log (bool): Clear events, trials, states, blocks, and controls from the log. Defaults to True.
            reset_io (bool): Reset serial_out values to their defaults and t_ms to 0. Defaults to True.
        """
        if reset_io:
            # reset outputs to defaults if present, otherwise to 0
            for k, v in self.serial_out.items():
                if 'default' in v:
                    v['value'] = v['default']
                else:
                    v['value'] = 0
            # only reset clock here, others depend on the task and will be reset in the machine reset
            self.serial_in['t_ms']['value'] = 0
            
        
        if clear_log:
            # Clear framework keys
            for k in ['events', 'trials', 'states', 'blocks']:
                self.log[k].clear()
            self.log['controls'].clear()
            # Also clear any task-specific list keys added by task.py (e.g. collected, spawns,
            # rewards_t[ms]/x/y, etc.) so per-episode counts don't accumulate across resets.
            # We skip non-list/non-dict keys and the experiment_data identity dict.
            _skip = {'experiment_data', 'events', 'trials', 'states', 'blocks', 'controls',
                     'cameras (t_ms/#frame/vid_time)', 'microphones (t_ms/audio_time)'}
            for k, v in list(self.log.items()):
                if k not in _skip and isinstance(v, list):
                    v.clear()
            # Main.draw() expects log['controls'][key] to exist for every serial_out entry.
            # _init_state repopulates it (and also rebuilds Main's internal serialout_key_lastval_updated
            # mirror so it reflects the post-reset default values).
            from core import main_loops
            main_loops.main._init_state()

        self.machine.stop_state_machine()
        # In headless mode, re-run pre_task for all states so they rebuild any world/effect
        # objects that were created during setup (e.g. task.Game.self.worlds).  pre_task is
        # designed as a one-shot setup run by load_task; without this, episode 2+ reuses the
        # depleted world from episode 1 and nothing gets spawned or collected.
        if self.headless:
            mock = _MockSketch()
            for block in self.machine.blocks.values():
                for state in block.values():
                    state.pre_task(mock)
        self.machine.reset()
        self.machine.start_state_machine()

    def step(self, action: dict, n_ticks: int | None = None):
        """Advance the task by n_ticks and return the resulting observation.

        Headless mode only - in visual mode the main loop is a blocking py5 sketch driven by run(), and the
        agent injects actions via its Agent.act() callback instead of calling step() directly. Must be called
        only after load_task() and run() (which returns immediately in headless mode).

        Args:
            action (dict): Keys matching serial_in entries to inject as the agent's action this tick.
                           The value persists across all n_ticks ticks (Dummy_Networker's virtual-clock branch
                           does not overwrite serial_in values between ticks).
            n_ticks (int, optional): Number of main-loop ticks to advance. Defaults to self.action_hold_steps.

        Returns:
            tuple: (obs dict from get_obs(), info dict with t_ms and quit flag)
        """
        if n_ticks is None:
            n_ticks = self.action_hold_steps
        # apply action by writing into serial_in values that are normally provided by Dummy_Networker
        for k, val in action.items():
            if k in self.serial_in:
                self.serial_in[k]['value'] = val

        from core import main_loops
        for _ in range(n_ticks):
            main_loops.main.draw()
            
        obs = self.get_obs()
        info = {
            "t_ms": self.serial_in['t_ms']['value'],
            "quit": self.run_controls.quitting,
        }
        return obs, info
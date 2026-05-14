
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
            mode (str, optional): Operating mode ('teensy', 'keyboard' or 'agent') Defaults to 'teensy'
            agent (class, optional): A class with a def act() method to run when mode='agent'
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
            TODO: write the description for the new parameters headless, sim_speed, task_tick_hz, render_hz, action_hold_steps
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
            os.mkdir(self.log_dir)

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

        self.log = {'experiment_data': {'datetime': str(datetime.now()),
                                        **subject # content of subject dict or subject.json
                                        },
                    'events': [], # (time,str) entries
                    'trials': [],
                    'blocks': [],
                    'states': [],
                    'cameras (t_ms/#frame/vid_time)': {},
                    'microphones (t_ms/audio_time)': {},
                    'controls': {}, # serial_out changes
                    # serial_in readings
                    }

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
            self.networker = netw.Dummy_Networker(mode='agent', agent=agent)
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
                                          # new:
                                          mode=self.mode, sim_speed = self.sim_speed, task_tick_hz=self.task_tick_hz, 
                                          render_hz=self.render_hz, action_hold_steps=self.action_hold_steps,
        )
        
        #------------------------- TASK DISPLAY -------------------------

        if self.display_config is not None and not self.headless:
            main_loops.visual = main_loops.Visual(self.machine, display_config=self.display_config, 
                                                  run_controls=self.run_controls, threads_info=self.threads_info,
                                                  run_at_visual_start=run_at_visual_start)
        #------------------------- LOAD ASSETS -------------------------
        
        if self.display_config is not None and not self.headless:
            from py5 import Sketch as _Sketch
            Sketch = _Sketch
            class Pre_Task(Sketch):
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
            # Headless: optionally call pre_task(None) without py5
            for block in self.machine.blocks.values():
                for state in block.values():
                    # only call if state overrides pre_task and can handle None
                    try:
                        state.pre_task(None)
                    except Exception:
                        # state.pre_task expects a Sketch; in headless we skip assets
                        pass

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

        if self.main_as_sketch:
            # more priority/consistency amidst parallel processes like camera capturing
            main_loops.main.run_sketch(block=True)
        else:
            # faster but less consistent frame intervals amidst parallel processes
            self.networker.write_teensy_data(self.serial_out)
            while main_loops.main.running:
                main_loops.main.draw()

        if self.log_dir is None:
            while True:
                try: 
                    self.tempdir.cleanup()
                    break
                except PermissionError as e:
                    # some process (likely video saving) is still utilizing the temp dir preventing deletion - try again later
                    time.sleep(0.1)
                    
                    
    # NEW: agent mode methods
    def get_obs(self, keys: list[str] | None = None) -> dict:
        """Observation for agents/RL. Gets the current serial_in values and task context."""
        from . import controls
        if keys is None:
            keys = [k for k in self.serial_in.keys() if k != 't_ms']
        obs_vec = [self.serial_in[k].get('value') for k in keys]
        return {
            'vector': obs_vec,
            'keys': keys,
            '_state': getattr(controls.get, 'current_state', None),
            '_block': getattr(controls.get, 'current_block', None),
            '_t_ms': self.serial_in['t_ms']['value'],
        }
    
    def get_reward_dict(self, include_input: bool = False) -> dict:
        """Get a dictionary of output values that would be considered a reward by the current task. Optionally include input values as well. This can be used as an interface to define a reward function for RL agents based on the task itself."""
        from . import controls
        reward_dict = {k: v.get('value') for k, v in self.serial_out.items()}
        if include_input:
            reward_dict.update({k: v.get('value') for k, v in self.serial_in.items()})
        # add task context
        reward_dict['_state'] = getattr(controls.get, 'current_state', None)
        reward_dict['_block'] = getattr(controls.get, 'current_block', None)
        return reward_dict

    def reset(self, clear_log: bool = True, reset_io: bool = True):
        """
        TODO add docstring
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
            for k in ['events', 'trials', 'states', 'blocks']:
                self.log[k].clear()
            self.log['controls'].clear()
            
        self.machine.stop_state_machine()
        # TODO: self.machine.reset()
        self.machine.start_state_machine()                
    
    def step(self, action: dict, n_ticks: int=1):
        """
        TODO add docstring
        """
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
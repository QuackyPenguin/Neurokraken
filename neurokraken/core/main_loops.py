from py5 import Sketch
from typing import Callable
import pathlib, json, pickle
main, visual = None, None

class Main(Sketch):
    def __init__(self, networker, serial_in, serial_out, run_controls, log:dict, log_dir:str, state_machine,
                 max_framerate=8_000, permanent_states:list[Callable]=[], threads_info:dict={}, log_performance=False,
                 run_at_start:Callable=lambda:None, run_at_quit:Callable=lambda:None, run_post_trial:Callable=lambda:None,
                 mode:str='teensy', sim_speed:float=1.0, task_tick_hz:int=200, render_hz:int=0, action_hold_steps:int=1):
        super().__init__()
        self.netw = networker
        self.serial_in, self.serial_out = serial_in, serial_out
        self.log_dict = log # py5 already has self.log()
        self.log_dir = log_dir
        self.machine = state_machine
        self.max_framerate = max_framerate
        self.permanent_states = permanent_states
        self.run_controls = run_controls
        self.threads_info = threads_info
        self.run_post_trial = run_post_trial
        self.run_at_start = run_at_start
        self.run_at_quit = run_at_quit
        self.log_performance = log_performance
        self.mode = mode
        self.sim_speed = sim_speed
        self.task_tick_hz = task_tick_hz
        self.render_hz = render_hz
        self.action_hold_steps = action_hold_steps

        self.running = True # pulse for standalone no-sketch use

    def _init_state(self):
        """Initialize runtime state that is normally set up in setup().
        Called directly in headless/agent mode where the py5 sketch never starts."""
        if self.log_performance:
            self.log_dict['t_main_loop'] = []
            self.log_dict['t_received'] = []
        self.serialout_key_lastval_updated = [[k, v['value'], False] for k, v in self.serial_out.items() if not k == 'start_stop']
        for out in self.serialout_key_lastval_updated:
            self.log_dict['controls'][out[0]] = [ [0, out[1]] ]
        self.netw.write_teensy_data(self.serial_out)

    def settings(self):
        self.size(10,10)

    def setup(self):
        self.window_title('Neurokraken Main Thread')
        self.get_surface().set_visible(False)
        self.frame_rate(self.max_framerate)
        self._init_state()

    def draw(self):
        if self.await_update():
            return

        data_updated, _ = self.netw.read_teensy_data(self.serial_in)
        if self.run_controls.active and not self.netw.archivist_mode:
            self.log_serial(self.serial_in, time='t_ms', log=self.log_dict)
            # otherwise the archivist_mode networker takes care of logging

        if self.run_controls.active:
            finished, next_state_name, trial_complete = self.machine.current_state.run()

            if finished:
                if trial_complete:
                    self.complete_trial(next_state_name)
                else:
                    self.machine.progress_state(next_state_name)

        if data_updated:
            if self.run_controls.active:
                # log changes
                for out in self.serialout_key_lastval_updated:
                    if out[2]:
                        # was marked for logging at the last communication
                        self.log_dict['controls'][out[0]].append( (self.serial_in['t_ms']['value'], out[1]) )
                        out[2] = False
                    if out[1] != self.serial_out[out[0]]['value']:
                        # keep the value and mark it for logging upon the next commnication with its time of action
                        out[1] = self.serial_out[out[0]]['value']
                        out[2] = True
            
            self.netw.write_teensy_data(self.serial_out)

            if self.log_performance and self.run_controls.active:
                self.log_dict['t_received'].append(self.serial_in['t_ms']['value'])
        if self.log_performance and self.run_controls.active:
            self.log_dict['t_main_loop'].append(self.serial_in['t_ms']['value'])
        
        for state in self.permanent_states:
            state.run()

        if self.is_running:
            # the main loop is running in sketch mode
            self.threads_info['framerate_main'] = self.get_frame_rate()

    def complete_trial(self, next_state_name):
        # run user functions
        self.run_post_trial()
        
        # a run_post_trial action may already have switched the block and thus progressed the trial and state
        if not self.log_dict['trials'][-1]['start'] == self.serial_in['t_ms']['value']:
            self.machine.progress_trial()
            self.machine.progress_state(next_state_name)

    def await_update(self):
        """Perform a teensy communication ahead of the main loop. This is useful for niche situations
        where the teensy needs to perform an action before a followup event"""
        keep_awaiting_activation = self.run_controls.beginning
        if keep_awaiting_activation:
            # don't run the main loop yet, wait until a serial write-and-read occured to read_teensy_data()
            # with updated time for the other devices before setting 'active'=True
            self.serial_out['start_stop']['value'] = 1 # start the clock
            data_updated, _ = self.netw.read_teensy_data(self.serial_in)
            if data_updated:
                # run user functions - i.e. set starting position for motors
                if callable(self.run_at_start):
                    self.run_at_start()
                else:
                    [func() for func in self.run_at_start]

                self.netw.write_teensy_data(self.serial_out)
                
                communications = 0
                while communications < 2:
                    received_updated_serial, _ = self.netw.read_teensy_data(self.serial_in)
                    if received_updated_serial:
                        self.netw.write_teensy_data(self.serial_out)
                        communications += 1

                self.run_controls.beginning = False
                self.run_controls.active = True

                # some time already passed since the experiment was defined - reset start_time of the first state
                self.machine.current_state.reset_time()
        
        keep_awaiting_quit = self.run_controls.quitting
        if keep_awaiting_quit:
            # As a safety against a previous run's pulse clock still running at an experiment start
            # and creating a false synchronization signal: stop the pulse clock when quiting.
            data_updated, _ = self.netw.read_teensy_data(self.serial_in)

            if data_updated:
                # run user functions
                if callable(self.run_at_quit):
                    self.run_at_quit()
                else:
                    [func() for func in self.run_at_quit]
                
                self.serial_out['start_stop']['value'] = 2 # end clock
                self.netw.write_teensy_data(self.serial_out)
                self.save_log()
                self.netw.close()
                self.running = False
                if self.is_running:
                    self.exit_sketch()
        
        keep_awaiting = keep_awaiting_activation or keep_awaiting_quit
        return keep_awaiting
    
    def log_serial(self, serial_in:dict, time:str='t_ms', log:dict={}):
        """Append the current 'value' entries of serial_in to the corresponding self.log entries.
        Entries that haven't changed or are 'logging=False' will be skipped

        Args:
            serial_in (dict): Serial Dictionary used by the networker
            time (str): This serial_in key defines the value used as time stamp
        """
        time = serial_in[time]['value']

        for key, data in serial_in.items():
            if data['logging']:
                history = log.setdefault(key, [])

                current_val = data['value']
                try:
                    if current_val != history[-1][1]:
                        history.append((time, current_val))
                except IndexError:
                    # there is no previous list entry yet
                    history.append((time, current_val))
    
    def save_log(self, format='.json', filename:str='log'):
        file_path = (self.log_dir / (filename + format)).resolve()
        if format == '.json':
            with open(file_path, 'w') as f:
                json.dump(self.log_dict, f, indent=2)
        elif format == '.pickle':
            with open(file_path, 'wb') as f:
                pickle.dump(self.log_dict, f, protocol=pickle.HIGHEST_PROTOCOL)

class Visual(Sketch):
    def __init__(self, state_machine, display_config, run_controls, 
                 threads_info:dict={}, run_at_visual_start:Callable[[Sketch], None]=None):
        super().__init__()
        self.machine = state_machine
        self.run_controls = run_controls
        self.threads_info = threads_info
        self.display_config = display_config
        
        self.run_at_visual_start = (lambda sketch: None) if run_at_visual_start is None else run_at_visual_start

    def settings(self):
        renderers = {'JAVA2D': self.JAVA2D,
                     'P2D': self.P2D,
                     'P3D': self.P3D}
        renderer = renderers[self.display_config.renderer]
        if not self.display_config.borderless:
            self.size(self.display_config.size[0], self.display_config.size[1], renderer)
        else:
            self.full_screen(renderer)

    def setup(self):
        surface = self.get_surface()
        surface.set_title('Kraken Visual')
        # surface.set_always_on_top(True)
        self.window_move(self.display_config.position[0], self.display_config.position[1])

        # py5graphics need to be created from the sketch that uses it to not
        # draw outside the respective draw loop. Create objects with this or
        # similar requirements in teh run_at_visual_start function.
        self.run_at_visual_start(self)

    def draw(self):
        # window_resize(w,h) could be called in def setup() with the JAVA2D renderer.
        # but would error with P2D or P3D until pixels exist in the sketch.
        # The alternative approach for a borderless size is to provide self.full_screen()
        # with a display index for the size of that display.
        # Revisit this approach if issues appear due to the first frame starting at different size
        if self.frame_count == 1: # first frame (no 0-indexing)
            if self.display_config.borderless:
                self.window_resize(self.display_config.size[0], self.display_config.size[1])

        if self.run_controls.quitting:
            self.exit_sketch()

        self.machine.current_state.loop_visual(self)

        self.threads_info['framerate_visual'] = self.get_frame_rate()
        
import random
from typing import Callable, Any, Self, Tuple
from core.print0 import print0

class State:
    """This base class contains the core functionalities to run a state and progress to the 
    next state. Once loop_main() returns True it is finished and the state will
    progress to its successor next_state. If the provided next_state is a list, then 
    loop_main()'s 2nd returned value, a outcome number, selects the list index.

    Args:
        next_state (str | list): The next state or list of possible next states.
                                    If you provide a list, the return of your def loop_main()
                                    will determine the next state. I.e. return True, 2 would
                                    progress to the the list entry[2].
        max_time_s (int | tuple[int,int] | Callable[[],float], optional): 
            The maximum state duration (seconds) after which the next state will be entered automatically.
            Defaults to 1_000_000.0.
        run_at_start (Callable | list[Callable], optional):
            A function or list of functions to run at the very start of this state.
            The function(s) may optionally accept 1 argument: state (State, optional)
            which can be used to access the state from within the provided function.
            Defaults to ().
        run_at_end (Callable | list[Callable], optional):
            A function or list of functions to run at the very end of this state.
            The function(s) may optionally accept 1 or 2 arguments: state (State, optional), finished (bool, optional) 
            which can be used to access the state from within the provided function and whether it has completed or timed out.
            Defaults to ().
        trial_complete (bool, optional): If True progresses the trials log.
                                         If your experiment has a run_post_trial function it will
                                         also run after all states with trial_complete=True.
                                         Defaults to False.
    """
    def __init__(self, next_state:str|list, max_time_s:float|tuple[float,float]|Callable[[],float]=1_000_000.0,
                 run_at_start:Callable[[Self],      Any] | list[Callable[[Self],      Any]]=(),
                 run_at_end:  Callable[[Self, bool],Any] | list[Callable[[Self, bool],Any]]=(),
                 trial_complete=False):
        self.max_t = max_time_s
        self.max_t_range = self.max_t if isinstance(self.max_t, tuple) else None
        self.max_t_func = self.max_t if callable(self.max_t) else None
        self.next_state = next_state
        self.run_at_start = run_at_start
        self.run_at_end = run_at_end
        self.trial_complete = trial_complete
        self.start_time:int = 0
        # the name and time access are provided by the state machine upon define_experiment() from its used blocks dictionary and serial_in
        self.name:str = ''
        self.t_ms = {'value': 0}

    def run_at_start_wrapper(self):
        # rather than always providing self to the callback function - self.run_at_start(self) -
        # provide the number of arguments *[] or *[state] that the provided callback function expects
        if callable(self.run_at_start):
            self.run_at_start(*[self][:self.run_at_start.__code__.co_argcount])
        else:
            for func in self.run_at_start:
                func(*[self][:func.__code__.co_argcount])
    
    def run_at_end_wrapper(self, finished):
        # rather than always providing self, finished to the callback function
        # - self.run_at_end(self, finished) - # provide the number of arguments
        # *[] or *[state] or *[state, finished] the provided callback function expects
        if callable(self.run_at_end):
            self.run_at_end(*[self, finished][:self.run_at_end.__code__.co_argcount])
        else:
            for func in self.run_at_end:
                func(*[self, finished][:func.__code__.co_argcount])
    
    def pre_task(self, sketch)->None:
        """This function will be run within the py5 sketch setup. It allows you for for example
        load textures that the State is going to use here at program start using the function's
        sketch argument to access py5. It can also be used for other py5 functions you only want
        to use once when setting up your state.
        
        Example usage in your class MyState(State):
        
        def pre_task(self, sketch):
            self.my_texture_A = sketch.load_image('tasks/mytask/mytextureA.png')
            self.my_texture_B = sketch.load_image('tasks/mytask/mytextureB.png')
        """
        pass

    def loop_main(self)->Tuple[bool,int]:
        '''this function runs state specific code - i.e. read a sensor or update the display
        It will be overwritten with the subclass provided loop_main function
        It returns a boolean True if the state is finished and an achieved outcome state'''
        return False, 0
    
    def loop_visual(self, sketch)->None:
        '''this function is to be overwritten with the subclass to show the visuals for the 
        current state. It is its own functions because unlike loop_main it will be called
        from the visual py5 sketch instance and not the main loop py5 sketch instance'''
        sketch.background(0)

    def on_start(self):
        '''on_start() will run at the start/restart/entry of a state 
        and can be overwritten to add functionality'''
        pass

    def on_end(self):
        '''on_end() will run at the end of a state and can be overwritten to add functionality'''
        pass
    
    def reset(self):
        self.reset_time()
        self.run_at_start_wrapper()
        self.on_start()

    def reset_time(self):
        self.start_time = self.t_ms['value']
        if self.max_t_range:
            self.max_t = random.uniform(self.max_t_range[0], self.max_t_range[1])
        elif self.max_t_func:
            self.max_t = self.max_t_func()

    def run(self)->Tuple[bool,str,bool]:
        """run the state's own function

        Returns:
            Tuple[bool,str,bool]:
                bool finished_or_timeout: the state was completed
                string: next_state_name - useful for updating metrics when a state was completed
                bool trial_complete - useful for updating metrics when a state was completed
        """
        
        finished, outcome = self.loop_main()
        if finished or self.t_ms['value'] - self.start_time > self.max_t * 1000:
            self.on_end()
            self.run_at_end_wrapper(finished)
            # move on to the next state
            if not isinstance(self.next_state, list):
                # there is only 1 next state option to go to
                next_state_name = self.next_state
            else:
                next_state_name = self.next_state[outcome]
            return True, next_state_name, self.trial_complete,
        return False, None, None

class State_Machine():
    def __init__(self, current_ms:dict, serial_out:dict, run_controls,
                 block_log:list=None, state_log:list=None, trial_log:list=None, verbose=True):
        '''the state machine will create a time window between start_state_machine() where 
        t_ms = 0 and stop_state_machine().
        stop_state_machine() allows for an non-program-quitting end to the running states
        verbose=True will print progression information.'''
        
        self.blocks:dict[str, State] = {}
        self.current_block:str = None
        self.trial_states = {}
        self.current_state:State = None
        self.starting_state:State = None

        # logging
        self.t_ms = current_ms
        self.block_log = []
        if not block_log is None:
            self.block_log = block_log
        self.state_log = []
        if not state_log is None:
            self.state_log = state_log
        self.trial_log = []
        if not trial_log is None:
            self.trial_log = trial_log

        self.current_block_trials = 0

        # controls
        self.serial_out = serial_out
        self.run_controls = run_controls
        self.was_stopped:bool = False

        print0.set_topic_threshold('state_machine', 6 if verbose else 0)
            
    #------------------------- STARTUP -------------------------

    def define_experiment(self, experiment_blocks:dict[str, State], start_block:str=None):
        self.blocks.update(experiment_blocks)
        for block in self.blocks.values():
            for state_name, state in block.items():
                state.name = state_name
                state.t_ms = self.t_ms
        if start_block is None:
            # the start_block is the first block key
            start_block = [*self.blocks][0]
        # remembered so reset() can return the machine to the same starting block for a new RL episode
        self._start_block = start_block
        self.switch_block(start_block)

    #------------------------- PROGRESS BLOCK/TRIAL/STATE -------------------------

    def progress_state(self, next_state_name:str):
        # reset() the upcoming state before it becomes the self.current_state parallel loops will access
        self.starting_state = self.trial_states[next_state_name]
        self.starting_state.reset_time()
        self.starting_state.on_start() 
        self.starting_state.run_at_start_wrapper()
        self.current_state = self.starting_state
        
        print0(f'progressed to state: {next_state_name}', priority=3, color='white', topic='state_machine')
        self.state_log.append((self.t_ms['value'], next_state_name))

    def progress_trial(self):
        self.trial_log.append({'start': self.t_ms['value']})
        self.current_block_trials += 1

    def switch_block(self, new_block:str):
        self.current_block = new_block

        self.trial_states = self.blocks[self.current_block]
        # start with the first state (dictionary order is preserved since python 3.7)
        print0(f'switching to block: {new_block}', priority=3, color='bright_white', topic='state_machine')
        self.block_log.append({'t': self.t_ms['value'], 'block': new_block})
        if not self.current_state:
            # this is the first time this program runs
            self.current_state = list(self.trial_states.values())[0]
        self.progress_trial()
        self.progress_state(next_state_name=list(self.trial_states)[0])
        self.current_block_trials = 0

    #------------------------- BEGIN/END A RUN -------------------------
    def start_state_machine(self):
        if self.was_stopped:
            # starting resets the experiment time => prevent restarts after ended experiments
            print0('state_machine was already started and stopped - no action will be taken',
                   priority=3, color='cyan', topic='state_machine')
            return
        if self.run_controls.active:
            print0('state_machine already active - no action will be taken', 
                   priority=3, color='cyan', topic='state_machine')
            return
        print0('resetting t_ms to 0, starting state, pulse clock, and camera saving',
               priority=3, color='cyan', topic='state_machine')
        self.serial_out['start_stop']['value'] = 1 # start the clock
        self.run_controls.beginning = True
        # the main loop will now  run the initial task communication and
        # then set run_controls.active=True

    def stop_state_machine(self):
        if not self.run_controls.active:
            print0('state_machine already inactive - no action will be taken', 
                   priority=3, color='cyan', topic='state_machine')
            return
        print0('stopping state machine and resetting teensy pulse clock to HIGH',
               priority=3, color='cyan', topic='state_machine')
        self.serial_out['start_stop']['value'] = 2 # end clock
        self.run_controls.active = False
        self.was_stopped = True

    def quit(self):
        "stop the state machine, save the log and exit the program"
        self.run_controls.quitting = True

    def reset(self):
        """Reset the state machine to its starting block/state for a new RL episode.

        Clears the was_stopped guard so the machine can be re-started after stop_state_machine(),
        zeros the trial counter, and re-enters the originally configured start_block via
        switch_block (which fires the new starting state's on_start / run_at_start hooks).
        """
        self.was_stopped = False
        self.run_controls.active = False
        self.run_controls.beginning = False
        self.current_block_trials = 0
        # forces switch_block to take its "first time this program runs" branch and seat current_state
        self.current_state = None
        self.switch_block(self._start_block)


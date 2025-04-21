#!/usr/bin/env python

from pid_controllers import NormalPID, FractionalPID, AdaptiveFractionalPID, NonlinearFractionalPID, TimeDelayFractionalPID

class PIDWrapper:
    """
    Wrapper class for PID controllers to manage distance and heading controllers
    """
    def __init__(self, controller_type, kp, ki, kd, alpha=0.5, distance_limits=(0.1, 1.0), heading_limits=(-1.0, 1.0)):
        """
        Initialize the PID wrapper
        
        Args:
            controller_type: Type of PID controller ('normal', 'fractional', 'adaptive', 'nonlinear', 'timedelay')
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            alpha: Fractional order (for fractional PID controllers)
            distance_limits: Output limits for distance controller
            heading_limits: Output limits for heading controller
        """
        self.controller_type = controller_type
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        self.distance_limits = distance_limits
        self.heading_limits = heading_limits
        
        # Create distance and heading controllers
        self.distance_controller = self._create_controller(0.0, distance_limits)
        self.heading_controller = self._create_controller(0.0, heading_limits)
    
    def _create_controller(self, setpoint, output_limits):
        """
        Create a PID controller of the specified type
        
        Args:
            setpoint: Desired setpoint
            output_limits: Output limits (min, max)
            
        Returns:
            PID controller instance
        """
        if self.controller_type == 'normal':
            return NormalPID(
                Kp=self.kp,
                Ki=self.ki,
                Kd=self.kd,
                setpoint=setpoint,
                output_limits=output_limits
            )
        elif self.controller_type == 'fractional':
            return FractionalPID(
                Kp=self.kp,
                Ki=self.ki,
                Kd=self.kd,
                alpha=self.alpha,
                setpoint=setpoint,
                output_limits=output_limits
            )
        elif self.controller_type == 'adaptive':
            return AdaptiveFractionalPID(
                Kp=self.kp,
                Ki=self.ki,
                Kd=self.kd,
                alpha=self.alpha,
                setpoint=setpoint,
                output_limits=output_limits,
                adaptation_rate=0.01
            )
        elif self.controller_type == 'nonlinear':
            return NonlinearFractionalPID(
                Kp=self.kp,
                Ki=self.ki,
                Kd=self.kd,
                alpha=self.alpha,
                setpoint=setpoint,
                output_limits=output_limits,
                sigmoid_factor=1.0
            )
        elif self.controller_type == 'timedelay':
            return TimeDelayFractionalPID(
                Kp=self.kp,
                Ki=self.ki,
                Kd=self.kd,
                alpha=self.alpha,
                setpoint=setpoint,
                output_limits=output_limits,
                delay_steps=5
            )
        else:
            raise ValueError("Unknown controller type: {}".format(self.controller_type))
    
    def update_distance_setpoint(self, setpoint):
        """
        Update the distance controller setpoint
        
        Args:
            setpoint: New setpoint
        """
        self.distance_controller.setpoint = setpoint
    
    def update_heading_setpoint(self, setpoint):
        """
        Update the heading controller setpoint
        
        Args:
            setpoint: New setpoint
        """
        self.heading_controller.setpoint = setpoint
    
    def compute_distance_control(self, current_distance):
        """
        Compute the distance control output
        
        Args:
            current_distance: Current distance
            
        Returns:
            Control output
        """
        return self.distance_controller(current_distance)
    
    def compute_heading_control(self, current_heading):
        """
        Compute the heading control output
        
        Args:
            current_heading: Current heading
            
        Returns:
            Control output
        """
        return self.heading_controller(current_heading)
    
    def reset(self):
        """
        Reset the controllers
        """
        # Create new controllers to reset internal state
        self.distance_controller = self._create_controller(
            self.distance_controller.setpoint, 
            self.distance_limits
        )
        self.heading_controller = self._create_controller(
            self.heading_controller.setpoint, 
            self.heading_limits
        ) 
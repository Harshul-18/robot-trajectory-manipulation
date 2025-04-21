#!/usr/bin/env python

import time
import math
import numpy as np
from scipy.special import gamma

class NormalPID:
    """
    Standard PID controller implementation
    """
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._last_time = None
        self._last_error = None
        self._integral = 0
    
    def __call__(self, current_value):
        """
        Update the PID controller with the current value
        
        Args:
            current_value: Current process variable
            
        Returns:
            Control output
        """
        error = self.setpoint - current_value
        
        # Get current time
        current_time = time.time()
        
        # Initialize last_time and last_error if this is the first call
        if self._last_time is None:
            self._last_time = current_time
            self._last_error = error
        
        # Calculate time difference
        dt = current_time - self._last_time
        
        # Calculate P term
        p_term = self.Kp * error
        
        # Calculate I term
        self._integral += error * dt
        i_term = self.Ki * self._integral
        
        # Calculate D term
        if dt > 0:
            d_term = self.Kd * (error - self._last_error) / dt
        else:
            d_term = 0
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits if specified
        if self.output_limits[0] is not None and output < self.output_limits[0]:
            output = self.output_limits[0]
        elif self.output_limits[1] is not None and output > self.output_limits[1]:
            output = self.output_limits[1]
        
        # Save current error and time for next iteration
        self._last_error = error
        self._last_time = current_time
        
        return output

class FractionalPID:
    """
    Fractional PID controller implementation
    """
    def __init__(self, Kp, Ki, Kd, alpha, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.alpha = alpha  # Fractional order
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._last_time = None
        self._errors = []
        self._times = []
    
    def __call__(self, current_value):
        """
        Update the Fractional PID controller with the current value
        
        Args:
            current_value: Current process variable
            
        Returns:
            Control output
        """
        error = self.setpoint - current_value
        
        # Get current time
        current_time = time.time()
        
        # Initialize last_time if this is the first call
        if self._last_time is None:
            self._last_time = current_time
        
        # Store error and time
        self._errors.append(error)
        self._times.append(current_time)
        
        # Keep only the last 100 points to limit memory usage
        if len(self._errors) > 100:
            self._errors = self._errors[-100:]
            self._times = self._times[-100:]
        
        # Calculate P term
        p_term = self.Kp * error
        
        # Calculate fractional I term
        i_term = self.Ki * self.calculate_fractional_integral(self._errors, self._times, self.alpha)
        
        # Calculate fractional D term
        d_term = self.Kd * self.calculate_fractional_derivative(self._errors, self._times, self.alpha)
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits if specified
        if self.output_limits[0] is not None and output < self.output_limits[0]:
            output = self.output_limits[0]
        elif self.output_limits[1] is not None and output > self.output_limits[1]:
            output = self.output_limits[1]
        
        # Save current time for next iteration
        self._last_time = current_time
        
        return output
    
    def calculate_fractional_integral(self, errors, times, alpha):
        """
        Calculate the fractional integral using the Grunwald-Letnikov definition
        """
        if len(errors) < 2:
            return 0
        
        h = (times[-1] - times[0]) / (len(times) - 1)  # Average time step
        integral = 0
        
        for i in range(len(errors)):
            weight = 1.0 / gamma(alpha) * (times[-1] - times[i])**(alpha-1)
            integral += weight * errors[i] * h
        
        return integral
    
    def calculate_fractional_derivative(self, errors, times, alpha):
        """
        Calculate the fractional derivative using the Grunwald-Letnikov definition
        """
        if len(errors) < 2:
            return 0
        
        h = (times[-1] - times[0]) / (len(times) - 1)  # Average time step
        
        if h == 0:
            return 0
        
        # Use the last two points for a simple approximation
        derivative = (errors[-1] - errors[-2]) / h
        
        return derivative

class AdaptiveFractionalPID(FractionalPID):
    """
    Adaptive Fractional PID controller implementation
    """
    def __init__(self, Kp, Ki, Kd, alpha, setpoint=0, output_limits=(None, None), adaptation_rate=0.01):
        super(AdaptiveFractionalPID, self).__init__(Kp, Ki, Kd, alpha, setpoint, output_limits)
        self.adaptation_rate = adaptation_rate
        self._last_error = 0
    
    def __call__(self, current_value):
        """
        Update the Adaptive Fractional PID controller with the current value
        
        Args:
            current_value: Current process variable
            
        Returns:
            Control output
        """
        error = self.setpoint - current_value
        
        # Get current time
        current_time = time.time()
        
        # Initialize last_time if this is the first call
        if self._last_time is None:
            self._last_time = current_time
            self._last_error = error
        
        # Store error and time
        self._errors.append(error)
        self._times.append(current_time)
        
        # Keep only the last 100 points to limit memory usage
        if len(self._errors) > 100:
            self._errors = self._errors[-100:]
            self._times = self._times[-100:]
        
        # Adapt the fractional order based on error change
        error_change = abs(error - self._last_error)
        if error_change > 0.1:  # Threshold for adaptation
            # Increase alpha when error is changing rapidly
            self.alpha = min(0.99, self.alpha + self.adaptation_rate * error_change)
        else:
            # Decrease alpha when error is stable
            self.alpha = max(0.01, self.alpha - self.adaptation_rate * 0.1)
        
        # Calculate P term
        p_term = self.Kp * error
        
        # Calculate fractional I term
        i_term = self.Ki * self.calculate_fractional_integral(self._errors, self._times, self.alpha)
        
        # Calculate fractional D term
        d_term = self.Kd * self.calculate_fractional_derivative(self._errors, self._times, self.alpha)
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits if specified
        if self.output_limits[0] is not None and output < self.output_limits[0]:
            output = self.output_limits[0]
        elif self.output_limits[1] is not None and output > self.output_limits[1]:
            output = self.output_limits[1]
        
        # Save current error and time for next iteration
        self._last_error = error
        self._last_time = current_time
        
        return output

class NonlinearFractionalPID(FractionalPID):
    """
    Nonlinear Fractional PID controller implementation
    """
    def __init__(self, Kp, Ki, Kd, alpha, setpoint=0, output_limits=(None, None), sigmoid_factor=1.0):
        super(NonlinearFractionalPID, self).__init__(Kp, Ki, Kd, alpha, setpoint, output_limits)
        self.sigmoid_factor = sigmoid_factor
    
    def __call__(self, current_value):
        """
        Update the Nonlinear Fractional PID controller with the current value
        
        Args:
            current_value: Current process variable
            
        Returns:
            Control output
        """
        error = self.setpoint - current_value
        
        # Apply nonlinear transformation to error using sigmoid function
        nonlinear_error = 2.0 / (1.0 + math.exp(-self.sigmoid_factor * error)) - 1.0
        
        # Get current time
        current_time = time.time()
        
        # Initialize last_time if this is the first call
        if self._last_time is None:
            self._last_time = current_time
        
        # Store error and time
        self._errors.append(nonlinear_error)
        self._times.append(current_time)
        
        # Keep only the last 100 points to limit memory usage
        if len(self._errors) > 100:
            self._errors = self._errors[-100:]
            self._times = self._times[-100:]
        
        # Calculate P term
        p_term = self.Kp * nonlinear_error
        
        # Calculate fractional I term
        i_term = self.Ki * self.calculate_fractional_integral(self._errors, self._times, self.alpha)
        
        # Calculate fractional D term
        d_term = self.Kd * self.calculate_fractional_derivative(self._errors, self._times, self.alpha)
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits if specified
        if self.output_limits[0] is not None and output < self.output_limits[0]:
            output = self.output_limits[0]
        elif self.output_limits[1] is not None and output > self.output_limits[1]:
            output = self.output_limits[1]
        
        # Save current time for next iteration
        self._last_time = current_time
        
        return output

class TimeDelayFractionalPID(FractionalPID):
    """
    Time-delay Fractional PID controller implementation
    """
    def __init__(self, Kp, Ki, Kd, alpha, setpoint=0, output_limits=(None, None), delay_steps=5):
        super(TimeDelayFractionalPID, self).__init__(Kp, Ki, Kd, alpha, setpoint, output_limits)
        self.delay_steps = delay_steps
    
    def __call__(self, current_value):
        """
        Update the Time-delay Fractional PID controller with the current value
        
        Args:
            current_value: Current process variable
            
        Returns:
            Control output
        """
        error = self.setpoint - current_value
        
        # Get current time
        current_time = time.time()
        
        # Initialize last_time if this is the first call
        if self._last_time is None:
            self._last_time = current_time
        
        # Store error and time
        self._errors.append(error)
        self._times.append(current_time)
        
        # Keep only the necessary points
        max_points = max(100, self.delay_steps + 10)
        if len(self._errors) > max_points:
            self._errors = self._errors[-max_points:]
            self._times = self._times[-max_points:]
        
        # Use delayed error for P term if enough history is available
        if len(self._errors) > self.delay_steps:
            delayed_error = self._errors[-self.delay_steps]
            p_term = self.Kp * delayed_error
        else:
            p_term = self.Kp * error
        
        # Calculate fractional I term
        i_term = self.Ki * self.calculate_fractional_integral(self._errors, self._times, self.alpha)
        
        # Calculate fractional D term
        d_term = self.Kd * self.calculate_fractional_derivative(self._errors, self._times, self.alpha)
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits if specified
        if self.output_limits[0] is not None and output < self.output_limits[0]:
            output = self.output_limits[0]
        elif self.output_limits[1] is not None and output > self.output_limits[1]:
            output = self.output_limits[1]
        
        # Save current time for next iteration
        self._last_time = current_time
        
        return output 

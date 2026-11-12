import numpy as np

class RingBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = np.zeros(size, dtype=float)  # Initialize buffer with zeros
        self.index = 0  # Pointer to the current position
    
    def append(self, value):
        self.buffer[self.index] = value
        self.index = (self.index + 1) % self.size  # Move pointer in a circular manner
    
    def get_buffer(self):
        # Return the buffer in the correct order
        return np.concatenate((self.buffer[self.index:], self.buffer[:self.index]))
    
    def reset(self):
        self.buffer = np.zeros(self.size, dtype=float)  # Initialize buffer with zeros
        self.index = 0  # Pointer to the current position
import time

class Timer:
    def __init__(self, fps=60):
        self.fps = fps                   # Frames per second target
        self.frame_count = 0             # Total frames elapsed
        self.start_time = time.time()    # Real-world start time of the timer

    def tick(self):
        """Call this method once per frame to update the frame count."""
        self.frame_count += 1

    def get_frame_count(self):
        """Returns the total number of frames that have passed."""
        return self.frame_count

    def get_game_time(self):
        """Returns the ideal game time in seconds based on the frame count."""
        return self.frame_count / self.fps

    def get_real_time_elapsed(self):
        """Returns the real-world elapsed time in seconds."""
        return time.time() - self.start_time

    def get_fps(self):
        """Calculates and returns the current frames per second."""
        elapsed_time = self.get_real_time_elapsed()
        if elapsed_time > 0:
            return self.frame_count / elapsed_time
        else:
            return 0  # Avoid division by zero

    def reset(self):
        """Resets the timer and frame count."""
        self.start_time = time.time()
        self.frame_count = 0

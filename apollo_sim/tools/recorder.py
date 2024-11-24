import json
from typing import Dict

class Recorder(object):

    def __init__(self):
        self.frames = []

    def update(self, frame_data: Dict):
        self.frames.append(frame_data)

    def export(self, filename: str):
        with open(filename, 'w') as f:
            json.dump(self.frames, f, indent=4)

    def reset(self):
        self.frames = []
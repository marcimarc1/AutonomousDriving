from boundary import Boundary
from bertha.environment.drivearea import morphingcommonroad

class ScipyBoundary(Boundary):
    def __init__(self, scenario: morphingcommonroad, length: int):
        super().__init__(scenario=scenario, length=length)

    def set_boundaries(self):
        # workon setting boundaries
        pass

    def get_boundaries(self):
        return self.boundaries
    
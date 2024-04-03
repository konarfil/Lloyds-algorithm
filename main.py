from VoronoiGenerator import DelaunayGrid, VoronoiGrid, Vertex
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import random
from Lloyd import Lloyd

if __name__ == "__main__":
    algorithm = Lloyd(10)
    algorithm.iterate(20)
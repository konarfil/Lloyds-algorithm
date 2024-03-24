from VoronoiGenerator import DelaunayGrid, VoronoiGrid, Vertex
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import random

if __name__ == "__main__":
    vertexCount = 20
    vertices = []
    vertices_x = []
    vertices_y = []
    for i in range(vertexCount):
        x = random.random()
        y = random.random()
        vertices_x.append(x)
        vertices_y.append(y)
        vertices.append(Vertex(x, y))
    
    delGrid = DelaunayGrid(vertices)
    delGrid.triangulate()

    vorGrid = VoronoiGrid(delGrid)
    vorGrid.generate()

    fig, ax = plt.subplots()
    for cell_edges in vorGrid.cells.values():
        polygon_vertices = []
        for edge in cell_edges:
            line = Polygon([(edge.p[0].x, edge.p[0].y), (edge.p[1].x, edge.p[1].y)], closed=True, edgecolor='r', facecolor='none')
            ax.add_patch(line)

    # Set plot limits and labels
    ax.scatter(vertices_x, vertices_y, color='black', label='Points')
    ax.set_xlim(delGrid.box_middle.x - delGrid.box_size / 2 - 0.1, delGrid.box_middle.x + delGrid.box_size / 2 + 0.1)
    ax.set_ylim(delGrid.box_middle.y - delGrid.box_size / 2 - 0.1, delGrid.box_middle.y + delGrid.box_size / 2 + 0.1)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Multiple Polygons')

    # Show the plot
    plt.show()
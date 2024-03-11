from VoronoiGenerator import Vertex, DelaunayGrid
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

    fig, ax = plt.subplots()
    for triangle in delGrid.triangulation:
        polygon_vertices = [(triangle.vertices[0].x, triangle.vertices[0].y),
                            (triangle.vertices[1].x, triangle.vertices[1].y),
                            (triangle.vertices[2].x, triangle.vertices[2].y)]
        print(polygon_vertices)
        polygon = Polygon(polygon_vertices, closed=True, edgecolor='r', facecolor='none')
        ax.add_patch(polygon)
        print(triangle.middle.x, triangle.middle.y)
    
    ax.scatter(vertices_x, vertices_y, color='black', label='Points')
    # Set plot limits and labels
    ax.set_xlim(delGrid.box_middle.x - delGrid.box_size / 2, delGrid.box_middle.x + delGrid.box_size / 2)
    ax.set_ylim(delGrid.box_middle.y - delGrid.box_size / 2, delGrid.box_middle.y + delGrid.box_size / 2)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Multiple Polygons')

    # Show the plot
    plt.show()
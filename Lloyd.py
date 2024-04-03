import random
from VoronoiGenerator import Vertex, DelaunayGrid, VoronoiGrid
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

class Lloyd:
    def __init__(self, N_points):
        """
        Initialize the Lloyd's algorithm
        Randomly choose positions of initial vertices
        """
        self.vertices = []

        for i in range(N_points):
            self.vertices.append(Vertex(random.random(), random.random()))
    
    def iterate(self, N_iterations):
        """
        Run N iterations of the Lloyd's algorithm and save an image of the Voronoi grid at each iteration
        """
        for i in range(N_iterations):
            delGrid = DelaunayGrid(self.vertices)
            delGrid.triangulate()

            vorGrid = VoronoiGrid(delGrid)
            vorGrid.generate()

            #save the image of a grid as png
            fig, ax = plt.subplots()
            for cell_edges in vorGrid.cells.values():
                polygon_vertices = []
                for edge in cell_edges:
                    line = Polygon([(edge.p[0].x, edge.p[0].y), (edge.p[1].x, edge.p[1].y)], closed=True, edgecolor='r', facecolor='none')
                    ax.add_patch(line)

            vertices_x = []
            vertices_y = []
            for vertex in self.vertices:
                vertices_x.append(vertex.x)
                vertices_y.append(vertex.y)

            ax.scatter(vertices_x, vertices_y, color='black', label='Points')
            ax.set_xlim(delGrid.box_middle.x - delGrid.box_size / 2 - 0.1, delGrid.box_middle.x + delGrid.box_size / 2 + 0.1)
            ax.set_ylim(delGrid.box_middle.y - delGrid.box_size / 2 - 0.1, delGrid.box_middle.y + delGrid.box_size / 2 + 0.1)
            ax.set_aspect('equal', adjustable='box')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')

            plt.savefig('iteration_' + str(i) + '.png', dpi=200)
            plt.close()

            #for each voronoi cell calculate centroid
            new_vertices = []
            #for cell_edges in vorGrid.cells.values():
            for vertex, cell_edges in vorGrid.cells.items():
                first_vertex = cell_edges[0].p[0]
                centroid = Vertex(0, 0)
                total_area = 0
                #we devide the cell into triangles and calculate centroid and area of each one
                #centroid of the cell is then average of triangle centroids weighted by their areas
                for edge in cell_edges:
                    triangle_centroid = Vertex((first_vertex.x + edge.p[0].x + edge.p[1].x) / 3, (first_vertex.y + edge.p[0].y + edge.p[1].y) / 3)
                    triangle_area = 0.5*abs(first_vertex.x*(edge.p[0].y - edge.p[1].y) + edge.p[0].x*(edge.p[1].y - first_vertex.y) + 
                                            edge.p[1].x*(first_vertex.y - edge.p[0].y))
                    total_area += triangle_area
                    centroid.x += triangle_area*triangle_centroid.x
                    centroid.y += triangle_area*triangle_centroid.y

                #check if the cell is degenerated
                if total_area > 0.001:
                    centroid.x /= total_area
                    centroid.y /= total_area
                    new_vertices.append(centroid)
                else:
                    new_vertices.append(vertex)

            self.vertices = new_vertices
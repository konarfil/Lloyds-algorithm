from math import sqrt
from typing import List

class Vertex:
	"""
	Class representing a vertex in the grid
	"""
	def __init__(self, x : float, y : float):
		"""
		Initialize the vertex
		"""
		self.x = x
		self.y = y

class Edge:
	"""
	Class representing an edge connection two vertices
	"""
	def __init__(self, p1 : Vertex, p2 : Vertex):
		"""
		Initialize the edge usign its vertices
		"""
		self.p = [p1, p2]

	def __eq__(self, __value: Vertex) -> bool:
		"""
		Edges are equal if they share the same vertices
		"""
		if __value == None:
			return False
		else:
			return (self.p[0] == __value.p[0] and self.p[1] == __value.p[1]) or (self.p[0] == __value.p[1] and self.p[1] == __value.p[0])
		
class Triangle:
	"""
	Class representing a triangle in the Delaunay triangulation
	"""
	def __init__(self, p1 : Vertex, p2 : Vertex, p3 : Vertex):
		"""
		Initialize the triangle using three vertices
		"""

		#asign three vertices of the triangle
		self.vertices = [p1, p2, p3]

		#assign three edges
		self.edges = [Edge(p1, p2), Edge(p2, p3), Edge(p3, p1)]
		
		#calculate the middle of the circumcircle and its radius
		D        = 2*(p1.x*(p2.y - p3.y) + p2.x*(p3.y - p1.y) + p3.x*(p1.y - p2.y))
		middle_x = 1/D*((p1.x*p1.x + p1.y*p1.y)*(p2.y - p3.y) + (p2.x*p2.x + p2.y*p2.y)*(p3.y - p1.y)
					+ (p3.x*p3.x + p3.y*p3.y)*(p1.y - p2.y))
		middle_y = 1/D*((p1.x*p1.x + p1.y*p1.y)*(p3.x - p2.x) + (p2.x*p2.x + p2.y*p2.y)*(p1.x - p3.x)
					+ (p3.x*p3.x + p3.y*p3.y)*(p2.x - p1.x))
		self.middle = Vertex(middle_x, middle_y)
		self.r = sqrt((middle_x - p1.x)*(middle_x - p1.x) + (middle_y - p1.y)*(middle_y - p1.y))

	def vertex_in_circumcircle(self, vertex : Vertex) -> bool:
		#check if given vertex is in circumcircle of the triangle
		r = sqrt((self.middle.x - vertex.x)*(self.middle.x - vertex.x) + (self.middle.y - vertex.y)*(self.middle.y - vertex.y))
		return r < self.r
	
class DelaunayGrid:
	"""
	Class used to calculad Delaunay triangulation from list of vertices
	It uses the Bowyer-Watson algorithm
	Inspired by https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
	"""
	def __init__(self, vertices : List[Vertex]):
		"""
		Initialize the class with list of vertices to use for the triangulation
		"""
		self.vertices = vertices
		self.triangulation = []

		#find bouding box of the vertices
		max_x = max(p.x for p in vertices)
		min_x = min(p.x for p in vertices)
		max_y = max(p.y for p in vertices)
		min_y = min(p.y for p in vertices)
		self.box_size = max(max_x - min_x, max_y - min_y)*1.05
		self.box_middle = Vertex((min_x + max_x) / 2, (min_y + max_y) / 2)

	def triangulate(self):
		self.triangulation = []
		#add triangle containing all vertices
		supertriangle = Triangle(Vertex(self.box_middle.x - 5*self.box_size, self.box_middle.y - 5*self.box_size),
						   		Vertex(self.box_middle.x + 5*self.box_size, self.box_middle.y - 5*self.box_size),
								Vertex(self.box_middle.x, self.box_middle.y + 5*self.box_size))
		self.triangulation.append(supertriangle)

		#conctruct triangulation by adding vertices one by one
		for vertex in self.vertices:
			badTriangles = []
			for triangle in self.triangulation:
				if triangle.vertex_in_circumcircle(vertex):
					badTriangles.append(triangle)

			polygon = []
			for triangle1 in badTriangles:
				for edge1 in triangle1.edges:
					shared = False
					for triangle2 in badTriangles:
						if triangle1 == triangle2:
							continue
						for edge2 in triangle2.edges:
							if edge1 == edge2 and triangle1 != triangle2:
								shared = True
					if not shared:
						polygon.append(edge1)

			for triangle in badTriangles:
				self.triangulation.remove(triangle)
			for edge in polygon:
				self.triangulation.append(Triangle(vertex, edge.p[0], edge.p[1]))

		#remove all triangles sharing edge with the supertriangle	
		badTriangles = []
		for triangle in self.triangulation:
			for vertex1 in triangle.vertices:
				for vertex2 in supertriangle.vertices:
					if vertex1 == vertex2:
						badTriangles.append(triangle)
						
		
		for triangle in badTriangles:
			if triangle in self.triangulation:
				self.triangulation.remove(triangle)

class VoronoiGrid:
	"""
	Implementation of the Voronoi grid
	"""
	def __init__(self, delaunayGrid : DelaunayGrid):
		"""
		Initialize Voronoi grid with dual Delaunay grid
		"""
		self.delaunayGrid = delaunayGrid
		#create dictionary of cells; each cell is represented by a point from the Delaunay grid and a list of edges
		self.cells = {}
		for vertex in delaunayGrid.vertices:
			self.cells[vertex] = []

		#find bounding box edges
		box_vertices = [Vertex(delaunayGrid.box_middle.x + delaunayGrid.box_size / 2, delaunayGrid.box_middle.y + delaunayGrid.box_size / 2),
				  		Vertex(delaunayGrid.box_middle.x + delaunayGrid.box_size / 2, delaunayGrid.box_middle.y - delaunayGrid.box_size / 2),
						Vertex(delaunayGrid.box_middle.x - delaunayGrid.box_size / 2, delaunayGrid.box_middle.y - delaunayGrid.box_size / 2),
						Vertex(delaunayGrid.box_middle.x - delaunayGrid.box_size / 2, delaunayGrid.box_middle.y + delaunayGrid.box_size / 2)]
		self.box_edges = [Edge(box_vertices[0], box_vertices[1]), Edge(box_vertices[1], box_vertices[2]),
							Edge(box_vertices[2], box_vertices[3]), Edge(box_vertices[3], box_vertices[0])]

	def edge_edge_intersection(self, edge1 : Edge, edge2 : Edge):
		"""
		Finds intersection between two edges
		Formulas from https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
		"""

		t = ((edge1.p[0].x - edge2.p[0].x)*(edge2.p[0].y - edge2.p[1].y) - (edge1.p[0].y - edge2.p[0].y)*(edge2.p[0].x - edge2.p[1].x)) / \
			((edge1.p[0].x - edge1.p[1].x)*(edge2.p[0].y - edge2.p[1].y) - (edge1.p[0].y - edge1.p[1].y)*(edge2.p[0].x - edge2.p[1].x))
		u = -((edge1.p[0].x - edge1.p[1].x)*(edge1.p[0].y - edge2.p[0].y) - (edge1.p[0].y - edge1.p[1].y)*(edge1.p[0].x - edge2.p[0].x)) / \
			((edge1.p[0].x - edge1.p[1].x)*(edge2.p[0].y - edge2.p[1].y) - (edge1.p[0].y - edge1.p[1].y)*(edge2.p[0].x - edge2.p[1].x))
		
		if t > 0 and t < 1 and u > 0 and u < 1:
			return Vertex(edge1.p[0].x + t*(edge1.p[1].x - edge1.p[0].x), edge1.p[0].y + t*(edge1.p[1].y - edge1.p[0].y))
		else:
			return None


	def find_finite_edge(self, edge : Edge):
		"""
		Finds intersection of the edge with the bounding box
		Returns edge ending on the bouding box and the intersection
		"""
		for box_edge in self.box_edges:
			intersection = self.edge_edge_intersection(box_edge, edge)
			if intersection != None:
				if max(abs(edge.p[0].x - self.delaunayGrid.box_middle.x),
		   		 	abs(edge.p[0].y - self.delaunayGrid.box_middle.y)) < self.delaunayGrid.box_size / 2:
					return (intersection, Edge(edge.p[0], intersection))
				else:
					return (intersection, Edge(edge.p[1], intersection))
		return (None, None)

	def generate(self):
		"""
		Generate Voronoi grid from Delaunay grid
		"""
		if self.delaunayGrid.triangulation == []:
			print("Delaunay grid not generated !")
		else:
			#iterate through all triangles and find triangles which share edge
			#line between midpoints of two adjacent triangles is an edge of the voronoi grid
			for triangle1 in self.delaunayGrid.triangulation:
				for edge1 in triangle1.edges:
					shared = False
					for triangle2 in self.delaunayGrid.triangulation:
						#do not compare a triangle with itself
						if triangle1 == triangle2:
							continue
						for edge2 in triangle2.edges:
							#found common edge
							if edge1 == edge2:
								shared = True
								middle_edge = Edge(triangle1.middle, triangle2.middle)

								#check if the edge is in the bounding box
								if max(abs(middle_edge.p[0].x - self.delaunayGrid.box_middle.x), \
								abs(middle_edge.p[0].y - self.delaunayGrid.box_middle.y)) >= self.delaunayGrid.box_size / 2 and \
								max(abs(middle_edge.p[1].x - self.delaunayGrid.box_middle.x), \
								abs(middle_edge.p[1].y - self.delaunayGrid.box_middle.y)) >= self.delaunayGrid.box_size / 2:
									continue

								if middle_edge not in self.cells[edge1.p[0]]:
									self.cells[edge1.p[0]].append(middle_edge)
								if middle_edge not in self.cells[edge1.p[1]]:
									self.cells[edge1.p[1]].append(middle_edge)
					#found an edge on the rim of the Delaunay grid -> we need to construct an infinite edge of the Voronoi grid
					#we construct a line between the middle of the triangle and a second point lying outside of the plotting area
					#also we skip edges which would be outside of the bouding box
					if shared == False and max(abs(triangle1.middle.x - self.delaunayGrid.box_middle.x), \
								abs(triangle1.middle.y - self.delaunayGrid.box_middle.y)) < self.delaunayGrid.box_size / 2:
						
						edge_midpoint = Vertex((edge1.p[0].x + edge1.p[1].x) / 2, (edge1.p[0].y + edge1.p[1].y) / 2)

						#calculate vector conection between the middle of the triangle and the middle of the edge
						line_vector = Vertex(triangle1.middle.x - edge_midpoint.x, triangle1.middle.y - edge_midpoint.y)

						#find two candidate points on the line connecting triangle midpoint and edge midpoint
						candidate1 = Vertex(0, 0)
						candidate2 = Vertex(0, 0)
						if line_vector.x == 0:
							candidate1.x = triangle1.middle.x
							candidate1.y = triangle1.middle.y + self.delaunayGrid.box_size
							candidate2.x = triangle1.middle.x
							candidate2.y = triangle1.middle.y - self.delaunayGrid.box_size
						else:
							candidate1.x = self.delaunayGrid.box_middle.x - self.delaunayGrid.box_size
							candidate1.y = triangle1.middle.y + line_vector.y/line_vector.x*(candidate1.x - triangle1.middle.x)
							candidate2.x = self.delaunayGrid.box_middle.x + self.delaunayGrid.box_size
							candidate2.y = triangle1.middle.y + line_vector.y/line_vector.x*(candidate2.x - triangle1.middle.x)

						#find the third vertex of the triangle which is not on the edge
						third_vertex = Vertex(0, 0)
						for vertex in triangle1.vertices:
							if vertex != edge1.p[0] and vertex != edge1.p[1]:
								third_vertex = vertex
								break
						
						#check if candidate1 and third_point are on the same side of the edge1
						edge_vec = Vertex(edge1.p[0].x - edge1.p[1].x, edge1.p[0].y - edge1.p[1].y)
						candidate_vec1 = Vertex(candidate1.x - edge1.p[0].x, candidate1.y - edge1.p[0].y)
						third_point_vec = Vertex(third_vertex.x - edge1.p[0].x, third_vertex.y - edge1.p[0].y)

						cross_product1 = edge_vec.x*third_point_vec.y - edge_vec.y*third_point_vec.x
						cross_product2 = edge_vec.x*candidate_vec1.y - edge_vec.y*candidate_vec1.x

						#choose the candidate on different side than third point
						infinite_edge = Edge(triangle1.middle, Vertex(0, 0))
						if cross_product1*cross_product2 > 0:
							infinite_edge.p[1] = candidate2
						else:
							infinite_edge.p[1] = candidate1

						#add the infinite edge to the cells
						if infinite_edge not in self.cells[edge1.p[0]]:
							self.cells[edge1.p[0]].append(infinite_edge)
						if infinite_edge not in self.cells[edge1.p[1]]:
							self.cells[edge1.p[1]].append(infinite_edge)
						#this whole infitite edge thing is probably terribly overcomplicated but atleast it works
			
			#finally we need to find Voronoi cells which intersect the bouding box and add edges so that they are not infinite
			for cell_edges in self.cells.values():
				intersecitons = []
				infinite_edges = []
				finite_edges = []
				for edge in cell_edges:
					interseciton, finite_edge = self.find_finite_edge(edge)
					if interseciton != None:
						intersecitons.append(interseciton)
						infinite_edges.append(edge)
						finite_edges.append(finite_edge)

				if len(intersecitons) == 0:
					continue
				
				for i in range(len(infinite_edges)):
					cell_edges.remove(infinite_edges[i])
					cell_edges.append(finite_edges[i])

				#intersections are on different edges of the bounding box
				if abs(intersecitons[0].x - intersecitons[1].x) > 0.001 and \
					abs(intersecitons[0].y - intersecitons[1].y) > 0.001:
					corner = Vertex(0, 0)
					if abs(intersecitons[0].x - self.delaunayGrid.box_middle.x) > abs(intersecitons[1].x - self.delaunayGrid.box_middle.x):
						corner.x = intersecitons[0].x
						corner.y = intersecitons[1].y
					else:
						corner.x = intersecitons[1].x
						corner.y = intersecitons[0].y
					cell_edges.append(Edge(intersecitons[0], corner))
					cell_edges.append(Edge(intersecitons[1], corner))
				else:
					cell_edges.append(Edge(intersecitons[0], intersecitons[1]))
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
		self.p1 = p1
		self.p2 = p2

	def __eq__(self, __value: Vertex) -> bool:
		"""
		Edges are equal if they share the same vertices
		"""
		return (self.p1 == __value.p1 and self.p2 == __value.p2) or (self.p1 == __value.p2 and self.p2 == __value.p1)
		
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

	def find_shared_edge(self, triangle1 : Triangle, triangle2 : Triangle) -> Edge:
		"""
		Find shared edge of two triangles
		"""
		for edge1 in triangle1.edges:
			for edge2 in triangle2.edges:
				if edge1 == edge2:
					return edge1
		return None

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
						for edge2 in triangle2.edges:
							#print(edge1.p1.x, edge1.p1.y, edge1.p2.x, edge1.p2.y, edge2.p1.x, edge2.p1.y, edge2.p2.x, edge2.p2.y, edge1 == edge2)
							if edge1 == edge2 and triangle1 != triangle2:
								shared = True
								#print("AAAAAAAAAAAAAAAA")
					if not shared:
						polygon.append(edge1)

			for triangle in badTriangles:
				self.triangulation.remove(triangle)
			for edge in polygon:
				self.triangulation.append(Triangle(vertex, edge.p1, edge.p2))

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

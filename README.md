# Lloyds-algorithm
A small project implementing the Lloyd's algorithm. The algorithm takes a Voronoi grid as an input. It then iteratively changes positions of vertices to create a Voronoi grid where the cells are equally sized. The code also contains an implementation of Voronoi grid generator. It uses the Bowyer-Watson algorithm to first generate a Delaunay grid. From this grid it then constructs the dual Voronoi grid.
# Usage
Running the main.py script creates a Voronoi grid with 100 cells and then runs 30 iterations of the Lloyd's algorithm. A PNG image of the grid is saved after each iteration.
# Images
![Initial Voronoi grid](https://github.com/konarfil/Lloyds-algorithm/blob/main/iteration_0.png)
![After 30 iterations](https://github.com/konarfil/Lloyds-algorithm/blob/main/iteration_29.png)

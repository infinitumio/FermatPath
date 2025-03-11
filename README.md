# FermatPath: Optics-Inspired Pathfinding

FermatPath is a pathfinding algorithm inspired by **Fermat’s Principle** of least time. It combines a discrete A* search with dynamic refractive index modeling and continuous path refinement to simulate how a light ray travels through heterogeneous media.

---

## Mathematical Foundations

### 1. Fermat’s Principle and the "Least Time" Concept

Fermat’s Principle states that the path taken between two points by a ray of light is the one that can be traversed in the least time. In a medium with a spatially varying refractive index $$\( n(\mathbf{r}) \)$$, the travel time $$\( T \)$$ along a path $$\( \gamma \)$$ is given by:

$$
T[\gamma] = \int_\gamma \frac{n(\mathbf{r})}{c} \, ds,
$$

where:
- $$\( n(\mathbf{r}) \)$$ is the refractive index at point $$\( \mathbf{r} \)$$,
- $$\( ds \)$$ is an infinitesimal element of the path,
- $$\( c \)$$ is the speed of light in vacuum.

Since $$\( c \)$$ is constant, minimising the travel time is equivalent to minimising

$$
\int_\gamma n(\mathbf{r}) \, ds.
$$

The algorithm mimics this principle by assigning each grid cell a “cost” proportional to its refractive index. A move from one cell to a neighbor incurs a cost approximately given by:

$$
\text{Cost} \approx n_{ij} \cdot \Delta s,
$$

where $$\( n_{ij} \)$$ is the refractive index of the destination cell and $$\( \Delta s \)$$ is the step length (taken as 1 unit in our discrete model). Thus, the total cost of a path is:

$$
\text{Total Cost} = \sum_{\text{cells}} n_{\text{cell}},
$$

which is a discrete analog of the continuous integral.

---

### 2. A* Search and the Admissible Heuristic

The A* algorithm is optimal if its heuristic function $$\( h(\cdot) \)$$ is **admissible** (i.e., it never overestimates the true cost from a node to the goal).

#### Heuristic Used

The heuristic is defined as:

$$
h(i, j) = d\big((i, j), (i_g, j_g)\big) \cdot \overline{n},
$$

where:
- $$\( d((i, j), (i_g, j_g)) = \sqrt{(i - i_g)^2 + (j - j_g)^2} \)$$ is the Euclidean distance between the current node and the goal,
- $$\( \overline{n} \)$$ is the average refractive index along the straight-line path (computed by sampling).

Because in a homogeneous medium the straight line is the fastest path, this heuristic serves as a lower bound on the true cost, and is therefore admissible. As a result, A* will expand nodes in order of increasing $$\( f = g + h \)$$ and will eventually find the optimal (least-cost) path.

---

### 3. Discrete Cost and Continuous Approximation

### Segment Cost Function and Numerical Integration

The segment cost function approximates the cost to travel from point $$\((p_x, p_y)\)$$ to $$\((q_x, q_y)\)$$ by sampling along the straight line and averaging the refractive indices. Mathematically, it approximates:

$$
\text{Cost} \approx \ell \cdot \left(\frac{1}{\text{samples}} \sum_{k=0}^{\text{samples}-1} n_k\right),
$$

with

$$
\ell = \sqrt{(q_x - p_x)^2 + (q_y - p_y)^2}.
$$

This formulation is equivalent to a numerical integration of the refractive index along the segment.

#### Connection to the Euler–Lagrange Equation

In the continuous formulation, applying Fermat’s principle via the calculus of variations leads to the Euler–Lagrange equation. The solution of this equation gives the path of least time. Our algorithm approximates this process by:

- **Discretising** space into a grid.
- **Assigning** each cell a cost proportional to $$\( n(\mathbf{r}) \)$$.
- **Using A\*** to find a discrete minimum-cost path.
- **Optionally refining** the path in continuous space using gradient-descent–like adjustments.

---

### Continuous Path Refinement

After the discrete path is found, the function `refinePath` smooths the path using the following steps:

- **Perturbation:**  
  For each intermediate point $$\( \mathbf{r}_i \)$$, small perturbations are applied in various directions.

- **Cost Evaluation:**  
  The cost of the adjacent segments $$\([ \mathbf{r}_{i-1}, \mathbf{r}_i ]\)$$ and $$\([ \mathbf{r}_i, \mathbf{r}_{i+1} ]\)$$ is recalculated using the segment cost function.

- **Gradient Descent:**  
  The perturbation that yields the greatest reduction in cost is chosen, and the point is updated accordingly.

This iterative minimisation reduces the total optical path length:

$$
\sum_{i} n(\mathbf{r}_i) \, \Delta s_i,
$$

making the refined path a closer approximation to the true light-like trajectory predicted by Fermat’s principle.

---

### Putting This All Together

- **Grid Representation:**  
  The environment is modeled as a grid with obstacles (walls) and free space.

- **Cost Assignment:**  
  Each cell’s cost is based on its refractive index, simulating varying light speeds in different media.

- **A* Search:**  
  The algorithm finds a discrete path that minimises the cumulative cost, analogous to finding the path of least time. This could also be implemented in a Unweighted and Weighted fashion, replacing A* with BFS or Dijkstra

- **Admissible Heuristic:**  
  The heuristic—based on Euclidean distance multiplied by an average refractive index—ensures that A* finds an optimal solution.

- **Continuous Refinement:**  
  A gradient-descent procedure further minimises the optical path length, smoothing the discrete solution into a continuous, light-like trajectory.

---

### Summary

The mathematical foundation of FermatPath is built on:

- **Fermat’s Principle:**  
  Light travels along the path that minimises the integral
$$
  \int_\gamma n(\mathbf{r}) \, ds.
$$

- **Admissible Heuristic:**  
  The heuristic based on Euclidean distance and average refractive index never overestimates the true cost.

- **A* Optimality:**  
  With an admissible heuristic, A* guarantees an optimal discrete solution.

- **Numerical Integration:**  
  The segment cost function approximates the continuous optical path integral via sampling.

- **Gradient Descent Refinement:**  
  Continuous refinement adjusts the discrete path to better approximate the smooth, least-time path.

This integrated approach of discrete search and continuous optimisation results in an algorithm that not only computes a valid path but also closely emulates the behavior of a light ray propagating through a medium with spatially varying properties.

## Contributing

Contributions are welcome! If you have ideas for improvements, additional features, or bug fixes, please fork the repository and create a pull request. For any issues or feature requests, feel free to open an issue on the [issues page](https://github.com/infinitumio/FermatPath/issues).

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

Inspired by the principles of optics and Fermat's Principle of least time, this project is a blend of classical physics and modern pathfinding algorithms.

## Scholarly References

- **Fermat, P. de.** (1662). *Methodus ad Disquirendam Maximam et Minimam...*  
  The original treatise in which Fermat's Principle was first formulated, establishing the concept that light follows the path of least time.

- **Russell, S., & Norvig, P.** (2022). *Artificial Intelligence: A Modern Approach* (4th ed.). Prentice Hall.  
  This book provides a comprehensive introduction to search algorithms like A*, including the importance of admissible heuristics.

- **LaValle, S. M.** (2006). *Planning Algorithms*. Cambridge University Press.  
  This text covers a wide range of path planning and optimization techniques, many of which draw inspiration from physics and variational methods.

import heapq
import random
import time


# ==========================================
# 1. A* Algorithm Implementation
# ==========================================
def heuristic(a, b):
  """Manhattan distance heuristic."""
  return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(start, goal, grid):
  """Finds the shortest path from start to goal on a 2D grid."""
  rows, cols = len(grid), len(grid[0])

  open_set = []
  heapq.heappush(open_set, (0, start))

  came_from = {}
  g_score = {start: 0}
  f_score = {start: heuristic(start, goal)}
  open_set_hash = {start}

  neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

  while open_set:
    current = heapq.heappop(open_set)[1]
    open_set_hash.remove(current)

    if current == goal:
      path = []
      while current in came_from:
        path.append(current)
        current = came_from[current]
      path.append(start)
      return path[::-1]

    for dx, dy in neighbors:
      neighbor = (current[0] + dx, current[1] + dy)

      if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
        if grid[neighbor[0]][neighbor[1]] == 1:
          continue

        tentative_g_score = g_score.get(current, float("inf")) + 1

        if tentative_g_score < g_score.get(neighbor, float("inf")):
          came_from[neighbor] = current
          g_score[neighbor] = tentative_g_score
          f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

          if neighbor not in open_set_hash:
            heapq.heappush(open_set, (f_score[neighbor], neighbor))
            open_set_hash.add(neighbor)

  return None  # No path found


# ==========================================
# 2. Grid Generator & Benchmark Runner
# ==========================================
def generate_random_grid(rows, cols, obstacle_prob=0.2):
  """Generates a 2D grid with random obstacles (1 = obstacle, 0 = clear)."""
  grid = [[1 if random.random() < obstacle_prob else 0 for _ in range(cols)] for _ in range(rows)]
  return grid


def benchmark_a_star(rows=600, cols=300, obstacle_prob=0.2, num_trials=5):
  """Runs A* multiple times on newly generated random grids and reports stats."""
  print(f"--- A* Benchmark ({rows}x{cols} Grid, {obstacle_prob*100}% Obstacles) ---")
  
  start = (0, 0)
  goal = (rows - 1, cols - 1)
  
  times = []
  successful_runs = 0

  for i in range(num_trials):
    # Generate a fresh random grid
    grid = generate_random_grid(rows, cols, obstacle_prob)
    
    # Ensure start and goal are always walkable
    grid[start[0]][start[1]] = 0
    grid[goal[0]][goal[1]] = 0

    # Time the execution
    start_time = time.perf_counter()
    path = a_star_search(start, goal, grid)
    end_time = time.perf_counter()

    elapsed = end_time - start_time
    times.append(elapsed)

    if path:
      successful_runs += int(1)
      print(f"Trial {i+1}: Success | Time: {elapsed:.4f}s | Path Length: {len(path)}")
    else:
      print(f"Trial {i+1}: No path found (Blocked) | Time: {elapsed:.4f}s")

  # Summary statistics
  if times:
    print("\n--- Benchmark Summary ---")
    print(f"Total Trials: {num_trials}")
    print(f"Successful Paths Found: {successful_runs}/{num_trials}")
    print(f"Average Time: {sum(times) / len(times):.4f} seconds")
    print(f"Fastest Time: {min(times):.4f} seconds")
    print(f"Slowest Time: {max(times):.4f} seconds")


# Run the benchmark
if __name__ == "__main__":
  benchmark_a_star(rows=300, cols=150, obstacle_prob=0.20, num_trials=5)
from world_writer import WorldWriter
from difficulty_quant import DifficultyMetrics
from pgm_writer import PGMWriter
from yaml_writer import YamlWriter
from gen_world_ca import JackalMap, Display
import numpy as np
import random

def print_world(world):
  for i in range(len(world)):
    print(world[i])

cols = 15
rows = 16
start = 1
end = 1

def make_world1():
  global start, end
  world_base = np.zeros(shape=(rows, cols))

  for i in range(rows):
    for j in range(cols):
      if i == 0 or i == rows-1:
        world_base[i][j] = 1
      else:
        world_base[i][j] = 0

  world_base[1][8] = 1
  world_base[1][7] = 1
  world_base[13][:10] = 1
  world_base[14][:10] = 1

  world_base = np.flip(world_base, axis=1)
  print_world(world_base)

  start = 3
  end = 4
  return world_base

def make_world2():
  global start, end
  world_base = np.zeros(shape=(rows, cols))

  for i in range(rows):
    for j in range(cols):
      if i == 0 or i == rows-1:
        world_base[i][j] = 1
      else:
        world_base[i][j] = 0

  world_base[1][8] = 1
  world_base[1][7] = 1
  world_base[2][7] = 1
  world_base[3][7] = 1
  world_base[4][7] = 1
  world_base[5][7] = 1
  world_base[13][:10] = 1
  world_base[14][:10] = 1

  world_base = np.flip(world_base, axis=1)
  print_world(world_base)

  start = 3
  end = 4
  return world_base

def make_world3():
  global start, end
  world_base = np.zeros(shape=(rows, cols))

  for i in range(rows):
    for j in range(cols):
      if i == 0 or i == rows-1:
        world_base[i][j] = 1
      else:
        world_base[i][j] = 0

  world_base[1][11] = 1
  world_base[2][11] = 1
  world_base[3][11] = 1
  world_base[4][11] = 1
  world_base[5][11] = 1
  world_base[6][4] = 1
  world_base[7][4] = 1
  world_base[8][4] = 1
  world_base[9][4] = 1
  world_base[10][4] = 1
  world_base[11][4] = 1
  world_base[12][4] = 1
  world_base[13][:10] = 1
  world_base[14][:10] = 1

  world_base = np.flip(world_base, axis=1)
  print_world(world_base)

  start = 3
  end = 4
  return world_base

world = make_world3().tolist()

def main(ob_map_gen):
  global start, end
  jackal_radius = 2
  cyl_radius = 0.075
  rows = len(ob_map_gen)
  cols = len(ob_map_gen[0])


  # get map from the obstacle map generator
  obstacle_map = ob_map_gen
  print("boxes=", np.array(obstacle_map).sum())
  # generate jackal's map from the obstacle map
  jmap_gen = JackalMap(obstacle_map, jackal_radius)
  start_region = jmap_gen.biggest_left_region()
  end_region = jmap_gen.biggest_right_region()
  
  # throw out any maps that don't have a path
  if not jmap_gen.regions_connected(start_region, end_region):
    print("Start and end regions don't connect")
    return

  # get the final Jackal Map (C-space)
  jackal_map = jmap_gen.get_map()

  # choose random start and end points for path
  left_open = []
  right_open = []
  for r in range(len(jackal_map)):
    if start_region[r][0] == 1:
      left_open.append(r)
    if end_region[r][len(jackal_map[0])-1] == 1:
      right_open.append(r)

  if(start in left_open):
    left_coord_r = start#left_open[random.randint(0, len(left_open)-1)]
  else:
    print("Start coord is box")
    return
  if(end in right_open):
    right_coord_r = end#right_open[random.randint(0, len(right_open)-1)]
  else:
    print("End coord is box")
    return


  # generate path, if possible
  path = []
  diff_quant = DifficultyMetrics(jackal_map, path, disp_radius=3)
  dist_map = diff_quant.closest_wall()
  print('Points: (%d, 0), (%d, %d)' % (left_coord_r, right_coord_r, len(jackal_map[0])-1))
  path = jmap_gen.get_path([(left_coord_r, 0), (right_coord_r, len(jackal_map[0])-1)], dist_map)

  if not path:
    print('path not found')
    return # path not found, don't use this world

  print('Found path!')

  # put paths into matrices to display them
  obstacle_map_with_path = [[obstacle_map[j][i] for i in range(len(obstacle_map[0]))] for j in range(len(obstacle_map))]
  jackal_map_with_path = [[jackal_map[j][i] for i in range(len(jackal_map[0]))] for j in range(len(jackal_map))]
  for r, c in path:
    # update jackal-space path display
    jackal_map_with_path[r][c] = 0.35

    # update obstacle-space path display
    for r_kernel in range(r - jackal_radius, r + jackal_radius + 1):
      for c_kernel in range(c - jackal_radius, c + jackal_radius + 1):
        if 0 <= r_kernel and r_kernel < rows and 0 <= c_kernel and c_kernel < cols:
          obstacle_map_with_path[r_kernel][c_kernel] = 0.35


  jackal_map_with_path[left_coord_r][0] = 0.65
  jackal_map_with_path[right_coord_r][len(jackal_map[0])-1] = 0.65
  obstacle_map_with_path[left_coord_r][0] = 0.65
  obstacle_map_with_path[right_coord_r][len(obstacle_map[0])-1] = 0.65


  # save metrics
  diff = DifficultyMetrics(jackal_map, path, disp_radius=3)
  metrics_arr = np.asarray(diff.avg_all_metrics())
  print(metrics_arr)



  # display world and heatmap of distances
  if True:
    display = Display(obstacle_map_with_path, jackal_map, jackal_map_with_path, 3, path)
    display()

  return True # path found

if __name__ == "__main__":
    main(world)

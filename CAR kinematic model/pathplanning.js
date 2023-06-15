import * as np from 'numpy';
import * as math from 'math';
import * as scipy_interpolate from 'scipy.interpolate';
import { angle_of_line } from './utils';

function interpolate_b_spline_path(x, y, n_path_points, degree = 3) {
  const ipl_t = np.linspace(0.0, x.length - 1, x.length);
  const spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k = degree);
  const spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k = degree);
  const travel = np.linspace(0.0, x.length - 1, n_path_points);
  return [spl_i_x(travel), spl_i_y(travel)];
}

function interpolate_path(path, sample_rate) {
  const choices = np.arange(0, path.length, sample_rate);
  if (!choices.includes(path.length - 1)) {
    choices.push(path.length - 1);
  }
  const way_point_x = path[choices, 0];
  const way_point_y = path[choices, 1];
  const n_course_point = path.length * 3;
  const [rix, riy] = interpolate_b_spline_path(way_point_x, way_point_y, n_course_point);
  const new_path = np.vstack([rix, riy]).T;
  // new_path[new_path < 0] = 0;
  return new_path;
}

export {
  interpolate_b_spline_path,
  interpolate_path
};

class AStarPlanner {
    constructor(ox, oy, resolution, rr) {
      this.resolution = resolution;
      this.rr = rr;
      this.min_x = 0;
      this.min_y = 0;
      this.max_x = 0;
      this.max_y = 0;
      this.obstacle_map = null;
      this.x_width = 0;
      this.y_width = 0;
      this.motion = this.get_motion_model();
      this.calc_obstacle_map(ox, oy);
    }}
  
class Node {
      constructor(x, y, cost, parent_index) {
        this.x = x;  // index of grid
        this.y = y;  // index of grid
        this.cost = cost;
        this.parent_index = parent_index;
      }
  
      toString() {
        return `${this.x},${this.y},${this.cost},${this.parent_index}`;
      }
    }
  
    planning(sx, sy, gx, gy) {
      const start_node = new this.Node(this.calc_xy_index(sx, this.min_x),
        this.calc_xy_index(sy, this.min_y), 0.0, -1);
      const goal_node = new this.Node(this.calc_xy_index(gx, this.min_x),
        this.calc_xy_index(gy, this.min_y), 0.0, -1);
  
      const open_set = {};
      const closed_set = {};
      open_set[this.calc_grid_index(start_node)] = start_node;
  
      while (true) {
        if (Object.keys(open_set).length === 0) {
          console.log("Open set is empty..");
          break;
        }
  
        const c_id = Object.keys(open_set).reduce((a, b) =>
          open_set[a].cost + this.calc_heuristic(goal_node, open_set[a]) <
          open_set[b].cost + this.calc_heuristic(goal_node, open_set[b]) ? a : b);
        const current = open_set[c_id];
  
        if (current.x === goal_node.x && current.y === goal_node.y) {
          console.log("Find goal");
          goal_node.parent_index = current.parent_index;
          goal_node.cost = current.cost;
          break;
        }
  
        delete open_set[c_id];
        closed_set[c_id] = current;
  
        for (let i = 0; i < this.motion.length; i++) {
          const node = new this.Node(current.x + this.motion[i][0],
            current.y + this.motion[i][1],
            current.cost + this.motion[i][2], c_id);
          const n_id = this.calc_grid_index(node);
  
          if (!this.verify_node(node)) {
            continue;
          }
  
          if (n_id in closed_set) {
            continue;
          }
  
          if (!(n_id in open_set)) {
            open_set[n_id] = node;
          } else {
            if (open_set[n_id].cost > node.cost) {
              open_set[n_id] = node;
            }
          }
        }
      }
  
      const [rx, ry] = this.calc_final_path(goal_node, closed_set);
      return [rx, ry];
    }
  
    calc_final_path(goal_node, closed_set) {
        const rx = [this.calc_grid_position(goal_node.x, this.min_x)];
        const ry = [this.calc_grid_position(goal_node.y, this.min_y)];
        let parent_index = goal_node.parent_index;
      
        while (parent_index !== -1) {
          const n = closed_set[parent_index];
          rx.push(this.calc_grid_position(n.x, this.min_x));
          ry.push(this.calc_grid_position(n.y, this.min_y));
          parent_index = n.parent_index;
        }
      
        return [rx, ry];
      }
      
      calc_heuristic(n1, n2) 
      {
        const w = 1.0; // weight of heuristic
        const d = w * Math.hypot(n1.x - n2.x, n1.y - n2.y);
        return d;
      }
      
      calc_grid_position(index, min_position) {
        const pos = index * this.resolution + min_position;
        return pos;
      }
      
      calc_xy_index(position, min_pos) {
        return Math.round((position - min_pos) / this.resolution);
      }
      
      calc_grid_index(node) {
        return (node.y - this.min_y) * this.x_width + (node.x - this.min_x);
      }
      
      verify_node(node) 
      {
        const px = this.calc_grid_position(node.x, this.min_x);
        const py = this.calc_grid_position(node.y, this.min_y);
      
        if (px < this.min_x) {
          return false;
        } else if (py < this.min_y) {
          return false;
        } else if (px >= this.max_x) {
          return false;
        } else if (py >= this.max_y) {
          return false;
        }
      
        // collision check
        if (this.obstacle_map[node.x][node.y]) {
          return false;
        }
      
        return true;
      }
      
      calc_obstacle_map(ox, oy) {
        this.min_x = Math.round(Math.min(...ox));
        this.min_y = Math.round(Math.min(...oy));
        this.max_x = Math.round(Math.max(...ox));
        this.max_y = Math.round(Math.max(...oy));
      
        this.x_width = Math.round((this.max_x - this.min_x) / this.resolution);
        this.y_width = Math.round((this.max_y - this.min_y) / this.resolution);
      
        // obstacle map generation
        this.obstacle_map = Array.from({ length: this.x_width }, () =>
          Array.from({ length: this.y_width }, () => false)
        );
      
        for (let ix = 0; ix < this.x_width; ix++) {
          const x = this.calc_grid_position(ix, this.min_x);
      
          for (let iy = 0; iy < this.y_width; iy++) {
            const y = this.calc_grid_position(iy, this.min_y);
      
            for (let i = 0; i < ox.length; i++) {
              const d = Math.hypot(ox[i] - x, oy[i] - y);
      
              if (d < this.rr) {
                this.obstacle_map[ix][iy] = true;
                break;
              }
            }
          }
        }
      }
      
      get_motion_model() {
        // dx, dy, cost
        const motion = [
          [1, 0, 1],
          [0, 1, 1],
          [-1, 0, 1],
          [0, -1, 1],
          [-1, -1, Math.sqrt(2)],
          [-1, 1, Math.sqrt(2)],
          [1, -1, Math.sqrt(2)],
          [1, 1, Math.sqrt(2)],
        ];
      
        return motion;
      }
  
  export default AStarPlanner;
  
  
class PathPlanning {
    constructor(obstacles) {
      this.margin = 5;
      // Scale obstacles from env margin to path planning margin
      obstacles = obstacles + [this.margin, this.margin];
      obstacles = obstacles.filter(([x, y]) => x >= 0 && y >= 0);
  
      this.obs = [
        ...Array.from({ length: 100 + this.margin }, (_, i) => [0, i]),
        ...Array.from(
          { length: 100 + 2 * this.margin },
          (_, i) => [100 + 2 * this.margin, i]
        ),
        ...Array.from({ length: 100 + this.margin }, (_, i) => [i, 0]),
        ...Array.from(
          { length: 100 + 2 * this.margin },
          (_, i) => [i, 100 + 2 * this.margin]
        ),
        ...obstacles,
      ];
  
      this.ox = this.obs.map(([x]) => Math.floor(x));
      this.oy = this.obs.map(([_, y]) => Math.floor(y));
      this.grid_size = 1;
      this.robot_radius = 4;
      this.a_star = new AStarPlanner(this.ox, this.oy, this.grid_size, this.robot_radius);
    }
  
    plan_path(sx, sy, gx, gy) {
      const rx = [];
      const ry = [];
  
      const [pathX, pathY] = this.a_star.planning(
        sx + this.margin,
        sy + this.margin,
        gx + this.margin,
        gy + this.margin
      );
  
      for (let i = 0; i < pathX.length; i++) {
        rx.push(pathX[i] - this.margin + 0.5);
        ry.push(pathY[i] - this.margin + 0.5);
      }
  
      const path = rx.map((_, i) => [rx[i], ry[i]]);
      return path.reverse();
    }
  }
  
class ParkPathPlanning {
    constructor(obstacles) {
      this.margin = 5;
      // Scale obstacles from env margin to path planning margin
      obstacles = obstacles + [this.margin, this.margin];
      obstacles = obstacles.filter(([x, y]) => x >= 0 && y >= 0);
  
      this.obs = [
        ...Array.from({ length: 100 + this.margin }, (_, i) => [0, i]),
        ...Array.from(
          { length: 100 + 2 * this.margin },
          (_, i) => [100 + 2 * this.margin, i]
        ),
        ...Array.from({ length: 100 + this.margin }, (_, i) => [i, 0]),
        ...Array.from(
          { length: 100 + 2 * this.margin },
          (_, i) => [i, 100 + 2 * this.margin]
        ),
        ...obstacles,
      ];
  
      this.ox = this.obs.map(([x]) => Math.floor(x));
      this.oy = this.obs.map(([_, y]) => Math.floor(y));
      this.grid_size = 1;
      this.robot_radius = 4;
      this.a_star = new AStarPlanner(this.ox, this.oy, this.grid_size, this.robot_radius);
    }
  
    generate_park_scenario(sx, sy, gx, gy) {
      const rx = [];
      const ry = [];
  
      const [pathX, pathY] = this.a_star.planning(
        sx + this.margin,
        sy + this.margin,
        gx + this.margin,
        gy + this.margin
      );
  
      for (let i = 0; i < pathX.length; i++) {
        rx.push(pathX[i] - this.margin + 0.5);
        ry.push(pathY[i] - this.margin + 0.5);
      }
  
      const path = rx.map((_, i) => [rx[i], ry[i]]);
      path.reverse();
      const computed_angle = angle_of_line(path[path.length - 10][0], path[path.length - 10][1], path[path.length - 1][0], path[path.length - 1][1]);
  
      const s = 4;
      const l = 8;
      const d = 2;
      const w = 4;
  
      let x_ensure1, y_ensure1;
      let ensure_path1, ensure_path2;
      let park_path;
  
      if (-Math.atan2(0, -1) < computed_angle && computed_angle <= Math.atan2(-1, 0)) {
        x_ensure2 = gx;
        y_ensure2 = gy;
        x_ensure1 = x_ensure2 + d + w;
        y_ensure1 = y_ensure2 - l - s;
        ensure_path1 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure1, y_ensure1 - 3 + i * 0.25]).reverse();
        ensure_path2 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure2, y_ensure2 + i * 0.25]).reverse();
        park_path = this.plan_park_down_right(x_ensure2, y_ensure2);
      } else if (Math.atan2(-1, 0) <= computed_angle && computed_angle <= Math.atan2(0, 1)) {
        x_ensure2 = gx;
        y_ensure2 = gy;
        x_ensure1 = x_ensure2 - d - w;
        y_ensure1 = y_ensure2 - l - s;
        ensure_path1 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure1, y_ensure1 - 3 + i * 0.25]).reverse();
        ensure_path2 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure2, y_ensure2 + i * 0.25]).reverse();
        park_path = this.plan_park_down_left(x_ensure2, y_ensure2);
      } else if (Math.atan2(0, 1) < computed_angle && computed_angle <= Math.atan2(1, 0)) {
        x_ensure2 = gx;
        y_ensure2 = gy;
        x_ensure1 = x_ensure2 - d - w;
        y_ensure1 = y_ensure2 + l + s;
        ensure_path1 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure1, y_ensure1 + i * 0.25]);
        ensure_path2 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure2, y_ensure2 - 3 + i * 0.25]);
        park_path = this.plan_park_up_left(x_ensure2, y_ensure2);
      } else if (Math.atan2(1, 0) < computed_angle && computed_angle <= Math.atan2(0, -1)) {
        x_ensure2 = gx;
        y_ensure2 = gy;
        x_ensure1 = x_ensure2 + d + w;
        y_ensure1 = y_ensure2 + l + s;
        ensure_path1 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure1, y_ensure1 + i * 0.25]);
        ensure_path2 = Array.from({ length: 3 / 0.25 }, (_, i) => [x_ensure2, y_ensure2 - 3 + i * 0.25]);
        park_path = this.plan_park_up_right(x_ensure2, y_ensure2);
      }
  
      return [[x_ensure1, y_ensure1], park_path, ensure_path1, ensure_path2];
    }
    plan_park_up_right(x1, y1) {
        const s = 4;
        const l = 8;
        const d = 2;
        const w = 4;
      
        const x0 = x1 + d + w;
        const y0 = y1 + l + s;
      
        let curve_x = [];
        let curve_y = [];
      
        let y = Array.from({ length: y0 - y1 + 1 }, (_, i) => y1 + i);
        const circle_fun = y.map((y) => 6.9 ** 2 - (y - y0) ** 2);
        let x = circle_fun
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x0 - 6.9);
        y = y.filter((_, i) => circle_fun[i] >= 0);
        const choices = x.map((val) => val > x0 - 6.9 / 2);
        x = x.filter((_, i) => choices[i]);
        y = y.filter((_, i) => choices[i]);
        curve_x = curve_x.concat(x.reverse());
        curve_y = curve_y.concat(y.reverse());
      
        y = Array.from({ length: y0 - y1 + 1 }, (_, i) => y1 + i);
        const circle_fun2 = y.map((y) => 6.9 ** 2 - (y - y1) ** 2);
        x = circle_fun2
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x1 + 6.9);
        y = y.filter((_, i) => circle_fun2[i] >= 0);
        x = x.map((val) => x0 - 2 * (val - (x1 + 6.9)));
        const choices2 = x.map((val) => val < x1 + 6.9 / 2);
        x = x.filter((_, i) => choices2[i]);
        y = y.filter((_, i) => choices2[i]);
        curve_x = curve_x.concat(x.reverse());
        curve_y = curve_y.concat(y.reverse());
      
        const park_path = curve_x.map((_, i) => [curve_x[i], curve_y[i]]);
        return park_path;
      }
      
      plan_park_up_left(x1, y1) {
        const s = 4;
        const l = 8;
        const d = 2;
        const w = 4;
      
        const x0 = x1 - d - w;
        const y0 = y1 + l + s;
      
        let curve_x = [];
        let curve_y = [];
      
        let y = Array.from({ length: y0 - y1 + 1 }, (_, i) => y1 + i);
        const circle_fun = y.map((y) => 6.9 ** 2 - (y - y0) ** 2);
        let x = circle_fun
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x0 + 6.9);
        y = y.filter((_, i) => circle_fun[i] >= 0);
        x = x.map((val) => x0 - 2 * (val - (x0 + 6.9)));
        const choices = x.map((val) => val < x0 + 6.9 / 2);
        x = x.filter((_, i) => choices[i]);
        y = y.filter((_, i) => choices[i]);
        curve_x = curve_x.concat(x.reverse());
        curve_y = curve_y.concat(y.reverse());
      
        y = Array.from({ length: y0 - y1 + 1 }, (_, i) => y1 + i);
        const circle_fun2 = y.map((y) => 6.9 ** 2 - (y - y1) ** 2);
        x = circle_fun2
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x1 - 6.9);
        y = y.filter((_, i) => circle_fun2[i] >= 0);
        const choices2 = x.map((val) => val > x1 - 6.9 / 2);
        x = x.filter((_, i) => choices2[i]);
        y = y.filter((_, i) => choices2[i]);
        curve_x = curve_x.concat(x.reverse());
        curve_y = curve_y.concat(y.reverse());
      
        const park_path = curve_x.map((_, i) => [curve_x[i], curve_y[i]]);
        return park_path;
      }
      plan_park_down_right(x1, y1) {
        const s = 4;
        const l = 8;
        const d = 2;
        const w = 4;
      
        const x0 = x1 + d + w;
        const y0 = y1 - l - s;
      
        let curve_x = [];
        let curve_y = [];
      
        let y = Array.from({ length: y1 - y0 + 1 }, (_, i) => y0 + i);
        const circle_fun = y.map((y) => 6.9 ** 2 - (y - y0) ** 2);
        let x = circle_fun
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x0 - 6.9);
        y = y.filter((_, i) => circle_fun[i] >= 0);
        const choices = x.map((val) => val > x0 - 6.9 / 2);
        x = x.filter((_, i) => choices[i]);
        y = y.filter((_, i) => choices[i]);
      
        curve_x = curve_x.concat(x);
        curve_y = curve_y.concat(y);
      
        y = Array.from({ length: y1 - y0 + 1 }, (_, i) => y0 + i);
        const circle_fun2 = y.map((y) => 6.9 ** 2 - (y - y1) ** 2);
        x = circle_fun2
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x1 + 6.9);
        x = x.map((val) => x0 - 2 * (val - (x1 + 6.9)));
        y = y.filter((_, i) => circle_fun2[i] >= 0);
        const choices2 = x.map((val) => val < x1 + 6.9 / 2);
        x = x.filter((_, i) => choices2[i]);
        y = y.filter((_, i) => choices2[i]);
      
        curve_x = curve_x.concat(x);
        curve_y = curve_y.concat(y);
      
        const park_path = curve_x.map((_, i) => [curve_x[i], curve_y[i]]);
        return park_path;
      }
      
      plan_park_down_left(x1, y1) {
        const s = 4;
        const l = 8;
        const d = 2;
        const w = 4;
      
        const x0 = x1 - d - w;
        const y0 = y1 - l - s;
      
        let curve_x = [];
        let curve_y = [];
      
        let y = Array.from({ length: y1 - y0 + 1 }, (_, i) => y0 + i);
        const circle_fun = y.map((y) => 6.9 ** 2 - (y - y0) ** 2);
        let x = circle_fun
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x0 + 6.9);
        y = y.filter((_, i) => circle_fun[i] >= 0);
        x = x.map((val) => x0 - 2 * (val - (x0 + 6.9)));
        const choices = x.map((val) => val < x0 + 6.9 / 2);
        x = x.filter((_, i) => choices[i]);
        y = y.filter((_, i) => choices[i]);
      
        curve_x = curve_x.concat(x);
        curve_y = curve_y.concat(y);
      
        y = Array.from({ length: y1 - y0 + 1 }, (_, i) => y0 + i);
        const circle_fun2 = y.map((y) => 6.9 ** 2 - (y - y1) ** 2);
        x = circle_fun2
          .filter((val) => val >= 0)
          .map((val) => Math.sqrt(val) + x1 - 6.9);
        y = y.filter((_, i) => circle_fun2[i] >= 0);
        const choices2 = x.map((val) => val > x1 - 6.9 / 2);
        x = x.filter((_, i) => choices2[i]);
        y = y.filter((_, i) => choices2[i]);
      
        curve_x = curve_x.concat(x);
        curve_y = curve_y.concat(y);
      
        const park_path = curve_x.map((_, i) => [curve_x[i], curve_y[i]]);
        return park_path;
      }
        
  }
  
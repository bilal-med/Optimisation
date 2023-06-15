class CarDynamics {
    constructor(x0, y0, v0, psi0, length, dt) {
      this.dt = dt; // sampling time
      this.L = length; // vehicle length
      this.x = x0;
      this.y = y0;
      this.v = v0;
      this.psi = psi0;
      this.state = [[this.x, this.y, this.v, this.psi]];
    }
  
    move(accelerate, delta) {
      const xDot = this.v * Math.cos(this.psi);
      const yDot = this.v * Math.sin(this.psi);
      const vDot = accelerate;
      const psiDot = (this.v * Math.tan(delta)) / this.L;
      return [[xDot, yDot, vDot, psiDot]];
    }
  
    updateState(stateDot) {
      this.state = this.state.map((s, i) => s.map((v, j) => v + this.dt * stateDot[i][j]));
      this.x = this.state[0][0];
      this.y = this.state[0][1];
      this.v = this.state[0][2];
      this.psi = this.state[0][3];
    }
  }
  
  class MpcController {
    constructor() {
      this.horiz = null;
      this.R = [[0.01, 0], [0, 0.01]]; // input cost matrix
      this.Rd = [[0.01, 0], [0, 1.0]]; // input difference cost matrix
      this.Q = [[1.0, 0], [0, 1.0]]; // state cost matrix
      this.Qf = this.Q; // state final matrix
    }
  
    mpcCost(uK, myCar, points) {
      const mpcCar = Object.assign({}, myCar);
      const uKReshaped = [];
      for (let i = 0; i < this.horiz; i++) {
        uKReshaped.push(uK.slice(i * 2, (i + 1) * 2));
      }
      const zK = new Array(2).fill(0).map(() => new Array(this.horiz + 1).fill(0));
  
      const desiredState = points;
      let cost = 0.0;
  
      for (let i = 0; i < this.horiz; i++) {
        const stateDot = mpcCar.move(uKReshaped[i][0], uKReshaped[i][1]);
        mpcCar.updateState(stateDot);
  
        zK[0][i] = mpcCar.x;
        zK[1][i] = mpcCar.y;
        cost += this.R[0][0] * uKReshaped[i][0] ** 2 + this.R[1][1] * uKReshaped[i][1] ** 2;
        cost += this.Q[0][0] * (desiredState[i][0] - zK[0][i]) ** 2 + this.Q[1][1] * (desiredState[i][1] - zK[1][i]) ** 2;
        if (i < this.horiz - 1) {
          cost += this.Rd[0][0] * (uKReshaped[i + 1][0] - uKReshaped[i][0]) ** 2 + this.Rd[1][1] * (uKReshaped[i + 1][1] - uKReshaped[i][1]) ** 2;
        }
      }
  
      return cost;
    }
  
    optimize(myCar, points) {
      this.horiz = points.length;
      const bnd = new Array(this.horiz).fill([-5, 5], [Math.deg2rad(-60), Math.deg2rad(60)]).flat();
      const result = minimize(this.mpcCost, myCar, points, new Array(2 * this.horiz).fill(0), 'SLSQP', bnd);
      return [result.x[0], result.x[1]];
    }
  }

  

class LinearMpcController {
    constructor() {
      this.horiz = null;
      this.R = [[0.01, 0], [0, 0.01]]; // input cost matrix
      this.Rd = [[0.01, 0], [0, 1.0]]; // input difference cost matrix
      this.Q = [[1.0, 0], [0, 1.0]]; // state cost matrix
      this.Qf = this.Q; // state final matrix
      this.dt = 0.2;
      this.L = 4;
    }
  
    makeModel(v, psi, delta) {
      const A = [
        [1, 0, this.dt * Math.cos(psi), -this.dt * v * Math.sin(psi)],
        [0, 1, this.dt * Math.sin(psi), this.dt * v * Math.cos(psi)],
        [0, 0, 1, 0],
        [0, 0, (this.dt * Math.tan(delta)) / this.L, 1],
      ];
      const B = [
        [0, 0],
        [0, 0],
        [this.dt, 0],
        [0, (this.dt * v) / (this.L * Math.cos(delta) ** 2)],
      ];
      const C = [
        [this.dt * v * Math.sin(psi) * psi],
        [-this.dt * v * Math.cos(psi) * psi],
        [0],
        [-this.dt * (v * delta) / (this.L * Math.cos(delta) ** 2)],
      ];
      return [A, B, C];
    }
  
    mpcCost(uK, myCar, points) {
      const uKReshaped = [];
      for (let i = 0; i < this.horiz; i++) {
        uKReshaped.push(uK.slice(i * 2, (i + 1) * 2));
      }
      const zK = new Array(2).fill(0).map(() => new Array(this.horiz + 1).fill(0));
      const desiredState = points;
      let cost = 0.0;
      const oldState = [[myCar.x], [myCar.y], [myCar.v], [myCar.psi]];
  
      for (let i = 0; i < this.horiz; i++) {
        const delta = uKReshaped[i][1];
        const [A, B, C] = this.makeModel(myCar.v, myCar.psi, delta);
        const newState = math.add(math.multiply(A, oldState), math.add(math.multiply(B, uKReshaped[i]), C));
  
        zK[0][i] = newState[0][0];
        zK[1][i] = newState[1][0];
        cost += this.R[0][0] * uKReshaped[i][0] ** 2 + this.R[1][1] * uKReshaped[i][1] ** 2;
        cost += this.Q[0][0] * (desiredState[i][0] - zK[0][i]) ** 2 + this.Q[1][1] * (desiredState[i][1] - zK[1][i]) ** 2;
        if (i < this.horiz - 1) {
          cost += this.Rd[0][0] * (uKReshaped[i + 1][0] - uKReshaped[i][0]) ** 2 + this.Rd[1][1] * (uKReshaped[i + 1][1] - uKReshaped[i][1]) ** 2;
        }
  
        oldState = newState;
      }
  
      return cost;
    }
  
    optimize(myCar, points) {
      this.horiz = points.length;
      const bnd = new Array(this.horiz).fill([-5, 5], [Math.deg2rad(-60), Math.deg2rad(60)]).flat();
      const result = minimize(this.mpcCost, myCar, points, new Array(2 * this.horiz).fill(0), 'SLSQP', bnd);
      return [result.x[0], result.x[1]];
    }
  }
  
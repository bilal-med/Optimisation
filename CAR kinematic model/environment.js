class Environment {
    constructor(obstacles) {
      this.margin = 5;
      this.car_length = 80;
      this.car_width = 40;
      this.wheel_length = 15;
      this.wheel_width = 7;
      this.wheel_positions = [
        [25, 15],
        [25, -15],
        [-25, 15],
        [-25, -15],
      ];
  
      this.color = [0, 0, 255];
      this.wheel_color = [20, 20, 20];
  
      this.car_struct = [
        [this.car_length / 2, this.car_width / 2],
        [this.car_length / 2, -this.car_width / 2],
        [-this.car_length / 2, -this.car_width / 2],
        [-this.car_length / 2, this.car_width / 2],
      ];
  
      this.wheel_struct = [
        [this.wheel_length / 2, this.wheel_width / 2],
        [this.wheel_length / 2, -this.wheel_width / 2],
        [-this.wheel_length / 2, -this.wheel_width / 2],
        [-this.wheel_length / 2, this.wheel_width / 2],
      ];
  
      this.background = Array.from(
        { length: 1000 + 20 * this.margin },
        () => Array(1000 + 20 * this.margin).fill(1)
      );
      for (let i = 1; i < 1000 + 20 * this.margin; i += 10) {
        this.background[i].fill([200, 200, 200]);
        for (let j = 10; j < 1000 + 20 * this.margin; j += 10) {
          this.background[i][j] = [200, 200, 200];
        }
      }
      this.placeObstacles(obstacles);
    }
  
    placeObstacles(obs) {
      const obstacles = [
        ...Array.from({ length: 100 + 2 * this.margin }, (_, i) => [0, i]),
        ...Array.from({ length: 100 + 2 * this.margin }, (_, i) => [
          100 + 2 * this.margin - 1,
          i,
        ]),
        ...Array.from({ length: 100 + 2 * this.margin }, (_, i) => [i, 0]),
        ...Array.from({ length: 100 + 2 * this.margin }, (_, i) => [
          i,
          100 + 2 * this.margin - 1,
        ]),
        ...obs.map(([x, y]) => [x + this.margin, y + this.margin]),
      ];
  
      obstacles.forEach(([x, y]) => {
        for (let i = y * 10; i < y * 10 + 10; i++) {
          for (let j = x * 10; j < x * 10 + 10; j++) {
            this.background[i][j] = 0;
          }
        }
      });
    }
    drawPath(path) {
        const color = Array.from({ length: 3 }, () => Math.floor(Math.random() * 150) / 255);
        path = path.map(([x, y]) => [x * 10, y * 10]);
        for (const [x, y] of path) {
          for (let i = y + 10 * this.margin; i < y + 10 * this.margin + 3; i++) {
            for (let j = x + 10 * this.margin; j < x + 10 * this.margin + 3; j++) {
              this.background[i][j] = color;
            }
          }
        }
      }
      
      rotateCar(pts, angle = 0) {
        const R = [
          [Math.cos(angle), -Math.sin(angle)],
          [Math.sin(angle), Math.cos(angle)],
        ];
        return pts.map(([x, y]) => [
          Math.round(R[0][0] * x + R[0][1] * y),
          Math.round(R[1][0] * x + R[1][1] * y),
        ]);
      }
      render(x, y, psi, delta) {
        x = 10 * x;
        y = 10 * y;
        const rotatedStruct = this.rotateCar(this.carStruct, psi);
        rotatedStruct.forEach(([px, py]) => {
          const xCoord = x + px + 10 * this.margin;
          const yCoord = y + py + 10 * this.margin;
          for (let i = yCoord; i < yCoord + 4; i++) {
            for (let j = xCoord; j < xCoord + 4; j++) {
              this.background[i][j] = this.color;
            }
          }
        });
      
        const rotatedWheelCenter = this.rotateCar(this.wheelPositions, psi);
        this.wheelPositions.forEach((wheelPos, i) => {
          const wheel = i < 2 ? this.rotateCar(this.wheelStruct, delta + psi) : this.rotateCar(this.wheelStruct, psi);
          wheel.forEach(([px, py]) => {
            const xCoord = x + wheelPos[0] + px + 10 * this.margin;
            const yCoord = y + wheelPos[1] + py + 10 * this.margin;
            for (let i = yCoord; i < yCoord + 4; i++) {
              for (let j = xCoord; j < xCoord + 4; j++) {
                this.background[i][j] = this.wheelColor;
              }
            }
          });
        });
      
        const gel = [
          ...Array.from({ length: 16 }, (_, i) => [Math.floor(Math.random() * 21) - 50, Math.floor(Math.random() * 31) - 20]),
          ...Array.from({ length: 16 }, (_, i) => [Math.floor(Math.random() * 21) - 50, Math.floor(Math.random() * 31) + 10]),
        ];
        const rotatedGel = this.rotateCar(gel, psi);
        rotatedGel.forEach(([px, py]) => {
          const xCoord = x + px + 10 * this.margin;
          const yCoord = y + py + 10 * this.margin;
          for (let i = yCoord; i < yCoord + 4; i++) {
            for (let j = xCoord; j < xCoord + 4; j++) {
              this.background[i][j] = [60 / 255, 60 / 255, 135 / 255];
            }
          }
        });
      
        const newCenter = [x + 10 * this.margin, y + 10 * this.margin];
        this.background[newCenter[1]][newCenter[0]] = [255 / 255, 150 / 255, 100 / 255];
      
        const rendered = this.background.map((row) => row.map((color) => [...color]));
        return rendered;
      }     
  }
  
  class Parking1 {
    constructor(carPos) {
      this.carObstacle = this.makeCar();
      this.walls = [...Array(90)].map((_, i) => [70, i - 5])
        .concat([...Array(95)].map((_, i) => [30, i + 10]))
        .concat([...Array(6)].map((_, i) => [i + 30, 10]))
        .concat([...Array(6)].map((_, i) => [i + 70, 90]));
      this.obs = [...Array(100)].map((_, i) => i);
      this.cars = {
        1: [[35, 20]],
        2: [[65, 20]],
        3: [[75, 20]],
        4: [[95, 20]],
        5: [[35, 50]],
        6: [[65, 50]],
        7: [[75, 50]],
        8: [[95, 50]],
        9: [[35, 42]],
        10: [[65, 42]],
        11: [[75, 42]],
        12: [[95, 42]],
        13: [[35, 32]],
        14: [[65, 32]],
        15: [[75, 32]],
        16: [[95, 32]],
        17: [[35, 44]],
        18: [[65, 44]],
        19: [[75, 44]],
        20: [[95, 44]],
        21: [[35, 56]],
        22: [[65, 56]],
        23: [[75, 56]],
        24: [[95, 56]],
        25: [[35, 68]],
        26: [[65, 68]],
        27: [[75, 68]],
        28: [[95, 68]],
        29: [[35, 80]],
        30: [[65, 80]],
        31: [[75, 80]],
        32: [[95, 80]]
      };
      this.end = this.cars[carPos][0];
      delete this.cars[carPos];
    }
  
    generateObstacles() {
      Object.values(this.cars).forEach(car => {
        const obstacle = car.map(([x, y]) => this.carObstacle.map(([dx, dy]) => [x + dx, y + dy])).flat();
        this.obs.push(...obstacle);
      });
      return [this.end, this.obs];
    }
  
    makeCar() {
      const carObstacle = [];
      for (let i = -2; i < 2; i++) {
        for (let j = -4; j < 4; j++) {
          carObstacle.push([i, j]);
        }
      }
      return carObstacle;
    }
  }
  
function angle_of_line(x1, y1, x2, y2) {
    return Math.atan2(y2 - y1, x2 - x1);
}

function make_square(x, y, width) {
    var square = [];
    for (var i = y - Math.floor(width / 2); i <= y + Math.floor(width / 2); i++) {
        square.push([x - Math.floor(width / 2), i]);
    }
    for (var i = y - Math.floor(width / 2); i <= y + Math.floor(width / 2); i++) {
        square.push([x + Math.floor(width / 2), i]);
    }
    for (var i = x - Math.floor(width / 2); i <= x + Math.floor(width / 2); i++) {
        square.push([i, y - Math.floor(width / 2)]);
    }
    for (var i = x - Math.floor(width / 2); i <= x + Math.floor(width / 2); i++) {
        square.push([i, y + Math.floor(width / 2)]);
    }
    return square;
}
class DataLogger {
    constructor() {
        this.path = [];
        this.car_state = [];
        this.u = [];
    }

    log(point, my_car, acc, delta) {
        this.path.push(point);
        this.car_state.push([my_car.x, my_car.y, my_car.v, my_car.psi]);
        this.u.push([acc, delta]);
    }

    save_data() {
        const fs = require('fs');
        const path = require('path');
        const mkdirp = require('mkdirp');
        const fontManager = require('matplotlib-font-manager');

        mkdirp.sync('log results');
        const t = [];
        for (let i = 0; i < this.path.length / 5; i += 0.2) {
            t.push(i);
        }
        this.path = [...this.path];
        this.car_state = [...this.car_state];
        this.u = [...this.u];
        const font = new fontManager.FontProperties({
            family: 'Times New Roman',
            weight: 'bold',
            style: 'normal',
            size: 20
        });

        // plot x
        plt.figure(figsize = [12, 8]);
        plt.plot(t, this.path.map(p => p[0]), { color: 'b', linewidth: 5 });
        plt.plot(t, this.car_state.map(state => state[0]), { color: 'r', linewidth: 4 });
        plt.title('car\'s x in time', { fontsize: 20 });
        plt.xlabel('time (s)', { fontsize: 20 });
        plt.ylabel('x (m)', { fontsize: 20 });
        plt.grid();
        plt.legend(['reference', 'car\'s x'], { prop: font });
        plt.savefig('log results/x.png');

        // plot y
        plt.figure(figsize = [12, 8]);
        plt.plot(t, this.path.map(p => p[1]), { color: 'b', linewidth: 5 });
        plt.plot(t, this.car_state.map(state => state[1]), { color: 'r', linewidth: 4 });
        plt.title('car\'s y in time', { fontsize: 20 });
        plt.xlabel('time (s)', { fontsize: 20 });
        plt.ylabel('y (m)', { fontsize: 20 });
        plt.grid();
        plt.legend(['reference', 'car\'s y'], { prop: font });
        plt.savefig('log results/y.png');

        // plot v
        plt.figure(figsize = [12, 8]);
        plt.plot(t, this.car_state.map(state => state[2]), { color: 'r', linewidth: 4 });
        plt.title('car\'s speed in time', { fontsize: 20 });
        plt.xlabel('time (s)', { fontsize: 20 });
        plt.ylabel('v (m/s)', { fontsize: 20 });
        plt.grid();
        plt.legend(['car speed (m/s)'], { prop: font });
        plt.savefig('log results/v.png');

        // plot psi
        plt.figure(figsize = [12, 8]);
        plt.plot(t, this.car_state.map(state => Math.rad2deg(state[3])), { color: 'r', linewidth: 4 });
        plt.title('car\'s angle in time', { fontsize: 20 });
        plt.xlabel('time (s)', { fontsize: 20 });
        plt.ylabel('psi (degree)', { fontsize: 20 });
        plt.grid();
        plt.legend(['car angle (degree)'], { prop: font });
        plt.savefig('log results/psi.png');

        // plot position
        plt.figure(figsize = [12, 12]);
        plt.plot(this.path.map(p => p[0]), this.path.map(p => p[1]), { color: 'b', linewidth: 5 });
        plt.plot(this.car_state.map(state => state[0]), this.car_state.map(state => state[1]), { color: 'r', linewidth: 4 });
        plt.title('car\'s position in time', { fontsize: 20 });
        plt.xlabel('x (m)', { fontsize: 20 });
        plt.ylabel('y (m)', { fontsize: 20 });
        plt.grid();
        plt.legend(['reference', 'car\'s position'], { prop: font });
        plt.savefig('log results/position.png');

        // plot accelerate
        plt.figure(figsize = [12, 8]);
        plt.plot(t, this.u.map(u => u[0]), { color: 'r', linewidth: 4 });
        plt.title('car\'s accelerate in time', { fontsize: 20 });
        plt.xlabel('time (s)', { fontsize: 20 });
        plt.ylabel('accelerate (m^2/s)', { fontsize: 20 });
        plt.grid();
        plt.legend(['car accelerate (m^2/s)'], { prop: font });
        plt.savefig('log results/accelerate.png');

        // plot delta
        plt.figure(figsize = [12, 8]);
        plt.plot(t, this.u.map(u => Math.rad2deg(u[1])), { color: 'r', linewidth: 4 });
        plt.title('car\'s steer in time', { fontsize: 20 });
        plt.xlabel('time (s)', { fontsize: 20 });
        plt.ylabel('steer (degree)', { fontsize: 20 });
        plt.grid();
        plt.legend(['car steer (degree)'], { prop: font });
        plt.savefig('log results/steer.png');

        console.log('all data saved on log results ...');
    }
}

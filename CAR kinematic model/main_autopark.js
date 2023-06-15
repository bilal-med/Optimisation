const cv2 = require('opencv4nodejs');
const np = require('numpy');
const sleep = require('sleep');

const Environment = require('./environment');
const Parking1 = require('./parking1');
const PathPlanning = require('./pathplanning');
const ParkPathPlanning = require('./parkpathplanning');
const Car_Dynamics = require('./car_dynamics');
const MPC_Controller = require('./mpc_controller');
const Linear_MPC_Controller = require('./linear_mpc_controller');
const { angle_of_line, make_square, DataLogger } = require('./utils');

const args = {
    x_start: 0,
    y_start: 90,
    psi_start: 0,
    x_end: 90,
    y_end: 80,
    parking: 1
};

const logger = new DataLogger();

//########################## default variables ################################################
const start = np.array([args.x_start, args.y_start]);
const psi_start = np.deg2rad(0);
let end = np.array([args.x_end, args.y_end]);
//#############################################################################################

// environment margin  : 5
// pathplanning margin : 5

//########################## defining obstacles ###############################################
const parking1 = new Parking1(args.parking);
end = parking1.generate_obstacles();

// add squares
// const square1 = make_square(10, 65, 20);
// const square2 = make_square(15, 30, 20);
// const square3 = make_square(50, 50, 10);
// const obs = np.vstack([obs, square1, square2, square3]);

// Rahneshan logo
const start = np.array([50, 95]);
const end = np.array([35, 20]);
// const rah = np.flip(cv2.imread('READ_ME/rahneshan_obstacle.png', 0), axis=0);
// const obs = np.vstack([np.where(rah < 100)[1], np.where(rah < 100)[0]]).T;

// const new_obs = np.array([[78, 78], [79, 79], [78, 79]]);
// const obs = np.vstack([obs, new_obs]);
//#############################################################################################

//########################### initialization ##################################################
const env = new Environment(obs);
const my_car = new Car_Dynamics(start[0], start[1], 0, np.deg2rad(args.psi_start), 4, 0.2);
const MPC_HORIZON = 5;
const controller = new MPC_Controller();
// const controller = new Linear_MPC_Controller();

const res = env.render(my_car.x, my_car.y, my_car.psi, 0);
cv2.imshow('environment', res);
const key = cv2.waitKey(1);
//#############################################################################################

//############################# path planning #################################################
const park_path_planner = new ParkPathPlanning(obs);
const path_planner = new PathPlanning(obs);

console.log('planning park scenario ...');
const [new_end, park_path, ensure_path1, ensure_path2] = park_path_planner.generate_park_scenario(
    parseInt(start[0]),
    parseInt(start[1]),
    parseInt(end[0]),
    parseInt(end[1])
);

console.log('routing to destination ...');
const path = path_planner.plan_path(
    parseInt(start[0]),
    parseInt(start[1]),
    parseInt(new_end[0]),
    parseInt(new_end[1])
);
const ensurePath1 = ensure_path1.reverse();
const ensurePath2 = ensure_path2.reverse();
const interpolated_path = interpolate_path(path, 5);
const interpolated_park_path = interpolate_path(park_path, 2);
const final_path = np.vstack([interpolated_path, interpolated_park_path, ensurePath2]);
env.draw_path(interpolated_path);
env.draw_path(interpolated_park_path);
//#############################################################################################

//################################## control ##################################################
console.log('driving to destination ...');
for (let i = 0; i < final_path.length; i++) {
    const point = final_path[i];
    const [acc, delta] = controller.optimize(my_car, final_path.slice(i, i + MPC_HORIZON));
    my_car.update_state(my_car.move(acc, delta));
    const res = env.render(my_car.x, my_car.y, my_car.psi, delta);
    logger.log(point, my_car, acc, delta);
    cv2.imshow('environment', res);
    const key = cv2.waitKey(1);
    if (key === 's'.charCodeAt(0)) {
        cv2.imwrite('res.png', res * 255);
    }
}

// zeroing car steer
const res = env.render(my_car.x, my_car.y, my_car.psi, 0);
logger.save_data();
cv2.imshow('environment', res);
const key = cv2.waitKey();
//#############################################################################################

cv2.destroyAllWindows();

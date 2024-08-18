#![no_std]

use core::f32::consts::PI;

use nalgebra::Vector3;

fn min(v1: f32, v2: f32) -> f32 {
    if v1 < v2 {
        return v1;
    }
    return v2;
}

fn max(v1: f32, v2: f32) -> f32 {
    if v1 > v2 {
        return v1;
    }
    return v2;
}

struct Motor {
    speed: f32,
    pos: Vector3<f32>,
}
impl Motor {
    fn new(pos: Vector3<f32>) -> Self {
        Self { speed: 0.0, pos }
    }
}

pub struct MotorSpeeds {
    front_left: Motor,
    front_right: Motor,
    rear_left: Motor,
    rear_right: Motor,
}
impl MotorSpeeds {
    pub fn new() -> Self {
        Self {
            front_left: Motor::new(Vector3::new(1.0, 0.0, -1.0)),
            front_right: Motor::new(Vector3::new(1.0, 0.0, 1.0)),
            rear_left: Motor::new(Vector3::new(-1.0, 0.0, -1.0)),
            rear_right: Motor::new(Vector3::new(-1.0, 0.0, 1.0)),
        }
    }
    pub fn set_front_left(&mut self, val: f32) {
        self.front_left.speed = min(max(val, 0.0), 1.0);
    }
    pub fn set_front_right(&mut self, val: f32) {
        self.front_right.speed = min(max(val, 0.0), 1.0);
    }
    pub fn set_rear_left(&mut self, val: f32) {
        self.rear_left.speed = min(max(val, 0.0), 1.0);
    }
    pub fn set_rear_right(&mut self, val: f32) {
        self.rear_right.speed = min(max(val, 0.0), 1.0);
    }
    pub fn get_front_left(&self) -> f32 {
        self.front_left.speed
    }
    pub fn get_front_right(&self) -> f32 {
        self.front_right.speed
    }
    pub fn get_rear_left(&self) -> f32 {
        self.rear_left.speed
    }
    pub fn get_rear_right(&self) -> f32 {
        self.rear_right.speed
    }
}

struct IMUData {
    imu_data: [IMUDataPoint; 10],
    data_idx: usize,
}
impl IMUData {
    fn new() -> Self {
        Self {
            imu_data: Default::default(),
            data_idx: 0,
        }
    }

    fn add_data_point(&mut self, data_point: IMUDataPoint) {
        self.data_idx += 1;
        self.imu_data[self.data_idx] = data_point;
    }

    fn get_data_point(&self) -> &IMUDataPoint {
        &self.imu_data[self.data_idx]
    }
}

pub struct IMUDataPoint {
    pub gyro: Vector3<f32>,
    pub accel: Vector3<f32>,
    pub time_point: f32,
}

impl Default for IMUDataPoint {
    fn default() -> Self {
        Self::new(Vector3::zeros(), Vector3::zeros(), 0.0)
    }
}
impl IMUDataPoint {
    pub fn new(gyro: Vector3<f32>, accel: Vector3<f32>, time_point: f32) -> Self {
        Self {
            gyro,
            accel,
            time_point,
        }
    }
}

pub struct TransmitterState {
    up_down: f32,
    rotate_pos_neg: f32,
    left_right: f32,
    forwar_backward: f32,
}
impl TransmitterState {
    fn validate_input(val: f32) -> f32 {
        if val > 1.0 || val < 0.0 {
            panic!("Transmitter values must be between 1 and 0");
        }
        val
    }
    pub fn new(up_down: f32, rotate_pos_neg: f32, forwar_backward: f32, left_right: f32) -> Self {
        Self {
            up_down: Self::validate_input(up_down),
            rotate_pos_neg: Self::validate_input(rotate_pos_neg),
            forwar_backward: Self::validate_input(forwar_backward),
            left_right: Self::validate_input(left_right),
        }
    }
}

fn length(vec: Vector3<f32>) -> f32 {
    f32::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)
}

fn constrain(val: f32) -> f32 {
    return min(max(val, 0.0), 1.0);
}

pub struct Controller {
    motors: MotorSpeeds,
    imu: IMUData,
}
impl Controller {
    pub fn new() -> Self {
        Self {
            motors: MotorSpeeds::new(),
            imu: IMUData::new(),
        }
    }

    fn calculate_torque(&self, desired_rotation: Vector3<f32>) -> Vector3<f32> {
        let mut dt = desired_rotation - self.imu.get_data_point().gyro;
        dt.y = 0.0;
        dt
    }

    pub fn calculate_motor_speeds(
        &mut self,
        imu_data_point: IMUDataPoint,
        transmitter_state: &TransmitterState,
    ) -> &MotorSpeeds {
        self.imu.add_data_point(imu_data_point);
        let desired_rotation = Vector3::new(
            (1.0 / 6.0) * PI * transmitter_state.left_right,
            0.0,
            (1.0 / 6.0) * PI * transmitter_state.forwar_backward,
        );

        let desiered_torque: Vector3<f32> = self.calculate_torque(desired_rotation);
        self.motors.front_left.speed = constrain(
            length(
                desiered_torque
                    - self.motors.front_left.pos * self.motors.front_left.pos.dot(&desiered_torque),
            ) + transmitter_state.up_down * 0.5
                + transmitter_state.rotate_pos_neg * 0.25,
        );
        self.motors.front_right.speed = constrain(
            length(
                desiered_torque
                    - self.motors.front_right.pos
                        * self.motors.front_right.pos.dot(&desiered_torque),
            ) + transmitter_state.up_down * 0.5
                - transmitter_state.rotate_pos_neg * 0.25,
        );
        self.motors.rear_left.speed = constrain(
            length(
                desiered_torque
                    - self.motors.rear_left.pos * self.motors.rear_left.pos.dot(&desiered_torque),
            ) + transmitter_state.up_down * 0.5
                - transmitter_state.rotate_pos_neg * 0.25,
        );
        self.motors.rear_right.speed = constrain(
            length(
                desiered_torque
                    - self.motors.rear_right.pos * self.motors.rear_right.pos.dot(&desiered_torque),
            ) + transmitter_state.up_down * 0.5
                + transmitter_state.rotate_pos_neg * 0.25,
        );
        &self.motors
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {
        assert_eq!(4, 4);
    }
}

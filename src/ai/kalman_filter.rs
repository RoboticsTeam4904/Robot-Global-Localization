use crate::map::Map2D;
use crate::utility::{KinematicState, Point, Pose};
use generic_array::ArrayLength;
use nalgebra::{
    base::allocator::Allocator, DefaultAllocator, DimName, DimNameAdd, DimNameMul, DimSub,
    MatrixMN, MatrixN, U1, U13, U6,
};
use std::sync::Arc;
use typenum::operator_aliases::Prod;
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub alpha: f64,
    pub beta: f64,
    pub kappa: f64,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            alpha: 1e-5,
            beta: 2.,
            kappa: 0.,
        }
    }
}
type LocalizationStateD = U6;
type LocalizationSensorD = U6;
type LocalizationStateDx2 = U13;
pub trait KalmanFilter<StateD, SensorD, StateDx2>
where
    StateD: DimName + DimNameMul<StateD> + DimNameAdd<StateD> + DimSub<U1>,
    StateDx2: DimName + DimNameMul<StateDx2> + DimNameAdd<StateDx2>,
    SensorD: DimName + DimNameMul<SensorD> + DimNameAdd<SensorD>,
    StateD::Value: std::ops::Mul<SensorD::Value>,
    Prod<StateD::Value, SensorD::Value>: ArrayLength<f64>,
    DefaultAllocator: Allocator<f64, StateD, SensorD>
        + Allocator<f64, SensorD, StateD>
        + Allocator<f64, U1, StateD>
        + Allocator<f64, U1, SensorD>
        + Allocator<f64, SensorD, U1>
        + Allocator<f64, StateD, StateD>
        + Allocator<f64, SensorD, SensorD>
        + Allocator<f64, StateD>
        + Allocator<f64, <StateD as nalgebra::DimSub<nalgebra::U1>>::Output>
        + Allocator<f64, StateDx2, StateD>
        + Allocator<f64, StateD, StateDx2>
        + Allocator<f64, StateDx2, SensorD>
        + Allocator<f64, SensorD, StateDx2>,
{
    fn new(
        covariance_matrix: MatrixMN<f64, StateD, StateD>,
        init_state: MatrixMN<f64, U1, StateD>,
        config: Config,
    ) -> Self;
    fn control_sigma_matrix(&self) -> MatrixMN<f64, StateDx2, StateD>;
    fn set_control_sigma_matrix(&mut self, control_sigma_matrix: MatrixMN<f64, StateDx2, StateD>);
    fn sensor_sigma_matrix(&self) -> MatrixMN<f64, StateDx2, SensorD>;
    fn set_sensor_sigma_matrix(&mut self, sensor_sigma_matrix: MatrixMN<f64, StateDx2, SensorD>);
    fn config(&self) -> Config;
    fn set_config(&mut self, config: Config);
    fn known_state(&self) -> MatrixMN<f64, U1, StateD>;
    fn set_known_state(&mut self, known_state: MatrixMN<f64, U1, StateD>);
    fn covariance_matrix(&self) -> MatrixMN<f64, StateD, StateD>;
    fn set_covariance_matrix(&mut self, covariance_matrix: MatrixMN<f64, StateD, StateD>);
    fn gen_sigma_matrix(&mut self) {
        let mut rows: Vec<MatrixMN<f64, U1, StateD>> = Vec::new();
        rows.push(self.known_state());
        let known_state = self.known_state();
        let config = self.config();
        let dim = self.control_sigma_matrix().ncols();
        let lambda = (config.alpha.powi(2)) * (dim as f64 + config.kappa) - dim as f64;
        let eigendecomp = (self.covariance_matrix() * (dim as f64 + lambda)).symmetric_eigen();
        let mut diagonalization = eigendecomp.eigenvalues.clone();
        diagonalization.apply(|e| e.max(0.).sqrt());
        let square_root_cov = eigendecomp.eigenvectors.clone()
            * MatrixN::<f64, StateD>::from_diagonal(&diagonalization)
            * eigendecomp.eigenvectors.try_inverse().unwrap();
        for row in square_root_cov.row_iter() {
            rows.push(known_state.clone() + row.clone());
        }
        for row in square_root_cov.row_iter() {
            rows.push(known_state.clone() - row.clone());
        }

        self.set_control_sigma_matrix(MatrixMN::<f64, StateDx2, StateD>::from_rows(
            rows.as_slice(),
        ));
    }
    fn control_update(
        &self,
        row: &[f64],
        time: f64,
        control_input: &Vec<f64>,
        map: &Arc<Map2D>,
    ) -> MatrixMN<f64, U1, StateD>;
    fn prediction_update(
        &mut self,
        time: f64,
        control_input: Vec<f64>,
        q: MatrixMN<f64, StateD, StateD>,
        map: &Arc<Map2D>,
    ) {
        self.gen_sigma_matrix();
        let sigma_elements: Vec<f64> = self
            .control_sigma_matrix()
            .transpose()
            .iter()
            .map(|elem| elem.clone())
            .collect();
        let dim = self.control_sigma_matrix().ncols();
        let sigma_rows: Vec<MatrixMN<f64, U1, StateD>> = sigma_elements
            .chunks(dim)
            .map(|row| self.control_update(row, time, &control_input, map))
            .collect();
        self.set_control_sigma_matrix(MatrixMN::<f64, StateDx2, StateD>::from_rows(
            &sigma_rows[..],
        ));

        let config = self.config();
        let lambda = (config.alpha.powi(2)) * (dim as f64 + config.kappa) - dim as f64;
        let mut temp_known_state = MatrixMN::<f64, U1, StateD>::from_element(0.);
        for i in 0..=(2 * dim) {
            temp_known_state += self.control_sigma_matrix().row(i) / (dim as f64 + lambda)
                * if i != 0 { 1. / 2. } else { lambda };
        }
        self.set_known_state(temp_known_state);
        let temp_sigma_matrix = self.control_sigma_matrix()
            - MatrixMN::<f64, StateDx2, StateD>::from_rows(&vec![
                self.known_state();
                self.control_sigma_matrix()
                    .nrows()
            ]);

        let mut temp_covariance_matrix = MatrixMN::<f64, StateD, StateD>::from_element(0.);
        for i in 0..=(2 * dim) {
            temp_covariance_matrix += temp_sigma_matrix.row(i).transpose()
                * temp_sigma_matrix.row(i)
                * if i == 0 {
                    lambda / (dim as f64 + lambda) + (1. - config.alpha.powi(2) + config.beta)
                } else {
                    1. / (2. * (dim as f64 + lambda))
                };
        }
        temp_covariance_matrix += q;
        self.set_covariance_matrix(temp_covariance_matrix);
    }
    fn sensor_transform(&self, row: &[f64]) -> MatrixMN<f64, U1, SensorD>;
    fn measurement_update(
        &mut self,
        sensor_input: MatrixMN<f64, U1, SensorD>,
        r: MatrixMN<f64, SensorD, SensorD>,
    ) {
        let config = self.config();
        let sigma_elements: Vec<f64> = self
            .control_sigma_matrix()
            .transpose()
            .iter()
            .map(|elem| elem.clone())
            .collect();
        let dim = self.control_sigma_matrix().ncols();
        let sigma_rows: Vec<MatrixMN<f64, U1, SensorD>> = sigma_elements
            .chunks(dim)
            .map(|row| self.sensor_transform(row))
            .collect();
        self.set_sensor_sigma_matrix(MatrixMN::<f64, StateDx2, SensorD>::from_rows(
            &sigma_rows[..],
        ));

        let lambda = (config.alpha.powi(2)) * (dim as f64 + config.kappa) - dim as f64;
        let mut sensor_predicted = MatrixMN::<f64, U1, SensorD>::from_element(0.);
        for i in 0..=(2 * dim) {
            sensor_predicted += self.sensor_sigma_matrix().row(i) / (dim as f64 + lambda)
                * if i != 0 { 1. / 2. } else { lambda };
        }

        let mut cov_zz = MatrixMN::<f64, SensorD, SensorD>::from_element(0.);
        let temp_sensor_sigma_matrix = self.sensor_sigma_matrix()
            - MatrixMN::<f64, StateDx2, SensorD>::from_rows(&vec![
                sensor_predicted.clone();
                self.sensor_sigma_matrix()
                    .nrows()
            ]);
        for i in 0..=(2 * dim) {
            cov_zz += temp_sensor_sigma_matrix.row(i).transpose()
                * temp_sensor_sigma_matrix.row(i)
                * if i == 0 {
                    lambda / (dim as f64 + lambda) + (1. - config.alpha.powi(2) + config.beta)
                } else {
                    1. / (2. * (dim as f64 + lambda))
                };
        }
        cov_zz += r;
        let mut cov_xz = MatrixMN::<f64, StateD, SensorD>::from_element(0.);
        let temp_sigma_matrix = self.control_sigma_matrix()
            - MatrixMN::<f64, StateDx2, StateD>::from_rows(&vec![
                self.known_state();
                self.sensor_sigma_matrix().nrows()
            ]);
        for i in 0..=(2 * dim) {
            cov_xz += temp_sigma_matrix.row(i).transpose()
                * temp_sensor_sigma_matrix.row(i)
                * if i == 0 {
                    lambda / (dim as f64 + lambda) + (1. - config.alpha.powi(2) + config.beta)
                } else {
                    1. / (2. * (dim as f64 + lambda))
                };
        }
        let k: MatrixMN<f64, StateD, SensorD> = cov_xz
            * cov_zz.clone().try_inverse().unwrap_or_else(|| {
                panic!("Inverse of covariance matrix z, z failed");
            });
        let known_state_transpose: MatrixMN<f64, StateD, U1> =
            k.clone() * (sensor_input - sensor_predicted).transpose();
        self.set_known_state(self.known_state() + known_state_transpose.transpose());
        self.set_covariance_matrix(self.covariance_matrix() - (k.clone() * cov_zz * k.transpose()));
    }
}

pub struct LocalizationFilter {
    pub covariance_matrix: MatrixMN<f64, LocalizationStateD, LocalizationStateD>,
    pub known_state: MatrixMN<f64, U1, LocalizationStateD>,
    pub control_sigma_matrix: MatrixMN<f64, LocalizationStateDx2, LocalizationStateD>,
    sensor_sigma_matrix: MatrixMN<f64, LocalizationStateDx2, LocalizationSensorD>,
    config: Config,
}

impl KalmanFilter<LocalizationStateD, LocalizationSensorD, LocalizationStateDx2>
    for LocalizationFilter
{
    fn new(
        covariance_matrix: MatrixMN<f64, LocalizationStateD, U6>,
        init_state: MatrixMN<f64, U1, LocalizationStateD>,
        config: Config,
    ) -> LocalizationFilter {
        LocalizationFilter {
            covariance_matrix,
            known_state: init_state,
            control_sigma_matrix:
                MatrixMN::<f64, LocalizationStateDx2, LocalizationStateD>::from_element(0.),
            sensor_sigma_matrix:
                MatrixMN::<f64, LocalizationStateDx2, LocalizationSensorD>::from_element(0.),
            config,
        }
    }
    fn control_sigma_matrix(&self) -> MatrixMN<f64, LocalizationStateDx2, LocalizationStateD> {
        self.control_sigma_matrix
    }
    fn set_control_sigma_matrix(
        &mut self,
        control_sigma_matrix: MatrixMN<f64, LocalizationStateDx2, LocalizationStateD>,
    ) {
        self.control_sigma_matrix = control_sigma_matrix;
    }
    fn sensor_sigma_matrix(&self) -> MatrixMN<f64, LocalizationStateDx2, LocalizationSensorD> {
        self.sensor_sigma_matrix
    }
    fn set_sensor_sigma_matrix(
        &mut self,
        sensor_sigma_matrix: MatrixMN<f64, LocalizationStateDx2, LocalizationSensorD>,
    ) {
        self.sensor_sigma_matrix = sensor_sigma_matrix;
    }
    fn config(&self) -> Config {
        self.config
    }
    fn set_config(&mut self, config: Config) {
        self.config = config;
    }
    fn known_state(&self) -> MatrixMN<f64, U1, LocalizationStateD> {
        self.known_state
    }
    fn set_known_state(&mut self, known_state: MatrixMN<f64, U1, LocalizationStateD>) {
        self.known_state = known_state;
    }
    fn covariance_matrix(&self) -> MatrixMN<f64, LocalizationStateD, LocalizationStateD> {
        self.covariance_matrix
    }
    fn set_covariance_matrix(
        &mut self,
        covariance_matrix: MatrixMN<f64, LocalizationStateD, LocalizationStateD>,
    ) {
        self.covariance_matrix = covariance_matrix;
    }
    fn control_update(
        &self,
        row: &[f64],
        time: f64,
        control_input: &Vec<f64>,
        map: &Arc<Map2D>,
    ) -> MatrixMN<f64, U1, LocalizationStateD> {
        let control: Pose = control_input.clone().into();
        let mut sigma_state = KinematicState {
            angle: *row.get(0).unwrap(),
            position: Point {
                x: *row.get(1).unwrap(),
                y: *row.get(2).unwrap(),
            },
            vel_angle: *row.get(3).unwrap(),
            velocity: Point {
                x: *row.get(4).unwrap(),
                y: *row.get(5).unwrap(),
            },
        };
        sigma_state.control_update(control, time, map);
        MatrixMN::<f64, U1, LocalizationStateD>::from_vec(sigma_state.into())
    }
    fn sensor_transform(&self, row: &[f64]) -> MatrixMN<f64, U1, LocalizationSensorD> {
        let sigma_state: Vec<f64> = row.iter().cloned().collect();
        MatrixMN::<f64, U1, LocalizationSensorD>::from_vec(sigma_state)
    }
}

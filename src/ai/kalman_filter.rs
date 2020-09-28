use abomonation::Abomonation;
use generic_array::ArrayLength;
use nalgebra::{
    base::allocator::Allocator, ArrayStorage, DefaultAllocator, DimName, DimNameAdd, DimNameMul,
    DimSub, MatrixMN, MatrixN, U1,
};
use std::ops::Mul;
use typenum::operator_aliases::Prod;

pub struct Config {
    pub alpha: f64,
    pub beta: f64,
    pub kappa: f64,
}
pub trait KalmanFilter<StateD, SensorD, StateDx2>
where
    StateD: DimName + DimNameMul<StateD> + DimNameAdd<StateD> + DimSub<U1>,
    StateDx2: DimName + DimNameMul<StateDx2> + DimNameAdd<StateDx2>,
    SensorD: DimName + DimNameMul<SensorD> + DimNameAdd<SensorD>,
    StateD::Value: std::ops::Mul<SensorD::Value>,
    Prod<StateD::Value, SensorD::Value>: ArrayLength<f64>,
    DefaultAllocator: Allocator<f64, StateD, SensorD>,
    DefaultAllocator: Allocator<f64, U1, StateD>,
    DefaultAllocator: Allocator<f64, StateD, StateD>,
    DefaultAllocator: Allocator<f64, SensorD, SensorD>,
    DefaultAllocator: Allocator<f64, StateD>,
    DefaultAllocator: Allocator<f64, <StateD as nalgebra::DimSub<nalgebra::U1>>::Output>,
    DefaultAllocator: Allocator<f64, StateDx2, StateD>,
{
    fn control_sigma_matrix(&self) -> MatrixMN<f64, StateDx2, StateD>;
    fn set_control_sigma_matrix(&self, control_sigma_matrix: MatrixMN<f64, StateDx2, StateD>);
    fn sensor_sigma_matrix(&self) -> MatrixMN<f64, StateD, SensorD>;
    fn set_sensor_sigma_matrix(&self, sensor_sigma_matrix: MatrixMN<f64, StateD, SensorD>);
    fn config(&self) -> Config;
    fn set_config(&self, config: Config);
    fn known_state(&self) -> MatrixMN<f64, U1, StateD>;
    fn set_known_state(&self, known_state: MatrixMN<f64, U1, StateD>);
    fn covariance_matrix(&self) -> MatrixMN<f64, StateD, StateD>;
    fn set_covariance_matrix(&self, covariance_matrix: MatrixMN<f64, StateD, StateD>);
    fn new(
        covariance_matrix: MatrixMN<f64, StateD, StateD>,
        init_state: MatrixMN<f64, U1, StateD>,
        config: Config,
        q: MatrixMN<f64, StateD, StateD>,
        r: MatrixMN<f64, SensorD, SensorD>,
    ) -> Self;
    fn gen_sigma_matrix(&mut self) {
        let mut rows: Vec<MatrixMN<f64, U1, StateD>> = Vec::new();
        rows.push(self.known_state());
        let known_state = self.known_state();
        let config = self.config();
        let lambda = (config.alpha.powi(2)) * (6. + config.kappa) - 6.;
        let eigendecomp = (self.covariance_matrix() * (6. + lambda)).symmetric_eigen();
        let mut diagonalization = eigendecomp.eigenvalues;
        diagonalization.data.iter_mut().for_each(|e| {
            *e = e.max(0.).sqrt();
        });
        let square_root_cov = eigendecomp.eigenvectors
            * MatrixN::<f64, StateD>::from_diagonal(&diagonalization)
            * eigendecomp.eigenvectors.try_inverse().unwrap();
        for i in 0..=(6 - 1) {
            rows.push(known_state + square_root_cov.row(i));
        }
        for i in 0..=(6 - 1) {
            rows.push(known_state - square_root_cov.row(i));
        }

        self.set_control_sigma_matrix(MatrixMN::<f64, StateDx2, StateD>::from_rows(
            rows.as_slice(),
        ));
    }
    fn prediction_update(&mut self, time: f64, control: Pose) {
        self.gen_sigma_matrix();
        self.sigma_matrix.row_iter_mut().for_each(|mut e| {
            e[1] += e[4] * time;
            e[2] += e[5] * time;
            e[4] += (control.position.x * e[0].cos()
                + control.position.y * (e[0] - FRAC_PI_2).cos())
                * time;
            e[5] += (control.position.x * e[0].sin()
                + control.position.y * (e[0] - FRAC_PI_2).sin())
                * time;
            e[0] = (e[0] + e[3] * time) % (2. * PI);
            e[3] += control.angle * time;
        });
        // self.sigma_matrix.column(4) = Vector13::from_element(mult_pose.position.x);
        let lambda = (self.alpha.powi(2)) * (6. + self.kappa) - 6.;
        self.known_state = RowVector6::from_element(0.);
        for i in 0..=(2 * 6) {
            self.known_state +=
                self.sigma_matrix.row(i) / (6. + lambda) * if i != 0 { 1. / 2. } else { lambda };
        }
        let temp_sigma_matrix =
            self.sigma_matrix - Matrix13x6::from_rows(&vec![self.known_state; 13]);
        self.covariance_matrix = Matrix6::from_element(0.);
        for i in 0..=(2 * 6) {
            self.covariance_matrix += temp_sigma_matrix.row(i).transpose()
                * temp_sigma_matrix.row(i)
                * if i == 0 {
                    lambda / (6. + lambda) + (1. - self.alpha.powi(2) + self.beta)
                } else {
                    1. / (2. * (6. + lambda))
                };
        }
        self.covariance_matrix += self.q * time.powi(2);
    }
}

use generic_array::ArrayLength;
use nalgebra::{
    base::allocator::Allocator, DefaultAllocator, DimName, DimNameAdd, DimNameMul, DimSub,
    MatrixMN, MatrixN, U1,
};
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
    DefaultAllocator: Allocator<f64, SensorD, StateD>,
    DefaultAllocator: Allocator<f64, U1, StateD>,
    DefaultAllocator: Allocator<f64, U1, SensorD>,
    DefaultAllocator: Allocator<f64, SensorD, U1>,
    DefaultAllocator: Allocator<f64, StateD, StateD>,
    DefaultAllocator: Allocator<f64, SensorD, SensorD>,
    DefaultAllocator: Allocator<f64, StateD>,
    DefaultAllocator: Allocator<f64, <StateD as nalgebra::DimSub<nalgebra::U1>>::Output>,
    DefaultAllocator: Allocator<f64, StateDx2, StateD>,
    DefaultAllocator: Allocator<f64, StateD, StateDx2>,
    DefaultAllocator: Allocator<f64, StateDx2, SensorD>,
    DefaultAllocator: Allocator<f64, SensorD, StateDx2>,
{
    fn new(
        covariance_matrix: MatrixMN<f64, StateD, StateD>,
        init_state: MatrixMN<f64, U1, StateD>,
        config: Config,
    ) -> Self;
    fn control_sigma_matrix(&self) -> MatrixMN<f64, StateDx2, StateD>;
    fn set_control_sigma_matrix(&self, control_sigma_matrix: MatrixMN<f64, StateDx2, StateD>);
    fn sensor_sigma_matrix(&self) -> MatrixMN<f64, StateDx2, SensorD>;
    fn set_sensor_sigma_matrix(&self, sensor_sigma_matrix: MatrixMN<f64, StateDx2, SensorD>);
    fn config(&self) -> Config;
    fn set_config(&self, config: Config);
    fn known_state(&self) -> MatrixMN<f64, U1, StateD>;
    fn set_known_state(&self, known_state: MatrixMN<f64, U1, StateD>);
    fn covariance_matrix(&self) -> MatrixMN<f64, StateD, StateD>;
    fn set_covariance_matrix(&self, covariance_matrix: MatrixMN<f64, StateD, StateD>);
    fn gen_sigma_matrix(&mut self) {
        let mut rows: Vec<MatrixMN<f64, U1, StateD>> = Vec::new();
        rows.push(self.known_state());
        let known_state = self.known_state();
        let config = self.config();
        let lambda = (config.alpha.powi(2)) * (6. + config.kappa) - 6.;
        let eigendecomp = (self.covariance_matrix() * (6. + lambda)).symmetric_eigen();
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
    // e[1] += e[4] * time;
    // e[2] += e[5] * time;
    // e[4] += (control.position.x * e[0].cos()
    //     + control.position.y * (e[0] - FRAC_PI_2).cos())
    //     * time;
    // e[5] += (control.position.x * e[0].sin()
    //     + control.position.y * (e[0] - FRAC_PI_2).sin())
    //     * time;
    // e[0] = (e[0] + e[3] * time) % (2. * PI);
    // e[3] += control.angle * time;
    fn control_update(
        &self,
        row: &[f64],
        time: f64,
        control_input: &Vec<f64>,
    ) -> MatrixMN<f64, U1, StateD>;
    fn prediction_update(
        &mut self,
        time: f64,
        control_input: Vec<f64>,
        q: MatrixMN<f64, StateD, StateD>,
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
            .windows(dim)
            .map(|row| self.control_update(row, time, &control_input))
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
    fn sensor_transform(&self, row: &[f64], sensor_input: &Vec<f64>) -> MatrixMN<f64, U1, SensorD>;
    fn measurement_update(
        &mut self,
        sensor_input: MatrixMN<f64, U1, SensorD>,
        r: MatrixMN<f64, SensorD, SensorD>,
    ) {
        let config = self.config();
        let sensor_vec: Vec<f64> = sensor_input.iter().map(|e| e.clone()).collect();
        let sigma_elements: Vec<f64> = self
            .sensor_sigma_matrix()
            .transpose()
            .iter()
            .map(|elem| elem.clone())
            .collect();
        let dim = self.control_sigma_matrix().ncols();
        let sigma_rows: Vec<MatrixMN<f64, U1, SensorD>> = sigma_elements
            .windows(dim)
            .map(|row| self.sensor_transform(row, &sensor_vec))
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

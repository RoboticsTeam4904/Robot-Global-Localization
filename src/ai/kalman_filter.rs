use crate::utility::{KinematicState, Point, Pose};
use nalgebra::{base::allocator::Allocator, Const, DefaultAllocator, DimSub};
use nalgebra::{OMatrix, ToTypenum};
#[derive(Debug, Clone, Copy)]

/// The configuration of a given Unscented Kalman Filter with specific parameters.
///
/// α: the spread of sigma points around the expected state: set to a small positive value,
/// between 0 and 0.001.
///
/// κ: a scaling factor relevant to the algorithm - normally set to 0.
///
/// β: a variable set to either 2 or 0, depending on whether we assume a normal distribution.
pub struct Config {
    pub alpha: f64,
    pub beta: f64,
    pub kappa: f64,
}

impl Default for Config {
    // default configuration, assuming a normal distribution with alpha = 0.00001
    fn default() -> Config {
        Config {
            alpha: 1e-5,
            beta: 2.,
            kappa: 0.,
        }
    }
}

const LOCALIZATION_STATE_D: usize = 6;
const LOCALIZATION_SENSOR_D: usize = 6;
const LOCALIZATION_STATE_D_1: usize = LOCALIZATION_STATE_D * 2 + 1;

/// Trait for a general Kalman Filter with arbitrary state and sensor dimension space.
/// In each time step, run the control update (with struct-specified control transform)
/// and sensor update (with struct-specified sensor transform).
pub trait KalmanFilter<const STATE_D: usize, const STATE_D_1: usize, const SENSOR_D: usize>
where
    Const<STATE_D>: ToTypenum + DimSub<Const<1_usize>>,
    DefaultAllocator: Allocator<f64, Const<STATE_D>>
        + Allocator<f64, Const<STATE_D_1>, Const<STATE_D>>
        + Allocator<f64, <Const<STATE_D> as nalgebra::DimSub<Const<1_usize>>>::Output>
        + Allocator<f64, Const<SENSOR_D>, Const<STATE_D>>
        + Allocator<f64, Const<1_usize>, Const<STATE_D>>
        + Allocator<f64, Const<1_usize>, Const<SENSOR_D>>
        + Allocator<f64, Const<SENSOR_D>, Const<1_usize>>
        + Allocator<f64, Const<STATE_D>, Const<1_usize>>
        + Allocator<f64, Const<STATE_D>, Const<STATE_D>>
        + Allocator<f64, Const<SENSOR_D>, Const<SENSOR_D>>
        + Allocator<f64, Const<STATE_D>>
        + Allocator<f64, Const<STATE_D_1>, Const<STATE_D>>
        + Allocator<f64, Const<STATE_D>, Const<STATE_D_1>>
        + Allocator<f64, Const<STATE_D_1>, Const<SENSOR_D>>
        + Allocator<f64, Const<SENSOR_D>, Const<STATE_D_1>>,
{
    /// Create a new Kalman Filter state, using an initial covariance matrix for the state,
    /// an initial state that corresponds to the expected value, and a configuration for the
    /// Kalman Filter.
    fn new(
        covariance_matrix: OMatrix<f64, Const<STATE_D>, Const<STATE_D>>,
        init_state: OMatrix<f64, Const<1>, Const<STATE_D>>,
        config: Config,
    ) -> Self;

    /// Return the matrix of 2n + 1 sigma points of dimension n, row-wise.
    fn control_sigma_matrix(&self) -> OMatrix<f64, Const<STATE_D_1>, Const<STATE_D>>;

    fn set_control_sigma_matrix(
        &mut self,
        control_sigma_matrix: OMatrix<f64, Const<STATE_D_1>, Const<STATE_D>>,
    );

    /// Return the matrix of 2n + 1 mapped sigma points in sensor space of dimension m, row-wise.
    fn sensor_sigma_matrix(&self) -> OMatrix<f64, Const<STATE_D_1>, Const<SENSOR_D>>;

    fn set_sensor_sigma_matrix(
        &mut self,
        sensor_sigma_matrix: OMatrix<f64, Const<STATE_D_1>, Const<SENSOR_D>>,
    );

    fn config(&self) -> Config;

    fn set_config(&mut self, config: Config);

    /// Returns Kalman Filter expected state.
    fn known_state(&self) -> OMatrix<f64, Const<1_usize>, Const<STATE_D>>;

    fn set_known_state(&mut self, known_state: OMatrix<f64, Const<1_usize>, Const<STATE_D>>);

    /// Returns Kalman Filter covariance matrix.
    fn covariance_matrix(&self) -> OMatrix<f64, Const<STATE_D>, Const<STATE_D>>;

    fn set_covariance_matrix(
        &mut self,
        covariance_matrix: OMatrix<f64, Const<STATE_D>, Const<STATE_D>>,
    );

    /// Generate the sigma points from the covariance matrix and state. Follows the algorithm
    /// described in https://github.com/RoboticsTeam4904/wiki/wiki/Unscented-Kalman-Filters.
    fn gen_sigma_matrix(&mut self) {
        let mut rows: Vec<OMatrix<f64, Const<1_usize>, Const<STATE_D>>> = Vec::new();
        rows.push(self.known_state());

        let known_state = self.known_state();
        let config = self.config();
        let lambda = (config.alpha.powi(2)) * (STATE_D as f64 + config.kappa) - STATE_D as f64;

        // Generate the eigenvector and eigenvalues decomposition of the covariance matrix
        // (which is a positive operator), so that we can take the unique positive square
        // root.
        let eigendecomp =
            (self.covariance_matrix().scale(STATE_D as f64 + lambda)).symmetric_eigen();
        let mut diagonalization = eigendecomp.eigenvalues.clone();

        // Square root eigenvalues
        diagonalization.apply(|e| *e = e.max(0.).sqrt());

        // Rebuild the square root matrix by conjugating the diagonalized matrix by the
        // initial eigenvectors.
        let square_root_cov = eigendecomp.eigenvectors.clone()
            * OMatrix::<f64, Const<STATE_D>, Const<STATE_D>>::from_diagonal(&diagonalization)
            * eigendecomp.eigenvectors.try_inverse().unwrap();

        // Create the sigma points in the directions of the square root eigenvectors,
        // and make a corresponding matrix.
        for row in square_root_cov.row_iter() {
            rows.push(known_state.clone() + row.clone());
        }
        for row in square_root_cov.row_iter() {
            rows.push(known_state.clone() - row.clone());
        }

        // Set the new control matrix with the generated sigma points.
        self.set_control_sigma_matrix(OMatrix::<f64, Const<STATE_D_1>, Const<STATE_D>>::from_rows(
            rows.as_slice(),
        ));
    }

    /// Map a given state, using odometry control data, over time to a new state.
    /// Include a kinematics update from velocity to influence position.
    fn control_update(
        &self,
        row: &[f64],
        time: f64,
        control_input: &Vec<f64>,
    ) -> OMatrix<f64, Const<1_usize>, Const<STATE_D>>;

    /// Prediction update step. Taking a control update, and a corresponding noise,
    /// map the sigma points and update the covariance matrix.
    fn prediction_update(
        &mut self,
        time: f64,
        control_input: Vec<f64>,
        q: OMatrix<f64, Const<STATE_D>, Const<STATE_D>>, // control error matrix
    ) {
        self.gen_sigma_matrix();

        let sigma_elements: Vec<f64> = self
            .control_sigma_matrix()
            .transpose()
            .iter()
            .map(|elem| elem.clone())
            .collect();

        // Mapping the sigma points through the control update, for a new sigma matrix.
        let sigma_rows: Vec<OMatrix<f64, Const<1>, Const<STATE_D>>> = sigma_elements
            .chunks(STATE_D)
            .map(|row| self.control_update(row, time, &control_input))
            .collect();
        self.set_control_sigma_matrix(OMatrix::<f64, Const<STATE_D_1>, Const<STATE_D>>::from_rows(
            &sigma_rows[..],
        ));

        // Take a weighted average of the mapped sigma points to find the expected mapped.
        let config = self.config();
        let lambda = (config.alpha.powi(2)) * (STATE_D as f64 + config.kappa) - STATE_D as f64;
        let mut temp_known_state = OMatrix::<f64, Const<1>, Const<STATE_D>>::from_element(0.);
        for i in 0..=(2 * STATE_D) {
            temp_known_state += self.control_sigma_matrix().row(i) / (STATE_D as f64 + lambda)
                * if i != 0 { 1. / 2. } else { lambda };
        }

        // Set this average as the known state.
        self.set_known_state(temp_known_state);

        // Subtract each row of the outputted sigma matrix by the known state.
        let temp_sigma_matrix = self.control_sigma_matrix()
            - OMatrix::<f64, Const<STATE_D_1>, Const<STATE_D>>::from_rows(&vec![
                self.known_state();
                self.control_sigma_matrix()
                    .nrows()
            ]);

        // Calculate the new covariance using the matrix above, weighted, and by
        // adding on an additional noise term from the control update (q).
        let mut temp_covariance_matrix =
            OMatrix::<f64, Const<STATE_D>, Const<STATE_D>>::from_element(0.);
        for i in 0..=(2 * STATE_D) {
            temp_covariance_matrix += temp_sigma_matrix.row(i).transpose()
                * temp_sigma_matrix.row(i)
                * if i == 0 {
                    lambda / (STATE_D as f64 + lambda) + (1. - config.alpha.powi(2) + config.beta)
                } else {
                    1. / (2. * (STATE_D as f64 + lambda))
                };
        }
        temp_covariance_matrix += q;
        self.set_covariance_matrix(temp_covariance_matrix);
    }

    // Transform data in the state space to sensor data.
    fn sensor_transform(&self, row: &[f64]) -> OMatrix<f64, Const<1>, Const<SENSOR_D>>;

    // Measurement step of the Kalman Filter. Map sigma points to sensor space, intersect
    // with measurements, and find the new expected state and covariance matrix.
    fn measurement_update(
        &mut self,
        sensor_input: OMatrix<f64, Const<1>, Const<SENSOR_D>>,
        r: OMatrix<f64, Const<SENSOR_D>, Const<SENSOR_D>>, // sensor error covariance matrix
    ) {
        let config = self.config();
        let sigma_elements: Vec<f64> = self
            .control_sigma_matrix()
            .transpose()
            .iter()
            .map(|elem| elem.clone())
            .collect();

        // Sensor transform all of the sigma points.
        let sigma_rows: Vec<OMatrix<f64, Const<1>, Const<SENSOR_D>>> = sigma_elements
            .chunks(STATE_D)
            .map(|row| self.sensor_transform(row))
            .collect();
        self.set_sensor_sigma_matrix(
            OMatrix::<f64, Const<STATE_D_1>, Const<SENSOR_D>>::from_rows(&sigma_rows[..]),
        );

        // Find the expected sensor measurement with the weighted average of the sensor
        // sigma points.
        let lambda = (config.alpha.powi(2)) * (STATE_D as f64 + config.kappa) - STATE_D as f64;
        let mut sensor_predicted = OMatrix::<f64, Const<1>, Const<SENSOR_D>>::from_element(0.);
        for i in 0..=(2 * STATE_D) {
            sensor_predicted += self.sensor_sigma_matrix().row(i) / (STATE_D as f64 + lambda)
                * if i != 0 { 1. / 2. } else { lambda };
        }

        let mut cov_zz = OMatrix::<f64, Const<SENSOR_D>, Const<SENSOR_D>>::from_element(0.);
        let temp_sensor_sigma_matrix = self.sensor_sigma_matrix()
            - OMatrix::<f64, Const<STATE_D_1>, Const<SENSOR_D>>::from_rows(&vec![
                sensor_predicted.clone();
                self.sensor_sigma_matrix()
                    .nrows()
            ]);

        // Calculate the new covariance using the matrix above, weighted, and by
        // adding on an additional noise term from the control update (r).
        for i in 0..=(2 * STATE_D) {
            cov_zz += temp_sensor_sigma_matrix.row(i).transpose()
                * temp_sensor_sigma_matrix.row(i)
                * if i == 0 {
                    lambda / (STATE_D as f64 + lambda) + (1. - config.alpha.powi(2) + config.beta)
                } else {
                    1. / (2. * (STATE_D as f64 + lambda))
                };
        }
        cov_zz += r;

        // Follow the Kalman Filter algorithm, using the Kalman gain, to calculate the
        // correct final state and covariance matrix in state space.
        let mut cov_xz = OMatrix::<f64, Const<STATE_D>, Const<SENSOR_D>>::from_element(0.);
        let temp_sigma_matrix = self.control_sigma_matrix()
            - OMatrix::<f64, Const<STATE_D_1>, Const<STATE_D>>::from_rows(&vec![
                self.known_state();
                self.sensor_sigma_matrix()
                    .nrows()
            ]);
        for i in 0..=(2 * STATE_D) {
            cov_xz += temp_sigma_matrix.row(i).transpose()
                * temp_sensor_sigma_matrix.row(i)
                * if i == 0 {
                    lambda / (STATE_D as f64 + lambda) + (1. - config.alpha.powi(2) + config.beta)
                } else {
                    1. / (2. * (STATE_D as f64 + lambda))
                };
        }

        let k: OMatrix<f64, Const<STATE_D>, Const<SENSOR_D>> = cov_xz
            * cov_zz.clone().try_inverse().unwrap_or_else(|| {
                panic!("Inverse of covariance matrix z, z failed");
            });

        let sensor_diff: OMatrix<f64, Const<SENSOR_D>, Const<1>> =
            (sensor_input - sensor_predicted).transpose();
        let known_state_transpose: OMatrix<f64, Const<STATE_D>, Const<1>> = k.clone() * sensor_diff;

        self.set_known_state(self.known_state() + known_state_transpose.transpose());
        self.set_covariance_matrix(self.covariance_matrix() - (k.clone() * cov_zz * k.transpose()));
    }
}

pub struct LocalizationFilter {
    pub covariance_matrix: OMatrix<f64, Const<LOCALIZATION_STATE_D>, Const<LOCALIZATION_STATE_D>>,
    pub known_state: OMatrix<f64, Const<1>, Const<LOCALIZATION_STATE_D>>,
    pub control_sigma_matrix:
        OMatrix<f64, Const<LOCALIZATION_STATE_D_1>, Const<LOCALIZATION_STATE_D>>,
    sensor_sigma_matrix: OMatrix<f64, Const<LOCALIZATION_STATE_D_1>, Const<LOCALIZATION_SENSOR_D>>,
    config: Config,
}

impl KalmanFilter<LOCALIZATION_STATE_D, LOCALIZATION_STATE_D_1, LOCALIZATION_SENSOR_D>
    for LocalizationFilter
{
    fn new(
        covariance_matrix: OMatrix<f64, Const<LOCALIZATION_STATE_D>, Const<LOCALIZATION_STATE_D>>,
        init_state: OMatrix<f64, Const<1>, Const<LOCALIZATION_STATE_D>>,
        config: Config,
    ) -> LocalizationFilter {
        LocalizationFilter {
            covariance_matrix,
            known_state: init_state,
            control_sigma_matrix: OMatrix::<
                f64,
                Const<LOCALIZATION_STATE_D_1>,
                Const<LOCALIZATION_STATE_D>,
            >::from_element(0.),
            sensor_sigma_matrix: OMatrix::<
                f64,
                Const<LOCALIZATION_STATE_D_1>,
                Const<LOCALIZATION_SENSOR_D>,
            >::from_element(0.),
            config,
        }
    }

    fn control_sigma_matrix(
        &self,
    ) -> OMatrix<f64, Const<LOCALIZATION_STATE_D_1>, Const<LOCALIZATION_STATE_D>> {
        self.control_sigma_matrix
    }

    fn set_control_sigma_matrix(
        &mut self,
        control_sigma_matrix: OMatrix<
            f64,
            Const<LOCALIZATION_STATE_D_1>,
            Const<LOCALIZATION_STATE_D>,
        >,
    ) {
        self.control_sigma_matrix = control_sigma_matrix;
    }

    fn sensor_sigma_matrix(
        &self,
    ) -> OMatrix<f64, Const<LOCALIZATION_STATE_D_1>, Const<LOCALIZATION_SENSOR_D>> {
        self.sensor_sigma_matrix
    }

    fn set_sensor_sigma_matrix(
        &mut self,
        sensor_sigma_matrix: OMatrix<
            f64,
            Const<LOCALIZATION_STATE_D_1>,
            Const<LOCALIZATION_SENSOR_D>,
        >,
    ) {
        self.sensor_sigma_matrix = sensor_sigma_matrix;
    }

    fn config(&self) -> Config {
        self.config
    }

    fn set_config(&mut self, config: Config) {
        self.config = config;
    }

    fn known_state(&self) -> OMatrix<f64, Const<1>, Const<LOCALIZATION_STATE_D>> {
        self.known_state
    }
    fn set_known_state(
        &mut self,
        known_state: OMatrix<f64, Const<1>, Const<LOCALIZATION_STATE_D>>,
    ) {
        self.known_state = known_state;
    }

    fn covariance_matrix(
        &self,
    ) -> OMatrix<f64, Const<LOCALIZATION_STATE_D>, Const<LOCALIZATION_STATE_D>> {
        self.covariance_matrix
    }

    fn set_covariance_matrix(
        &mut self,
        covariance_matrix: OMatrix<f64, Const<LOCALIZATION_STATE_D>, Const<LOCALIZATION_STATE_D>>,
    ) {
        self.covariance_matrix = covariance_matrix;
    }

    fn control_update(
        &self,
        row: &[f64],
        time: f64,
        control_input: &Vec<f64>,
    ) -> OMatrix<f64, Const<1>, Const<LOCALIZATION_STATE_D>> {
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

        // Run a control update on the kinematics state with the acceleration of interest.
        sigma_state.control_update(control, time);
        OMatrix::<f64, Const<1>, Const<LOCALIZATION_STATE_D>>::from_vec(sigma_state.into())
    }

    // This sensor is literally just the state, with velocity data in this case
    // coming from the IMU and the position data coming as an output of MCL.
    fn sensor_transform(
        &self,
        row: &[f64],
    ) -> OMatrix<f64, Const<1>, Const<LOCALIZATION_SENSOR_D>> {
        let sigma_state: Vec<f64> = row.iter().cloned().collect();
        OMatrix::<f64, Const<1>, Const<LOCALIZATION_SENSOR_D>>::from_vec(sigma_state)
    }
}

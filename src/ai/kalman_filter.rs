use crate::utility::Pose;
use generic_array::ArrayLength;
use nalgebra::base::allocator::Allocator;
use nalgebra::base::default_allocator::DefaultAllocator;
use nalgebra::base::dimension::{Dim, DimAdd, DimMul, DimName, DimNameAdd, DimNameMul};
use nalgebra::{ArrayStorage, Matrix, Matrix6, RowVector6, U1, U13, U6};
use std::f64::consts::*;
use std::ops::Mul;
use typenum::operator_aliases::Prod;

pub struct Config {
    alpha: f64,
    beta: f64,
    kappa: f64,
}

pub trait KalmanFilter<A, B, C, D, E, F, G>
where
    A: DimName,
    B: DimName,
    A::Value: Mul<B::Value>,
    Prod<A::Value, B::Value>: ArrayLength<f64>,
    <<A as nalgebra::base::dimension::DimName>::Value as std::ops::Mul<
        <B as nalgebra::base::dimension::DimName>::Value,
    >>::Output: generic_array::ArrayLength<f64>,
{
    type StateDimension: DimName;
    type SensorDimension: DimName;
    fn control_sigma_matrix(&self) -> Matrix<f64, A, B, ArrayStorage<f64, A, B>>;

    // fn set_control_sigma_matrix(
    //     &self,
    //     control_sigma_matrix: Matrix<
    //         f64,
    //         Self::State2x1Dimension,
    //         Self::ControlDimension,
    //         ArrayStorage<f64, Self::State2x1Dimension, Self::ControlDimension>,
    //     >,
    // );

    // fn sensor_sigma_matrix(
    //     &self,
    // ) -> Matrix<
    //     f64,
    //     Self::State2x1Dimension,
    //     Self::SensorDimension,
    //     ArrayStorage<f64, Self::State2x1Dimension, Self::SensorDimension>,
    // >;

    // fn known_state(
    //     &self,
    // ) -> Matrix<f64, Self::StateDimension, U1, ArrayStorage<f64, Self::StateDimension, U1>>;

    // fn set_known_state(
    //     &self,
    //     set_matrix: Matrix<
    //         f64,
    //         Self::StateDimension,
    //         U1,
    //         ArrayStorage<f64, Self::StateDimension, U1>,
    //     >,
    // );

    // fn config(&self) -> Config;

    // fn covariance_matrix(
    //     &self,
    // ) -> Matrix<
    //     f64,
    //     Self::StateDimension,
    //     Self::StateDimension,
    //     ArrayStorage<f64, Self::StateDimension, Self::StateDimension>,
    // >;

    // fn update_sigma_matrix(&mut self);

    // fn control(
    //     control_sigma_matrix: Matrix<
    //         f64,
    //         Self::State2x1Dimension,
    //         Self::ControlDimension,
    //         ArrayStorage<f64, Self::State2x1Dimension, Self::ControlDimension>,
    //     >,
    //     control: Matrix<
    //         f64,
    //         Self::ControlDimension,
    //         U1,
    //         ArrayStorage<f64, Self::ControlDimension, U1>,
    //     >,
    //     time: f64,
    // ) -> Matrix<
    //     f64,
    //     Self::State2x1Dimension,
    //     Self::ControlDimension,
    //     ArrayStorage<f64, Self::State2x1Dimension, Self::ControlDimension>,
    // > {
    //     control_sigma_matrix.row_iter_mut().for_each(|mut e| {
    //         e[1] += e[4] * time;
    //         e[2] += e[5] * time;
    //         e[4] += (control.position.x * e[0].cos()
    //             + control.position.y * (e[0] - FRAC_PI_2).cos())
    //             * time;
    //         e[5] += (control.position.x * e[0].sin()
    //             + control.position.y * (e[0] - FRAC_PI_2).sin())
    //             * time;
    //         e[0] = (e[0] + e[3] * time) % (2. * PI);
    //         e[3] += control.angle * time;
    //     });
    //     control_sigma_matrix
    // }

    // fn sense();

    // fn prediction_update(
    //     &mut self,
    //     time: f64,
    //     control: Matrix<
    //         f64,
    //         Self::ControlDimension,
    //         U1,
    //         ArrayStorage<f64, Self::ControlDimension, U1>,
    //     >,
    // ) {
    //     self.gen_sigma_matrix();

    //     self.set_control_sigma_matrix(control(&mut self.control_sigma_matrix(), control));
    //     // self.control_sigma_matrix().column(4) = Vector13::from_element(mult_pose.position.x);
    //     let lambda = (self.config().alpha.powi(2)) * (6. + self.config().kappa) - 6.;
    //     self.set_known_state(RowVector6::from_element(0.));
    //     for i in 0..=(2 * 6) {
    //         self.set_known_state(
    //             self.known_state()
    //                 + self.control_sigma_matrix().row(i) / (6. + lambda)
    //                     * if i != 0 { 1. / 2. } else { lambda },
    //         );
    //     }
    //     let temp_sigma_matrix =
    //         self.control_sigma_matrix() - Matrix::from_rows(&vec![self.known_state(); 13]);
    //     self.covariance_matrix() = Matrix::from_element(0.);
    //     for i in 0..=(2 * 6) {
    //         self.set_covariance_matrix(
    //             self.covariance_matrix()
    //                 + temp_sigma_matrix.row(i).transpose()
    //                     * temp_sigma_matrix.row(i)
    //                     * if i == 0 {
    //                         lambda / (6. + lambda)
    //                             + (1. - self.config().alpha.powi(2) + self.config().beta)
    //                     } else {
    //                         1. / (2. * (6. + lambda))
    //                     },
    //         );
    //     }
    //     self.set_covariance_matrix(self.q() + self.covariance_matrix());
    // }

    // fn measurement_update(
    //     &mut self,
    //     sensor_data: Matrix<
    //         f64,
    //         Self::SensorDimension,
    //         U1,
    //         ArrayStorage<f64, Self::SensorDimension, U1>,
    //     >,
    // ) {
    // }
}

// /// Uses Unscented Kalman Filter to approximate robot state
// pub struct KalmanFilter {
//     pub covariance_matrix(): Matrix6<f64>,
//     pub known_state(): RowVector6<f64>,
//     pub sigma_matrix: Matrix13x6,
//     q: Matrix6<f64>,
//     r: Matrix6<f64>,
//     sensor_sigma_matrix: Matrix13x6,
//     config().beta: f64,
//     config().alpha: f64,
//     kappa: f64,
// }

// impl KalmanFilter {
//     pub fn new(
//         covariance_matrix(): Matrix6<f64>,
//         init_state: RowVector6<f64>, // the components of this vector are x-pos, y-pos, theta, x-acceleration, y-acceleration
//         config().alpha: f64,
//         kappa: f64,
//         config().beta: f64,
//         q: Matrix6<f64>,
//         r: Matrix6<f64>,
//     ) -> Self {
//         Self {
//             covariance_matrix(),
//             known_state(): init_state,
//             sigma_matrix: Matrix13x6::from_element(0.),
//             q,
//             r,
//             sensor_sigma_matrix: Matrix13x6::from_element(0.),
//             config().beta,
//             config().alpha,
//             kappa,
//         }
//     }

//     fn update_sigma_matrix(&mut self) {
//         let mut rows: Vec<RowVector6<f64>> = Vec::new();
//         rows.push(self.known_state());
//         let lambda = (self.config().alpha.powi(2)) * (6. + self.kappa) - 6.;
//         let eigendecomp = (self.covariance_matrix() * (6. + lambda)).symmetric_eigen();
//         let mut diagonalization = eigendecomp.eigenvalues;
//         diagonalization.data.iter_mut().for_each(|e| {
//             *e = e.max(0.).sqrt();
//         });
//         let square_root_cov = eigendecomp.eigenvectors
//             * Matrix6::from_diagonal(&diagonalization)
//             * eigendecomp.eigenvectors.try_inverse().unwrap();
//         for i in 0..=(6 - 1) {
//             rows.push(self.known_state() + square_root_cov.row(i));
//         }
//         for i in 0..=(6 - 1) {
//             rows.push(self.known_state() - square_root_cov.row(i));
//         }

//         self.control_sigma_matrix() = Matrix13x6::from_rows(rows.as_slice());
//     }

//     pub fn prediction_update(&mut self, time: f64, control: Pose) {
//         self.gen_sigma_matrix();
//         self.control_sigma_matrix()
//             .row_iter_mut()
//             .for_each(|mut e| {
//                 e[1] += e[4] * time;
//                 e[2] += e[5] * time;
//                 e[4] += (control.position.x * e[0].cos()
//                     + control.position.y * (e[0] - FRAC_PI_2).cos())
//                     * time;
//                 e[5] += (control.position.x * e[0].sin()
//                     + control.position.y * (e[0] - FRAC_PI_2).sin())
//                     * time;
//                 e[0] = (e[0] + e[3] * time) % (2. * PI);
//                 e[3] += control.angle * time;
//             });
//         // self.control_sigma_matrix().column(4) = Vector13::from_element(mult_pose.position.x);
//         let lambda = (self.config().alpha.powi(2)) * (6. + self.kappa) - 6.;
//         self.known_state() = RowVector6::from_element(0.);
//         for i in 0..=(2 * 6) {
//             self.known_state() += self.control_sigma_matrix().row(i) / (6. + lambda)
//                 * if i != 0 { 1. / 2. } else { lambda };
//         }
//         let temp_sigma_matrix =
//             self.control_sigma_matrix() - Matrix13x6::from_rows(&vec![self.known_state(); 13]);
//         self.covariance_matrix() = Matrix6::from_element(0.);
//         for i in 0..=(2 * 6) {
//             self.covariance_matrix() += temp_sigma_matrix.row(i).transpose()
//                 * temp_sigma_matrix.row(i)
//                 * if i == 0 {
//                     lambda / (6. + lambda) + (1. - self.config().alpha.powi(2) + self.config().beta)
//                 } else {
//                     1. / (2. * (6. + lambda))
//                 };
//         }
//         self.covariance_matrix() += self.config().q;
//     }

//     pub fn measurement_update(&mut self, velocity_sensor_data: Pose, mcl_pose: Pose) {
//         let sensor_update_vector = vec![
//             mcl_pose.angle,
//             mcl_pose.position.x,
//             mcl_pose.position.y,
//             velocity_sensor_data.angle,
//             velocity_sensor_data.position.x,
//             velocity_sensor_data.position.y,
//         ];
//         let sensor_update = RowVector6::from_vec(sensor_update_vector);
//         self.sensor_sigma_matrix = Matrix13x6::from_rows(
//             self.control_sigma_matrix()
//                 .row_iter()
//                 .map(|e| {
//                     let mut sensor_data: Vec<f64> = Vec::new();
//                     sensor_data.extend(vec![e[0], e[1], e[2], e[3], e[4], e[5]]);
//                     RowVector6::from_vec(sensor_data.clone())
//                 })
//                 .collect::<Vec<_>>()
//                 .as_slice(),
//         );

//         let lambda = (self.config().alpha.powi(2)) * (6. + self.kappa) - 6.;
//         let mut sensor_predicted = RowVector6::from_element(0.);
//         for i in 0..=(2 * 6) {
//             sensor_predicted += self.sensor_sigma_matrix.row(i) / (6. + lambda)
//                 * if i != 0 { 1. / 2. } else { lambda };
//         }

//         let mut cov_zz = Matrix6::from_element(0.);
//         let temp_sensor_sigma_matrix =
//             self.sensor_sigma_matrix - Matrix13x6::from_rows(&vec![sensor_predicted; 13]);
//         for i in 0..=(2 * 6) {
//             cov_zz += temp_sensor_sigma_matrix.row(i).transpose()
//                 * temp_sensor_sigma_matrix.row(i)
//                 * if i == 0 {
//                     lambda / (6. + lambda) + (1. - self.config().alpha.powi(2) + self.config().beta)
//                 } else {
//                     1. / (2. * (6. + lambda))
//                 };
//         }
//         cov_zz += self.r;
//         let mut cov_xz = Matrix6::from_element(0.);
//         let temp_sigma_matrix =
//             self.control_sigma_matrix() - Matrix13x6::from_rows(&vec![self.known_state(); 13]);
//         for i in 0..=(2 * 6) {
//             cov_xz += temp_sigma_matrix.row(i).transpose()
//                 * temp_sensor_sigma_matrix.row(i)
//                 * if i == 0 {
//                     lambda / (6. + lambda) + (1. - self.config().alpha.powi(2) + self.config().beta)
//                 } else {
//                     1. / (2. * (6. + lambda))
//                 };
//         }
//         let k = cov_xz
//             * cov_zz.try_inverse().unwrap_or_else(|| {
//                 panic!("{:?} {:?}", cov_zz, cov_zz.determinant());
//             });
//         self.known_state() += (k * (sensor_update - sensor_predicted).transpose()).transpose();
//         self.covariance_matrix() -= k * cov_zz * k.transpose();
//     }
// }

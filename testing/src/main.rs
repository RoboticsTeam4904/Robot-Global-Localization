extern crate nalgebra as na;
use na::{Matrix, Dynamic, VecStorage, DMatrix, DVector};


struct PoseEstimate {
    pub pose_vector: DVector<f64>,
    pub covariance_matrix: Matrix<f64, Dynamic, Dynamic, 
    VecStorage<f64, Dynamic, Dynamic>>,
}

impl PoseEstimate {
    pub fn new(sigmas: std::vec::Vec<f64>, 
    initial_pose: std::vec::Vec<f64>) -> PoseEstimate {
        assert_eq!(initial_pose.len(), sigmas.len(),
        "Testing if the pose vector and covariance matrix have the same \
        number of rows");
        Self{
            pose_vector: DVector::from_vec(initial_pose),
            covariance_matrix: DMatrix::from_diagonal(
                &DVector::from_vec(sigmas)),
        }
    }

    // // TODO: Make private after testing
    // pub fn Control_Update



}

fn main() {
    let mut tester = PoseEstimate::new(vec![1., 2.], vec![30.5, 40.]);
    println!("{}", tester.pose_vector);
    println!("{}", tester.covariance_matrix);
}
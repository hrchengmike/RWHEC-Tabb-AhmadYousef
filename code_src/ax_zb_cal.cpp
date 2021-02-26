// This code is being distributed with no warranties whatsoever.
//
// Author: Amy Tabb.  Original release circa 2016.  Current date May 30, 2018.
// small updates Dec. 3 2018
//
//  If you use this code to calibrate a robot in an academic setting, please cite this paper!  Thanks a bunch.
//@article{tabb_solving_2017,
//	title = {Solving the robot-world hand-eye(s) calibration problem with iterative methods},
//	volume = {28},
//	issn = {1432-1769},
//	url = {https://doi.org/10.1007/s00138-017-0841-7},
//	doi = {10.1007/s00138-017-0841-7},
//	abstract = {Robot-world, hand-eye calibration is the problem of determining the transformation between the robot end-effector and a camera, as well as the transformation between the robot base and the world coordinate system. This relationship has been modeled as                                                                           \$\$\{{\textbackslash}mathbf \{AX\}\}=\{{\textbackslash}mathbf \{ZB\}\}\$\$                                                                                    AX                        =                        ZB                                                                            , where                                                                           \$\$\{{\textbackslash}mathbf \{X\}\}\$\$                                                            X                                                       and                                                                           \$\$\{{\textbackslash}mathbf \{Z\}\}\$\$                                                            Z                                                       are unknown homogeneous transformation matrices. The successful execution of many robot manipulation tasks depends on determining these matrices accurately, and we are particularly interested in the use of calibration for use in vision tasks. In this work, we describe a collection of methods consisting of two cost function classes, three different parameterizations of rotation components, and separable versus simultaneous formulations. We explore the behavior of this collection of methods on real datasets and simulated datasets and compare to seven other state-of-the-art methods. Our collection of methods returns greater accuracy on many metrics as compared to the state-of-the-art. The collection of methods is extended to the problem of robot-world hand-multiple-eye calibration, and results are shown with two and three cameras mounted on the same robot.},
//	number = {5},
//	journal = {Machine Vision and Applications},
//	author = {Tabb, Amy and Ahmad Yousef, Khalil M.},
//	month = aug,
//	year = {2017},
//	pages = {569--590}
//}
//
// If you've inherited this code, the original source is from the github repository amy-tabb .  Check back there for updates periodically, particularly as it pertains to OpenCV versions.
// You're welcome to modify it to your needs for other projects.

#include "Tabb_AhmadYousef_RWHEC_Jun2018_main.hpp"

#include "DirectoryFunctions.hpp"
#include "StringFunctions.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

using namespace std;

bool VERBOSE = true;


int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	Eigen::initParallel();

	/////////////////////////////************************************/////////////////////////////
	string source_dir;
	string external_dir;
	string write_dir;
	string robot_file = "";
	string robot_dir;
	string hand_eye_dir;

	int flag;
	cout << "Usage for calibration is function_name input_directory write_directory " << endl;

	if (argc == 3){
		source_dir =  string(argv[1]);
		write_dir = argv[2];

		EnsureDirHasTrailingBackslash(source_dir);

	}	else {
		cout << "Wrong number of arguments " << endl;
		exit(1);
	}

	RobotWorldHandEyeCalibration(source_dir, write_dir);
	return 0;
}


int RobotWorldHandEyeCalibration(string source_dir, string write_dir){

	string command;
	std::ofstream out;
	std::ifstream in;
	string filename;

	vector<vector<Matrix4d>> As;
    As.push_back(vector<Matrix4d>());
	vector<Matrix4d> Bs;
    int robot_mounted_cameras = 1;
	vector<int> number_images_per(robot_mounted_cameras, 0);
    /*
			// write the As and image points
			for (int i = 0; i < int(COs[k].Rts.size()); i++){
				if (As[k][i].rows() > 0){
					out << As[k][i] << endl;

					for (int j = 0; j < number_points; j++){
						out << COs[k].all_points[COs[k].number_internal_images_written + i][j].x << " " << COs[k].all_points[COs[k].number_internal_images_written + i][j].y << " ";
					}
					out << endl;
				}
			}
			out.close();
    */

    //read A matrices
    filename = source_dir + "/As.txt";
    ReadRobotFileRobotCaliTxt(filename, As[0]);
    //read B matrices
    filename = source_dir + "/Bs.txt";
    ReadRobotFileRobotCaliTxt(filename, Bs);
    std::cout << Bs[3](2,3) << std::endl;
    std::cout << As[0][3](2,3) << std::endl;
	Matrix3d Rz;
	Matrix3d Ry;
	Matrix3d Rx;

	////////////////////////// ROBOT SECTION /////////////////////////////////////////

	Matrix3d TestR;
	Matrix4d X;
	Matrix4d Z;

	X.setIdentity();  Z.setIdentity();


	double null_triple[3];
	null_triple[0] = 0;
	null_triple[1] = 0;
	null_triple[2] = 0;
	double* xarray = new double[7*(robot_mounted_cameras + 1)];
	double xm[16];

	vector<double> rotation_error;
	vector<double> translation_error;
	vector<double> whole_error;
	vector<vector<double> > reprojection_error;

	vector<double> axis_angle_difference;
	vector<double> reconstruction_reprojection_errors;
	vector<double> difference_between_reconstruction_and_original;
	vector<string> name_vector;

	string descriptor_string;
	string abbreviated_descriptor_string;
	string write_result_directory;
	vector<Matrix4d> Zs;
    Zs.push_back(Z);
	PARAM_TYPE param_type;
	COST_TYPE cost_type;
    bool separable = false;
	// All of the methods -- this is optional.  You could choose one or two methods to save time, for instance, a reprojection error method is recommended.
	// (suggest any of 12-16)
    separable = false;
    descriptor_string = "Axis Angle c2 simultaneous";
    abbreviated_descriptor_string = "AA_c2_simul";
    //param_type = AxisAngle;
    param_type = Euler;
    cost_type = c2;	
	write_result_directory = write_dir + "/rwhe_" + abbreviated_descriptor_string;
	name_vector.push_back(descriptor_string);

    bool do_rwhec = true;
		/// create directories  //////////////////////////////////

		if (do_rwhec){

			command = "mkdir " + write_result_directory;
			system(command.c_str());

			filename = write_result_directory + "/details.txt";

			out.open(filename.c_str());


			switch (cost_type){
			case c1: {
				 // This case intentionally left blank
			}
			case c2: {
				switch (param_type){
				case Euler: {
					out << "Euler parameterization  ";
				} break;
				case AxisAngle: {
					out << "Axis Angle parameterization  ";
				} break;
				case Cali_Quaternion: {
					out << "Quaternion parameterization  ";
				} break;
				}

				switch (cost_type){
				case c1: {
					out <<  " ||AX-ZB||^2, no reprojection error" << endl;
				} break;
				case c2: {
					out << " ||A-ZBX^{-1}||^2, no reprojection error" << endl;
				} break;
				default: {
					cout << "Other types not handled in this switch." << endl;
				} break;
				}


				{

					for (int i = 0; i < (robot_mounted_cameras + 1)*7; i++){
						xarray[i] = 0;
					}

					// x array is x, then zs.
					if (param_type == Cali_Quaternion){
						for (int i = 0; i < (robot_mounted_cameras + 1); i++){
							ceres::AngleAxisToQuaternion(null_triple, &xarray[7*i]);
						}
					}

					// solve
                    
                    double* x = new double[6];
                    double* z = new double[6];
					if (separable == false){
						CF1_2_one_camera(As, Bs, x,z, out, param_type, cost_type);
					}	
                    /*else {
						CF1_2_multi_camera_separable(As, Bs, xarray, out, param_type, cost_type, rotation_only);

						CF1_2_multi_camera_separable(As, Bs, xarray, out, param_type, cost_type, translation_only);
					}
                    */

					// convert to the matrix representation
					for (int i = 1; i < (robot_mounted_cameras + 1); i++)
					{


						switch (param_type){
						case Euler: {
							Convert6ParameterEulerAngleRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
						} break;
						case AxisAngle: {
							Convert6ParameterAxisAngleRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
						} break;
						case Cali_Quaternion: {
							Convert7ParameterQuaternionRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
						} break;
						}


						for (int r = 0, in = 0; r < 4; r++){
							for (int c = 0; c < 4; c++, in++){
								Zs[i - 1](r, c) = xm[in];
							}
						}

					}

					switch (param_type){
					case Euler: {
						Convert6ParameterEulerAngleRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
					} break;
					case AxisAngle: {
						Convert6ParameterAxisAngleRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
					} break;
					case Cali_Quaternion: {
						Convert7ParameterQuaternionRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
					} break;
					}


					// convert to the X, Z representation
					for (int r = 0, in = 0; r < 4; r++){
						for (int c = 0; c < 4; c++, in++){
							X(r, c) = xm[in];
						}
					}

					if (cost_type == c2){
						X = X.inverse().eval();
					}


				}
			} break;

			}

			cout << "X " << endl << X << endl;
			out << "X " << endl << X << endl;

			for (int i = 0; i < robot_mounted_cameras; i++){
				cout << "Z " << i << endl << Zs[i] << endl;
				out << "Z " << i << endl << Zs[i] << endl;
			}
            /*
			double error;
			error  = 0;

			for (int i = 0; i < robot_mounted_cameras; i++){
				cout << "Size " << As[i].size() << ", " << Bs.size() << ", " << Zs.size() << endl;

				error += AssessRotationError(As[i], Bs, X, Zs[i]);
			}
			out << "Summed rotation error " << error << endl;
			rotation_error.push_back(error);

			error  = 0;
			for (int i = 0; i < robot_mounted_cameras; i++){
				cout << "Size " << As[i].size() << ", " << Bs.size() << ", " << Zs.size() << endl;
				error += AssessRotationErrorAxisAngle(As[i], Bs, X, Zs[i]);
			}
			out << "Summed angle difference rotation error " << error << endl;
			axis_angle_difference.push_back(error);

			error  = 0;
			for (int i = 0; i < robot_mounted_cameras; i++){
				error += AssessTranslationError(As[i], Bs, X, Zs[i]);
			}
			out << "Summed translation error " << error << endl;
			translation_error.push_back(error);

			error  = 0;
			for (int i = 0; i < robot_mounted_cameras; i++){
				error += AssessErrorWhole(As[i], Bs, X, Zs[i]);
			}
			out << "Summed whole error " << error << endl;
			whole_error.push_back(error);

			error  = 0;
			vector<double> ID_project_error;
			for (int i = 0; i < robot_mounted_cameras; i++){
				error = CalculateReprojectionError(&COs[i], As[i], Bs, X, Zs[i], out, write_result_directory, i);
				ID_project_error.push_back(error);
			}
			reprojection_error.push_back(ID_project_error);

			out.close();

			filename = write_result_directory + "/transformations.txt";
			out.open(filename.c_str());

			out << "X" << endl << X << endl;
			for (int i = 0; i < robot_mounted_cameras; i++){
				out << "Z " << i << endl << Zs[i] << endl;
			}
			out.close();

			for (int i = 0; i < robot_mounted_cameras; i++){
				command = "mkdir " + write_result_directory + "/camera" + ToString<int>(i);
				system(command.c_str());

				filename = write_result_directory + "/camera" + ToString<int>(i) + "/cali.txt";
				out.open(filename.c_str());

				WriteCaliFile(&COs[i], As[i], Bs, X, Zs[i], out);

				out.close();

			}

			if (cost_type == rp2){
				CopyToCalibration(COs, camera_parameters_from_cali);
			}
            */
		} 


	//}

	//delete [] xarray;

    /*
	if (do_rwhec){
		filename = write_dir + "/comparisons.txt";
		out.open(filename.c_str());
		out << "Errors are normalized by the number of robot positions, which is " << number_cameras << " summed for all cameras, which is " << robot_mounted_cameras << " excepting the rms" << endl;
		out << setw(36) << "Method name "
				<< setw(20) << "Rotation error"
				<< setw(20) << "Rot error - angle"
				<< setw(20) << "Translation error"
				<< setw(20) << "Whole error"
				<< setw(20*robot_mounted_cameras) << "reprojection error"
				<< setw(20*robot_mounted_cameras) << "RMS reprojection" << endl;

		std::cout.precision(6);

		for (int i = 0; i < int(rotation_error.size()); i++){
			out << i << " " << setw(36);
			out << name_vector[i];
			out <<  " " << rotation_error[i]
				 << " " << setw(20) << axis_angle_difference[i]
																				   << setw(20) <<  translation_error[i];

			out << setw(20) << whole_error[i];

			for (int j = 0; j < robot_mounted_cameras; j++){
				out <<  setw(20) << reprojection_error[i][j];
			}


			for (int j = 0; j < robot_mounted_cameras; j++){
				out <<  setw(20) << sqrt((1.0/double(As.size()*number_images_per[j]*COs[0].all_3d_corners[0].size())) * reprojection_error[i][j]);
			}
			out << endl;
		}


		out << endl;
		out << "___________________________________________________________________________________" << endl;
		out << endl;

		out.close();
	}

	if (do_reconstruction){
		filename = write_dir + "/reconstruction_accuracy_error_comparisons.txt";
		out.open(filename.c_str());

		out << setw(36) << "Method name "
				<< setw(20) << "reprojection error"
				<< setw(20) << "difference distance" <<  endl;

		for (int i = 0; i < int(reconstruction_reprojection_errors.size()); i++){
			out << i << " " << setw(36);
			out << name_vector[i];
			out << setw(40) << reconstruction_reprojection_errors[i]
																  << setw(40) << difference_between_reconstruction_and_original[i]<< endl;
		}
	}

    */
	return 1;
}



void WriteCaliFile(CaliObjectOpenCV2* CO, std::ofstream& out){

	int n = 0;
	for (int i = 0; i < int(CO->Rts.size()); i++){
		if (CO->Rts[i].size() > 0){
			n++;
		}
	}
	out << n << endl;

	for (int i = 0; i < int(CO->Rts.size()); i++){

		if (CO->Rts[i].size() > 0){
			out << "image" << ToString<int>(i) << ".png " << " ";
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					out << CO->A[r][c] << " ";
				}
			}

			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					out << CO->Rts[i][r][c] << " ";;
				}
			}

			for (int r = 0; r < 3; r++){
				out << CO->Rts[i][r][3] << " ";;
			}

			for (int r = 0; r < int(CO->k.size()); r++){
				out << CO->k[r] << " ";
			}

			out << endl;
		}
	}
}

void WriteCaliFile(CaliObjectOpenCV2* CO, vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z, std::ofstream& out){
	out << As.size() << endl;

	Matrix4d newA(4, 4);


	vector<Matrix4d> newAs;
	int n = As.size();
	for (int i = 0; i < n; i++){

		newA = Z*Bs[i]*X.inverse();

		out << "image" << ToString<int>(i) << ".png " << " ";
		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				out << CO->A[r][c] << " ";
			}
		}

		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				out << newA(r,c) << " ";
			}
		}

		for (int r = 0; r < 3; r++){
			out << newA(r,3) << " ";
		}

		for (int r = 0; r < int(CO->k.size()); r++){
			out << CO->k[r] << " ";
		}

		out << endl;

	}
}


void ReadRobotFileRobotCaliTxt(string filename, vector<Matrix4d>& Bs){
	FILE * file;
	file = fopen(filename.c_str(), "r");
	if (!file){
		cout << "Source file is null -- fix and exiting for now... " << filename << endl;
		exit(1);
	}	else {
		 fclose(file);
	}

	Matrix4d B1;

	std::ifstream in;

	in.open(filename.c_str());

	int N;
	in >> N;

	for (int i = 0; i < N; i++){

		for (int r = 0; r < 4; r++){
			for (int c = 0; c < 4; c++){
				in >> B1(r, c);
			}
		}

		Bs.push_back(B1);
	}
	in.close();
}

double AssessErrorWhole(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;
	Matrix4d H;

	cout << "error " << error << endl;
	double number_stops = Bs.size();

	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			H = As[i]*X - Z*Bs[i];

			cout << "H" << H << endl;
			for (int r = 0; r < 4; r++){
				for (int c = 0; c < 4; c++){
					error +=  H(r, c)*H(r, c);
				}
			}
			//cout << "Error after one image " << error << endl;
		}
	}

	return error/number_stops;
}

double AssessRotationError(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;
	double local_error = 0;


	double number_stops = Bs.size();
	Matrix3d RA;  Matrix3d RZ;  Matrix3d RX; Matrix3d RB;
	Matrix3d Rdiff;

		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				RZ(r, c) = Z(r, c);
				RX(r, c) = X(r, c);
			}
		}



	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					RA(r, c) = As[i](r, c);
					RB(r, c) = Bs[i](r, c);
				}
			}

			Rdiff = RA*RX - RZ*RB;
			local_error = 0;
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					local_error += Rdiff(r, c)*Rdiff(r, c);
				}
			}

			error += local_error;
		}
	}

	return error/number_stops;
}

double AssessRotationErrorAxisAngle(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;
	double local_error = 0;

	Matrix3d R0;
	Matrix3d R1;
	Matrix3d R_relative;

	Matrix3d RA;  Matrix3d RZ;  Matrix3d RX; Matrix3d RB;

	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			RZ(r, c) = Z(r, c);
			RX(r, c) = X(r, c);
		}
	}


	double RV[9];
	double aa[3];
	double number_stops = Bs.size();
	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			for (int r = 0; r < 3; r++){
					for (int c = 0; c < 3; c++){
						RA(r, c) = As[i](r, c);
						RB(r, c) = Bs[i](r, c);
					}
			}


			R0 = RA*RX;
			R1 = RZ*RB;

			R_relative = R1.transpose()*R0;
			for (int r = 0, index = 0; r < 3; r++){
				for (int c = 0; c < 3; c++, index++){
					RV[index] = R_relative(r, c);
				}
			}

			ceres::RotationMatrixToAngleAxis(RV, aa);

			local_error = 57.2958 * sqrt(pow(aa[0], 2) + pow(aa[1], 2) + pow(aa[2], 2));

			//R.NormFrobenius()
			error += local_error;
		}
	}

	// remember to average amoung cameras when there is more than one camera.
	return error/number_stops;
}



double AssessTranslationError(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;

	Matrix3d RA;  Matrix3d RZ;
	MatrixXd tx(3, 1);
	MatrixXd ta(3, 1);
	MatrixXd tb(3, 1);
	MatrixXd tz(3, 1);

	Matrix4d H;
//	double number_stops = Bs.size();

//	for (int i = 0; i < int(Bs.size()); i++){
//		if (As[i].size() > 0){
//			H = As[i]*X - Z*Bs[i];
//
//			//cout << "H" << H << endl;
//			for (int r = 0; r < 4; r++){
//				for (int c = 3; c < 4; c++){
//					error +=  H(r, c)*H(r, c);
//				}
//			}
//			//cout << "Error after one image " << error << endl;
//		}
//
//	}


	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			RZ(r, c) = Z(r, c);
		}

		tx(r, 0) = X(r, 3);
		tz(r, 0) = Z(r, 3);
	}

	MatrixXd result0, result1;

	double number_stops = Bs.size();

	Matrix4d H1, H2;

	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			H1 = As[i]*X;
			H2 = Z*Bs[i];


			// this is wrong.  top_row = As[i].SubMatrix(1, 3, 1, 3)*X.SubMatrix(1, 3, 4, 4) + As[i].SubMatrix(1, 3, 4, 4)
			for (int r = 0; r < 3; r++){
					for (int c = 0; c < 3; c++){
						RA(r, c) = As[i](r, c);
					}
					ta(r, 0) = As[i](r, 3);
					tb(r, 0) = Bs[i](r, 3);
			}

			result0 = RA*tx + ta - (RZ*tb + tz);

//			cout << "H1 " << H1 << endl;
//			cout << "H2 " << H2 << endl;
//
//			cout << "result0 " << result0 << endl;
//			cout << "result1 " << result1 << endl;
//
//			char fg; cin >> fg;

			//cout << "translation result " << result0 << endl;

			for (int r = 0; r < 3; r++){
				//error += (H1(r,3) - H2(r, 3))*(H1(r,3) - H2(r, 3));
				error += result0(r, 0)*result0(r, 0);
			}


		}
	}

	// this error is squared within the parentheses
	// Within the main loop, average amoung cameras for multi-camera datasets.
	return (error/number_stops);
}

double CalculateReprojectionError(CaliObjectOpenCV2* CO, vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z, std::ofstream& out, string directory, int cam_number){


	double reproj_error = 0;
	vector<cv::Point2f> imagePoints2;
	double err;
	Matrix4d newA;

	vector<Matrix4d> newAs;
	cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	cv::Mat im;
	cv::Mat gray;

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			cameraMatrix.at<double>(i, j)  = CO->A[i][j];
		}
	}

	for (int i = 0; i < 8; i++){
		distCoeffs.at<double>(i, 0) = CO->k[i];
	}

	string filename;
	int non_blanks = 0;

	for (int i = 0; i < int(As.size()); i++){
		if (CO->all_points[CO->number_internal_images_written + i].size() > 0){

			newA = Z*Bs[i]*X.inverse();

			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					R.at<double>(r, c) = newA(r, c);
				}
				tvec.at<double>(r, 0) = newA(r, 3);
			}

			cv::Rodrigues(R, rvec);

			cv::projectPoints( cv::Mat(CO->all_3d_corners[CO->number_internal_images_written + non_blanks]), rvec, tvec, cameraMatrix,  // project
					distCoeffs, imagePoints2);
			err = cv::norm(cv::Mat(CO->all_points[CO->number_internal_images_written + i]), cv::Mat(imagePoints2), CV_L2);              // difference

			reproj_error        += err*err;                                             // su

			im = CO->external_images[i].clone();
			cv::cvtColor(im, gray, CV_BGR2GRAY);
			cv::cvtColor(gray, im, CV_GRAY2BGR);


			for (int j = 0; j < int(imagePoints2.size()); j++){
				cv::line(im, imagePoints2[j], CO->all_points[CO->number_internal_images_written + i][j], cv::Scalar(255, 0, 0), 2, 8);
			}


			filename = directory + "/reproj" + ToString<int>(cam_number) + "_" + ToString<int>(i) + ".png";


			cv::imwrite(filename.c_str(), im);
			non_blanks++;
		}

	}

	out << endl << "Summed reproj error "  << reproj_error << endl << endl;


	for (int i = 0; i < int(As.size()); i++){

		newA = Z*Bs[i]*X.inverse();


		out << "newA for i " << i <<  endl << newA << endl;
	}

	return reproj_error;
}


void WritePatterns(double* pattern_points, int chess_h, int chess_w, int index_number, string outfile){

	// each vertex needs a color ....
	// first, find the range ....

	vector<vector<int> > colors;

	vector<int> c(3);
	// lowest value is grey
	c[0] = 0;
	c[1] = 0;
	c[2] = 0;

	colors.push_back(c);

	// next lowest value is purple
	c[0] = 128;
	c[1] = 0;
	c[2] = 128;
	colors.push_back(c);

	// next lowest value is blue
	c[0] = 0;
	c[1] = 0;
	c[2] = 200;
	colors.push_back(c);

	// next lowest value is cyan
	c[0] = 0;
	c[1] = 255;
	c[2] = 255;
	colors.push_back(c);

	// next lowest value is green
	c[0] = 0;
	c[1] = 255;
	c[2] = 0;
	colors.push_back(c);

	// next lowest value is yellow
	c[0] = 255;
	c[1] = 255;
	c[2] = 0;
	colors.push_back(c);

	// next lowest value is red
	c[0] = 255;
	c[1] = 0;
	c[2] = 0;
	colors.push_back(c);

	c = colors[index_number % colors.size()];


	cout << "Writing to " << outfile << endl;
	std::ofstream out;
	out.open(outfile.c_str());

	// first row has chess_h - 2 faces ?
	int number_faces = (chess_h/2)*(chess_w/2) + (chess_h/2 - 1)*(chess_w/2 - 1);


	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "element vertex " << chess_h*chess_w << endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	out << "property uchar red" << endl;
	out << "property uchar green" << endl;
	out << "property uchar blue" << endl;
	out << "property uchar alpha" << endl;
	out << "element face " << number_faces << endl;
	out << "property list uchar int vertex_indices"<< endl;
	out << "end_header" << endl;

	for (int i = 0; i < chess_h*chess_w; i++){
		out << pattern_points[i*3] << " " << pattern_points[i*3 + 1] << " " << pattern_points[i*3 + 2] << " " << c[0] << " " << c[1] << " " << c[2] << " 175" << endl;
	}


	int p0, p1, p2, p3;
	for (int i = 0; i < chess_h; i++){

		for (int j = 0; j < chess_w/2; j++){
			if (i % 2  == 0){
				p0 = i*chess_w + 2*j;
				p1 = i*chess_w + 2*j + 1;
				p2 = (i + 1)*chess_w + 2*j + 1;
				p3 = (i + 1)*chess_w + 2*j;
				out << "4 " << p0 << " " << p1 << " " << p2 << " " << p3 << endl;

			}	else {
				if (j < chess_w/2 - 1){
					p0 = i*chess_w + 2*j + 1;
					p1 = i*chess_w + 2*j + 2;
					p2 = (i + 1)*chess_w + 2*j + 2;
					p3 = (i + 1)*chess_w + 2*j + 1;
					out << "4 " << p0 << " " << p1 << " " << p2 << " " << p3 << endl;

				}
			}


		}
	}


	out << endl;

	out.close();
}

string FindValueOfFieldInFile(string filename, string fieldTag, bool seperator){

	/// reopen file each time, in case things get switched around.  Assume that these are very small files, not the most efficient.

	ifstream in(filename.c_str());

	if (!in.good()){
		cout << "Filename to find " << fieldTag << " is bad " << filename << " quitting !" << endl;
		exit(1);
	}

	string cmp_str;
	string read_str;


	vector<string> tokens;
	string token;
	string return_str = "";
	bool found = false;


	while (in  && found == false){

		in >> token;

		if (token.compare(fieldTag) == 0){
			found = true;

			if (seperator == true && in){
				in >> token;
			}

			if (in){
				in >> return_str;
			}

		}
	}


	cout << "Found! " << found << " field " << fieldTag << " and result " << return_str << endl;
	in.close();

	return return_str;

}



void EnsureDirHasTrailingBackslash(string& write_directory){
	int n_letters = write_directory.size();
	bool eval =  (write_directory[n_letters - 1] == '/');
	cout << "Last character compare " << write_directory << " " <<  eval << endl;
	if (eval == false){
		write_directory = write_directory + "/";
	}

}


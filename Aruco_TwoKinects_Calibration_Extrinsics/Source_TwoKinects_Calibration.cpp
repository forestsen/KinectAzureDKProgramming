#include <k4a/k4a.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <array>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <aruco/aruco.h>

#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"

using namespace std;
using namespace cv;
using namespace sen;
using namespace Eigen;

k4a::calibration calibration;
Mat camera_matrix_color;
Mat dist_coeffs_color;

aruco::Dictionary dic;
aruco::CameraParameters CamParam;
aruco::MarkerDetector MDetector;
std::map<int, aruco::MarkerPoseTracker> MTracker_master;
std::map<int, aruco::MarkerPoseTracker> MTracker_sub;
float MarkerSize = 0.1f;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");


template <typename DataType>
void writeToCSVfile(std::string name, Eigen::Array<DataType, -1, -1> matrix)
{
	std::ofstream file(name.c_str());
	file << matrix.format(CSVFormat);
}

void eigenTransform2cvRvecTvec(const Transform<double, 3, Affine> frame, cv::Vec3d &rvec, cv::Vec3d &tvec)
{
	Translation<double, 3> t(frame.translation());
	Quaternion<double> q(frame.linear());
	tvec = cv::Vec3d(t.x(), t.y(), t.z());
	cv::Mat rotM;
	eigen2cv(q.toRotationMatrix(), rotM);
	cv::Rodrigues(rotM, rvec);
}

void drawAxis(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
	InputArray _rvec, InputArray _tvec, float length)
{
	CV_Assert(_image.getMat().total() != 0 &&
		(_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
	CV_Assert(length > 0);

	// project axis points
	vector< Point3f > axisPoints;
	axisPoints.push_back(Point3f(0, 0, 0));
	axisPoints.push_back(Point3f(length, 0, 0));
	axisPoints.push_back(Point3f(0, length, 0));
	axisPoints.push_back(Point3f(0, 0, length));
	vector< Point2f > imagePoints;
	projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

	// draw axis lines
	line(_image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
	line(_image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
	line(_image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
}

int main(int argc, char **argv)
{
	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;

	cout << "Started opening K4A device..." << endl;
	k4a::device dev_sub = k4a::device::open(1);
	dev_sub.start_cameras(&config);
	k4a::device dev_master = k4a::device::open(K4A_DEVICE_DEFAULT);
	dev_master.start_cameras(&config);
	cout << "Finished opening K4A device." << endl;

	std::pair<int, int> color_dimensions = GetColorDimensions(config.color_resolution);
	int texture_width = color_dimensions.first;
	int texture_height = color_dimensions.second;

	calibration = dev_master.get_calibration(config.depth_mode, config.color_resolution);
	k4a_calibration_intrinsic_parameters_t *intrinsics_color = &calibration.color_camera_calibration.intrinsics.parameters;
	vector<float> _camera_matrix = {
	   intrinsics_color->param.fx, 0.f, intrinsics_color->param.cx,
	   0.f, intrinsics_color->param.fy, intrinsics_color->param.cy,
	   0.f, 0.f, 1.f };
	camera_matrix_color = Mat(3, 3, CV_32F, &_camera_matrix[0]);
	vector<float> _dist_coeffs = { intrinsics_color->param.k1, intrinsics_color->param.k2, intrinsics_color->param.p1,
								   intrinsics_color->param.p2, intrinsics_color->param.k3, intrinsics_color->param.k4,
								   intrinsics_color->param.k5, intrinsics_color->param.k6 };
	dist_coeffs_color = Mat::zeros(5, 1, CV_32F);
	dic = aruco::Dictionary::load("ARUCO_MIP_36h12");
	CamParam.setParams(camera_matrix_color, dist_coeffs_color, cv::Size(texture_width, texture_height));
	MDetector.setDictionary("ARUCO_MIP_36h12", 0.05f);
	MDetector.setDetectionMode(aruco::DM_NORMAL);

	k4a::capture capture_master;
	k4a::capture capture_sub;

	k4a::image colorImage_master;
	uint8_t *colorTextureBuffer_master;
	cv::Mat colorFrame_master;

	k4a::image colorImage_sub;
	uint8_t *colorTextureBuffer_sub;
	cv::Mat colorFrame_sub;

	Eigen::Affine3d frame_master_marker;
	Eigen::Affine3d frame_sub_marker;
	Eigen::Affine3d frame_master_sub;

	bool poseEstimationOK_master = false;
	bool poseEstimationOK_sub = false;

	while (1)
	{
		if (dev_master.get_capture(&capture_master, std::chrono::milliseconds(0)) && dev_sub.get_capture(&capture_sub, std::chrono::milliseconds(0)))
		{
			//master
			{
				colorImage_master = capture_master.get_color_image();
				colorTextureBuffer_master = colorImage_master.get_buffer();
				colorFrame_master = cv::Mat(colorImage_master.get_height_pixels(), colorImage_master.get_width_pixels(), CV_8UC4, colorTextureBuffer_master);
				cvtColor(colorFrame_master, colorFrame_master, COLOR_BGRA2BGR);

				{
					vector<aruco::Marker> Markers = MDetector.detect(colorFrame_master);
					for (auto &marker : Markers)
					{
						MTracker_master[marker.id].estimatePose(marker, CamParam, MarkerSize);
					}
					if (CamParam.isValid() && MarkerSize != -1)
					{
						for (unsigned int i = 0; i < Markers.size(); ++i)
						{
							if (Markers[i].isPoseValid())
							{
								cv::Mat transformationMatrix_master = Markers[i].getTransformMatrix();
								cv2eigen(transformationMatrix_master, frame_master_marker.matrix());
								poseEstimationOK_master = true;
							}
							else
							{
								poseEstimationOK_master = false;
							}
						}
					}
				}
			}

			//sub
			{
				
				colorImage_sub = capture_sub.get_color_image();
				colorTextureBuffer_sub = colorImage_sub.get_buffer();
				colorFrame_sub = cv::Mat(colorImage_sub.get_height_pixels(), colorImage_sub.get_width_pixels(), CV_8UC4, colorTextureBuffer_sub);
				cvtColor(colorFrame_sub, colorFrame_sub, COLOR_BGRA2BGR);

				{
					vector<aruco::Marker> Markers = MDetector.detect(colorFrame_sub);
					for (auto &marker : Markers)
					{
						MTracker_sub[marker.id].estimatePose(marker, CamParam, MarkerSize);
					}
					if (CamParam.isValid() && MarkerSize != -1)
					{
						for (unsigned int i = 0; i < Markers.size(); ++i)
						{
							if (Markers[i].isPoseValid())
							{
								cv::Mat transformationMatrix_sub = Markers[i].getTransformMatrix();
								cv2eigen(transformationMatrix_sub, frame_sub_marker.matrix());
								poseEstimationOK_sub = true;
							}
							else
							{
								poseEstimationOK_sub = false;
							}
						}
					}
				}
			}
			
			if(poseEstimationOK_master && poseEstimationOK_sub)
			{
				Eigen::Affine3d frame_master_sub = frame_master_marker * frame_sub_marker.inverse();
				
				cv::Vec3d rvec, tvec;
				eigenTransform2cvRvecTvec(frame_sub_marker, rvec, tvec);
				drawAxis(colorFrame_sub, camera_matrix_color, dist_coeffs_color, rvec, tvec, 0.5f);

				Eigen::Affine3d frame_sub_master = frame_sub_marker * frame_master_marker.inverse();
				eigenTransform2cvRvecTvec(frame_sub_master, rvec, tvec);
				Eigen::Matrix4d frame_matrix = frame_sub_master.matrix();
				std::cout << "frame sub master" << std::endl;
				writeToCSVfile<double>("frame_sub_master.csv", frame_matrix);
				std::cout << "save matrix into csv file OK.\n";

				std::cout << "frame marker sub" << std::endl;
				frame_matrix = frame_sub_marker.inverse().matrix();
				writeToCSVfile<double>("frame_marker_sub.csv", frame_matrix);
				std::cout << "save matrix into csv file OK.\n";
				drawAxis(colorFrame_sub, camera_matrix_color, dist_coeffs_color, rvec, tvec, 0.5f);
			}
			else
			{
				std::cout << endl;
			}

			imshow("color sub", colorFrame_sub);
		}
		if (waitKey(30) == 27 || waitKey(30) == 'q')
		{
			dev_master.close();
			dev_sub.close();
			break;
		}
	}
	return 0;
}
#include "NTCamera.h"

#include <opencv2/core.hpp>

NTCamera::NTCamera(webots::Robot *robot, const Config &config) {
	camera = robot->getCamera(config.Name);

	if (camera == nullptr) {
		throw std::runtime_error(
				"Camera with name \"" + config.Name + "\" was not found!!!");
	}

	ntInst = nt::NetworkTableInstance::GetDefault();
	int timestep = robot->getBasicTimeStep();
	camera->enable(timestep);
	width = camera->getWidth();
	height = camera->getHeight();

	// Initialize the cvSource
	cvSource = cs::CvSource(config.Name, cs::VideoMode::kMJPEG, width, height,
			30);

	// Initialize the camera server
	mjpegServer = cs::MjpegServer(config.Name, 5800);
	mjpegServer.SetSource(cvSource);

	frc::CameraServer::AddServer (mjpegServer);
	cvSource = frc::CameraServer::PutVideo(config.Name, width, height);

}

void NTCamera::Init() {
}

void NTCamera::Update() {
	const unsigned char *image = camera->getImage();
	if (image == nullptr) {
		cout << "No image available" << endl;
		return;
	}

	Mat img = Mat(Size(width, height), CV_8UC4);
	img.data = const_cast<uchar*>(image);

	cvSource.PutFrame(img);

}

void to_json(nlohmann::json &j, const NTCamera::Config &c) {
	j = nlohmann::json { { "name", c.Name } };
}

void from_json(const nlohmann::json &j, NTCamera::Config &c) {
	j.at("name").get_to(c.Name);
}

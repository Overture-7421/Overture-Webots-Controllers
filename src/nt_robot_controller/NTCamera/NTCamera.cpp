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
	cvSource = cs::CvSource(config.Name, cs::VideoMode::kBGR, width, height,
			30);

	// Initialize the camera server
	mjpegServer = cs::MjpegServer(config.Name, config.Port);
	mjpegServer.SetSource(cvSource);

	cout << "INFO: The camera \"" << config.Name << "\" is located on the URL: "
			<< " http://localhost:" << config.Port << "/stream.mjpg" << endl;
}

void NTCamera::Init() {
	// Initialize any additional settings if needed
}

void NTCamera::Update() {
	const unsigned char *image = camera->getImage();
	if (image == nullptr) {
		return; // Handle error: no image available
	}

	Mat img = Mat(Size(width, height), CV_8UC4);
	img.data = const_cast<uchar*>(image);

	cvSource.PutFrame(img);
}

void to_json(nlohmann::json &j, const NTCamera::Config &c) {
	j = nlohmann::json { { "name", c.Name }, { "port", c.Port } };

}

void from_json(const nlohmann::json &j, NTCamera::Config &c) {
	j.at("name").get_to(c.Name);
	j.at("port").get_to(c.Port);
}

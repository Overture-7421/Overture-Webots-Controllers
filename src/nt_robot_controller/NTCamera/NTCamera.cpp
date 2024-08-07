#include "NTCamera.h"
#include <webots/Camera.hpp>
#include <iostream>
#include <sstream>

void process_image(const unsigned char *image, int length, int width,
		int height) {
	Mat img = Mat(Size(width, height), CV_8UC4);
	img.data = const_cast<uchar*>(image);
}

NTCamera::NTCamera(Robot *robot, const Config &config) {
	camera = robot->getCamera("Frontal Camera");

	if (camera == nullptr) {
		throw std::runtime_error(
				"Camera with name \"" + config.Name + "\" was not found!!!");

	}

	ntInst = nt::NetworkTableInstance::GetDefault();
	int timestep = robot->getBasicTimeStep();
	camera->enable(timestep);
	width = camera->getWidth();
	height = camera->getHeight();

}

void NTCamera::Init() {
}

void NTCamera::Update() {

	const unsigned char *image = camera->getImage();

	/* Variables for the display */
	int length = 4 * width * height * sizeof(unsigned char);

	process_image(image, length, width, height);

}

void to_json(nlohmann::json &j, const NTCamera::Config &c) {
	j = nlohmann::json { { "name", c.Name } };
}

void from_json(const nlohmann::json &j, NTCamera::Config &c) {
	j.at("name").get_to(c.Name);
}

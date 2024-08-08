#include <stdio.h>

#include "NTController/NTController.h"
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <ntcore.h>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <nlohmann/json.hpp>
#include <cscore_cv.h>
#include <cameraserver/CameraServer.h>

using namespace webots;
using namespace cv;
using namespace std;

class NTCamera: public NTController {

public:
	struct Config {
		std::string Name;
	};

	NTCamera(Robot *robot, const Config &config);
	void Init() override;
	void Update() override;

private:

	Camera *camera;
	nt::NetworkTableInstance ntInst;
	int width;
	int height;
	unsigned char *processed_image;
	cs::CvSource cvSource;
	cs::MjpegServer mjpegServer;

};

void to_json(nlohmann::json &j, const NTCamera::Config &c);
void from_json(const nlohmann::json &j, NTCamera::Config &c);

#include <stdio.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <webots/Robot.hpp>
#include <nlohmann/json.hpp>
#include "NTController/NTController.h"
#include <webots/Camera.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace webots;
using namespace cv;

class NTCamera: public NTController {

public:
	struct Config {
		std::string Name;
		bool Inverted;
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

};

void to_json(nlohmann::json &j, const NTCamera::Config &c);
void from_json(const nlohmann::json &j, NTCamera::Config &c);

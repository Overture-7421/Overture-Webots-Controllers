#pragma once

class NTController {
public:
	virtual void Init() = 0;
	virtual void Update() = 0;
	virtual ~NTController() {
	}
};

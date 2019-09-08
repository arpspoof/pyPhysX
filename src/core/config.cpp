#include "config.h"
#include <assert.h>
#include <fstream>

namespace {
	typedef ConfigEntry _C;
}

static Config default_config{
	// SWITCHES
	{ "S_ENABLE_GPU_DYNAMICS", _C(0) },
	{ "S_ENABLE_GPU_BROADPHASE", _C(0) },
	{ "S_GROUND", _C(1) },
	// SYSTEM
	{ "C_THREADS", _C(6) },
	{ "C_TIME_STEP", _C(0.001f) },
	{ "C_RENDER_FREQUENCY", _C(60.0f) },
	{ "C_STATIC_FRICTION", _C(1.0f) },
	{ "C_DYNAMIC_FRICTION", _C(1.0f) },
	// CONSTANTS
	{ "C_GRAVITY", _C(9.81f) },
	{ "C_TWIST_LIMIT", _C(0.7f) },
	// PARAMS
	{ "P_KP_chest", _C(500.0f) },
	{ "P_KD_chest", _C(100.0f) },
	{ "P_KP_neck", _C(50.0f) },
	{ "P_KD_neck", _C(10.0f) },
	{ "P_KP_right_hip", _C(250.0f) },
	{ "P_KD_right_hip", _C(50.0f) },
	{ "P_KP_left_hip", _C(250.0f) },
	{ "P_KD_left_hip", _C(50.0f) },
	{ "P_KP_right_knee", _C(250.0f) },
	{ "P_KD_right_knee", _C(50.0f) },
	{ "P_KP_left_knee", _C(250.0f) },
	{ "P_KD_left_knee", _C(50.0f) },
	{ "P_KP_right_ankle", _C(50.0f) },
	{ "P_KD_right_ankle", _C(40.0f) },
	{ "P_KP_left_ankle", _C(50.0f) },
	{ "P_KD_left_ankle", _C(40.0f) },
	{ "P_KP_right_shoulder", _C(200.0f) },
	{ "P_KD_right_shoulder", _C(40.0f) },
	{ "P_KP_left_shoulder", _C(200.0f) },
	{ "P_KD_left_shoulder", _C(40.0f) },
	{ "P_KP_right_elbow", _C(150.0f) },
	{ "P_KD_right_elbow", _C(30.0f) },
	{ "P_KP_left_elbow", _C(150.0f) },
	{ "P_KD_left_elbow", _C(30.0f) },
	{ "P_FL_left_shoulder", _C(100.0f) },
	{ "P_FL_neck", _C(50.0f) },
	{ "P_FL_right_shoulder", _C(100.0f) },
	{ "P_FL_left_knee", _C(150.0f) },
	{ "P_FL_left_ankle", _C(100.0f) },
	{ "P_FL_left_elbow", _C(60.0f) },
	{ "P_FL_right_knee", _C(150.0f) },
	{ "P_FL_right_ankle", _C(100.0f) },
	{ "P_FL_right_elbow", _C(60.0f) },
	{ "P_FL_chest", _C(200.0f) },
	{ "P_FL_left_hip", _C(200.0f) },
	{ "P_FL_right_hip", _C(200.0f) },
};

static const char* config_file_path = nullptr;
static Config config = default_config;

ConfigEntry& getConfig(const std::string& name) {
	assert(config.find(name) != config.end());
	return config.at(name);
}

void setConfig(const std::string& name, ConfigEntry c) {
	config[name] = c;
}

float getConfigF(const std::string& name) {
	return getConfig(name).f();
}

int getConfigI(const std::string& name) {
	return getConfig(name).i();
}

void setConfigF(const std::string& name, float f) {
	setConfig(name, _C(f));
}

void setConfigI(const std::string& name, int i) {
	setConfig(name, _C(i));
}

void readConfigFile(const char* path) {
	assert(path != nullptr);
	std::ifstream i(path);
	if (!i.is_open()) {
		printf("config file %s is not valid\n", path);
		return;
	}
//	printf("reading config file ...\n");

	std::string name, type;
	int vi;
	float vf;

	while (i >> name) {
		i >> type;
		if (type == "int") {
			i >> vi;
			config[name] = _C(vi);
//			printf("CONFIG: %s is set to %d\n", name.c_str(), vi);
		}
		else if (type == "float") {
			i >> vf;
			config[name] = _C(vf);
//			printf("CONFIG: %s is set to %f\n", name.c_str(), vf);
		}
		else {
			printf("typename %s is not valid\n", type.c_str());
			assert(false);
		}
	}

	config_file_path = path;
	i.close();

//	printf("reading config file done.\n");
}

void writeConfigFile() {
	if (config_file_path == nullptr) {
		printf("no valid config file specified.\n");
		return;
	}
	std::ofstream o(config_file_path);
	assert(o.is_open());
	printf("writing config file ...\n");
	for (auto& it : config) {
		o << it.first << "\t" << it.second.toString() << std::endl;
	}
	o.close();
	printf("write config file done.\n");
}

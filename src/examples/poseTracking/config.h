#pragma once

#include <memory.h>
#include <string>
#include <map>

enum ConfigType {
	_INT, _FLOAT
};

struct ConfigEntry {
	ConfigType type;
	unsigned char buffer[8];

	ConfigEntry(ConfigType _type) {
		type = _type;
		memset(buffer, 0, sizeof(buffer));
	}

	ConfigEntry() : ConfigEntry(ConfigType::_INT) {
	}

	ConfigEntry(int i) : ConfigEntry(ConfigType::_INT) {
		int* intbuffer = (int*)buffer;
		*intbuffer = i;
	}

	ConfigEntry(float f) : ConfigEntry(ConfigType::_FLOAT) {
		float* floatbuffer = (float*)buffer;
		*floatbuffer = f;
	}

	int i() { return *(int*)buffer; }
	float f() { return *(float*)buffer; }

	std::string toString() {
		std::string typestr;
		std::string valuestr;

		switch (type) {
		case ConfigType::_INT: 
			typestr = "int";
			valuestr = std::to_string(i());
			break;
		case ConfigType::_FLOAT: 
			typestr = "float";
			valuestr = std::to_string(f());
			break;
		}
		
		return typestr + "\t" + valuestr;
	}
};

typedef std::map<std::string, ConfigEntry> Config;

ConfigEntry& getConfig(const std::string& name);
float getConfigF(const std::string& name);
int getConfigI(const std::string& name);

void setConfig(const std::string& name, ConfigEntry c);
void setConfigF(const std::string& name, float f);
void setConfigI(const std::string& name, int i);

void readConfigFile(const char* path);
void writeConfigFile();

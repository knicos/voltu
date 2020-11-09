#include "cmd_args.hpp"

std::map<std::string, std::string> read_options(char ***argv, int *argc)
{
	std::map<std::string, std::string> opts;

	(*argc)--;  // Remove application path
	(*argv)++;

	while (*argc > 0) {
		std::string cmd((*argv)[0]);

		size_t p;
		if (cmd[0] != '-' || (p = cmd.find("=")) == std::string::npos) {
			opts[cmd.substr(0)] = "true";
		} else {
			auto val = cmd.substr(p+1);
#ifdef WIN32
			if ((val[0] >= 48 && val[0] <= 57) || val == "true" || val == "false" || val == "null") {
#else
			if (std::isdigit(val[0]) || val == "true" || val == "false" || val == "null") {
#endif
				opts[cmd.substr(0, p-2)] = val;
			} else {
				if (val[0] == '\\') opts[cmd.substr(2, p-2)] = val;
				else opts[cmd.substr(0, p-2)] = "\""+val+"\"";
			}
		}

		(*argc)--;
		(*argv)++;
	}

	return opts;
}

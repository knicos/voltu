#include <voltu/voltu.hpp>
#include <iostream>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using std::string;

int main(int argc, char **argv)
{
	if (argc != 2) return -1;

	auto vtu = voltu::instance();

	if (!vtu)
	{
		cout << "Failed to start VolTu" << endl;
		return -1;
	}

	if (!vtu->open(argv[1]))
	{
		cout << "Could not open source" << endl;
		return -1;
	}

	while (vtu->listRooms().size() == 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	auto room = vtu->getRoom(vtu->listRooms().front());

	return 0;
}

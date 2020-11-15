#include <voltu/voltu.hpp>
#include <voltu/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include <opencv2/highgui.hpp>

using std::cout;
using std::endl;
using std::string;

int main(int argc, char **argv)
{
	if (argc != 2) return -1;

	auto vtu = voltu::instance();

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
	if (!room)
	{
		cout << "Could not get room" << endl;
		return -1;
	}
	cout << "Room Name: " << room->getName() << endl;

	while (room->active())
	{
		if (!room->waitNextFrame(1000)) break;

		auto frame = room->getFrame();
		auto imgset = frame->getImageSet(voltu::Channel::kDepth);

		for (auto img : imgset)
		{
			cv::Mat m;
			voltu::opencv::visualise(img, m);
			cv::imshow(string("Image-") + img->getName(), m);
		}

		if (cv::waitKey(1) == 27) break;
	}

	return 0;
}

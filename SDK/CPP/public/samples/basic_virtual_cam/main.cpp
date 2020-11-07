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

	auto obs = vtu->createObserver();

	obs->setResolution(1280, 720);
	obs->setFocalLength(900);
	obs->setStereo(false);

	Eigen::Matrix4f pose;
	pose.setIdentity();
	float posz = 0.0f;
	float dz = 0.05f;

	while (room->active())
	{
		// Note: waitNextFrame is optional, getFrame will get old frame otherwise
		//if (!room->waitNextFrame(1000)) break;
		auto rframe = room->getFrame();

		Eigen::Affine3f aff = Eigen::Affine3f::Identity();
		aff.translate(Eigen::Vector3f(0.0f, 0.0f, posz));
		pose = aff.matrix();
		posz += dz;
		if (posz > 1.5f) dz = -dz;
		if (posz < -1.0f) dz = -dz;
		obs->setPose(pose);

		obs->submit(rframe);
		if (!obs->waitCompletion(1000)) break;
		
		auto cframe = obs->getFrame();
		auto imgset = cframe->getImageSet(voltu::Channel::kColour);

		for (auto img : imgset)
		{
			cv::Mat m;
			voltu::cv::convert(img, m);
			cv::imshow(string("Camera-") + img->getName(), m);
		}

		if (cv::waitKey(20) == 27) break;
	}

	return 0;
}

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

	auto frame = room->getFrame();

	auto pipe = vtu->createPipeline();
	auto op1 = pipe->appendOperator(voltu::OperatorId::kFusion);
	auto op2 = pipe->appendOperator(voltu::OperatorId::kGTEvaluator);

	op2->property("show_colour")->setBool(true);

	pipe->submit(frame);
	pipe->waitCompletion(1000);

	auto imgset = frame->getImageSet(voltu::Channel::kColour);

	for (auto img : imgset)
	{
		cv::Mat m;
		voltu::cv::visualise(img, m);
		cv::imshow(string("Image-") + img->getName(), m);
		break;
	}

	std::vector<std::string> msgs = frame->getMessages();
	for (const auto &s : msgs)
	{
		cout << s << endl;
	}

	cv::waitKey(-1);

	return 0;
}

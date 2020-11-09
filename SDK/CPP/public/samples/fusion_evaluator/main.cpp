#include <voltu/voltu.hpp>
#include <voltu/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include "../common/cmd_args.hpp"

#include <opencv2/highgui.hpp>

using std::cout;
using std::endl;
using std::string;

int main(int argc, char **argv)
{
	bool do_fusion = true;
	bool do_eval = true;
	voltu::Channel display_channel = voltu::Channel::kColour;
	std::list<std::string> paths;

	auto opts = read_options(&argv, &argc);

	for (const auto &s : opts)
	{
		cout << "ARGS " << s.first << " " << s.second << endl; 
		if (s.first == "--no-fusion")
		{
			do_fusion = false;
		}
		else if (s.first == "--display")
		{
			cout << "DISPLAY = " << s.second << endl;
			if (s.second == "\"normals\"")
			{
				display_channel = voltu::Channel::kNormals;
			}
		}
		else if (s.first == "--no-eval")
		{
			do_eval = false;
		}
		else if (s.first[0] != '-')
		{
			paths.push_back(s.first);
		}
	}

	auto vtu = voltu::instance();

	for (const auto &p : paths)
	{
		if (!vtu->open(p))
		{
			cout << "Could not open source" << endl;
			return -1;
		}
	}

	while (vtu->listRooms().size() == 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		cout << "Wait room..." << endl;
	}

	auto room = vtu->getRoom(vtu->listRooms().front());
	if (!room)
	{
		cout << "Could not get room" << endl;
		return -1;
	}

	//room->waitNextFrame(5000);
	auto frame = room->getFrame();

	auto pipe = vtu->createPipeline();
	auto op1 = pipe->appendOperator(voltu::OperatorId::kFusion);
	auto op2 = pipe->appendOperator(voltu::OperatorId::kGTEvaluator);

	op1->property("enabled")->setBool(do_fusion);
	op2->property("enabled")->setBool(do_eval);
	op2->property("show_colour")->setBool(true);

	pipe->submit(frame);
	if (!pipe->waitCompletion(3000))
	{
		cout << "Pipeline timeout" << endl;
		return -1;
	}

	auto imgset = frame->getImageSet(display_channel);

	if (imgset.size() == 0)
	{
		cout << "No images!" << endl;
		return -1;
	}

	for (auto img : imgset)
	{
		cv::Mat m;
		voltu::cv::visualise(img, m);
		cv::imshow(string("Image-") + img->getName(), m);
		break;
	}

	std::vector<std::vector<std::string>> msgs = frame->getMessages();
	if (msgs.size() > 0) {
		for (const auto &s : msgs[0])
		{
			cout << s << endl;
		}
	}

	cv::waitKey(-1);

	return 0;
}

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
	bool do_carving = false;
	bool show_changes = false;
	int frameno = 0;
	int sourceno = 0;
	int iters = 2;
	voltu::Channel display_channel = voltu::Channel::kColour;
	std::list<std::string> paths;

	auto opts = read_options(&argv, &argc);

	for (const auto &s : opts)
	{
		if (s.first == "--no-fusion")
		{
			do_fusion = false;
		}
		else if (s.first == "--display")
		{
			if (s.second == "\"normals\"")
			{
				display_channel = voltu::Channel::kNormals;
			}
			else if (s.second == "\"depth\"")
			{
				display_channel = voltu::Channel::kDepth;
			}
			else if (s.second == "\"corrections\"")
			{
				display_channel = voltu::Channel::kColour;
				show_changes = true;
			}
		}
		else if (s.first == "--no-eval")
		{
			do_eval = false;
		}
		else if (s.first == "--carving")
		{
			do_carving = true;
		}
		else if (s.first == "--frame")
		{
			frameno = std::stoi(s.second);
		}
		else if (s.first == "--source")
		{
			sourceno = std::stoi(s.second);
		}
		else if (s.first == "--iterations")
		{
			iters = std::stoi(s.second);
		}
		else if (s.first[0] != '-')
		{
			paths.push_back(s.first);
		}
	}

	auto vtu = voltu::instance();

	for (const auto &p : paths)
	{
		vtu->open(p);
	}

	while (vtu->listRooms().size() == 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		cout << "Wait room..." << endl;
	}

	auto room = vtu->getRoom(vtu->listRooms().front());

	for (int i=0; i<frameno; ++i)
	{
		room->waitNextFrame(5000);
		room->getFrame();
	}
	auto frame = room->getFrame();

	auto pipe = vtu->createPipeline();
	auto op1 = pipe->appendOperator(voltu::OperatorId::kFusion);
	auto op2 = pipe->appendOperator(voltu::OperatorId::kGTEvaluator);

	op1->property("enabled")->setBool(do_fusion);
	op2->property("enabled")->setBool(do_eval);
	op2->property("show_colour")->setBool(!show_changes);
	op1->property("show_changes")->setBool(show_changes);
	op1->property("visibility_carving")->setBool(do_carving);
	op1->property("mls_iterations")->setInt(iters);

	pipe->submit(frame);
	pipe->waitCompletion(3000);

	auto imgset = frame->getImageSet(display_channel);

	if (imgset.size() == 0)
	{
		cout << "No images!" << endl;
		return -1;
	}

	int srccount = 0;
	for (auto img : imgset)
	{
		if (srccount++ < sourceno) continue;
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

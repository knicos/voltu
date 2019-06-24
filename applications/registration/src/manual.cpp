#include "manual.hpp"
#include "correspondances.hpp"
#include "sfm.hpp"
#include "common.hpp"

#include <loguru.hpp>

#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <map>

using std::string;
using std::vector;
using std::pair;
using std::map;
using std::tuple;

using ftl::net::Universe;
using ftl::rgbd::Source;
using ftl::registration::Correspondances;

using cv::Mat;
using cv::Point;
using cv::Scalar;

using namespace ftl::registration;

using MouseAction = std::function<void(int, int, int, int)>;

static void setMouseAction(const std::string& winName, const MouseAction &action)
{
	cv::setMouseCallback(winName,
						[] (int event, int x, int y, int flags, void* userdata) {
	(*(MouseAction*)userdata)(event, x, y, flags);
	}, (void*)&action);
}

static MouseAction tmouse;
static MouseAction smouse;

void ftl::registration::manual(ftl::Configurable *root) {
	using namespace cv;
	
	Universe *net = ftl::create<Universe>(root, "net");

	net->start();
	net->waitConnections();

	auto sources = ftl::createArray<Source>(root, "sources", net);
 
	int curtarget = 0;
	bool freeze = false;

	vector<Mat> rgb;
	vector<Mat> depth;

	if (sources.size() == 0) return;

	rgb.resize(sources.size());
	depth.resize(sources.size());

	cv::namedWindow("Target", cv::WINDOW_KEEPRATIO);
	cv::namedWindow("Source", cv::WINDOW_KEEPRATIO);

	map<string, Eigen::Matrix4d> oldTransforms;
	ftl::registration::loadTransformations(root->value("output", string("./test.json")), oldTransforms);

	//Correspondances c(sources[targsrc], sources[cursrc]);
	map<string, Correspondances*> corrs;
	ftl::registration::build_correspondances(sources, corrs, root->value("origin", 0), oldTransforms);

	int lastTX = 0;
	int lastTY = 0;
	int lastSX = 0;
	int lastSY = 0;

	tmouse = [&]( int event, int ux, int uy, int) {
		if (event == 1) {   // click
			lastTX = ux;
			lastTY = uy;
		}
	};
	setMouseAction("Target", tmouse);

	smouse = [&]( int event, int ux, int uy, int) {
		if (event == 1) {   // click
			lastSX = ux;
			lastSY = uy;
		}
	};
	setMouseAction("Source", smouse);

	bool depthtoggle = false;
	double lastScore = 0.0;
	bool insearch = false;
	int point_n = -1;

	Correspondances *current = corrs[sources[curtarget]->getURI()];

	while (ftl::running) {
		if (!freeze) {
			// Grab the latest frames from sources
			for (int i=0; i<sources.size(); i++) {
				if (sources[i]->isReady()) {
					sources[i]->grab();
				}
			}
		}

		// Get the raw rgb and depth frames from sources
		Mat ttrgb, trgb, tdepth, ttdepth;
		Mat tsrgb, srgb, sdepth, tsdepth;

		if (current == nullptr) {
			srgb = Mat(Size(640,480), CV_8UC3, Scalar(0,0,0));
			trgb = Mat(Size(640,480), CV_8UC3, Scalar(0,0,0));

			putText(srgb, string("No correspondance for ") + sources[curtarget]->getURI(), Point(10,20), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1);

			if (!trgb.empty()) imshow("Target", trgb);
			if (!srgb.empty()) imshow("Source", srgb);
		} else if (current->source()->isReady() && current->target()->isReady()) {
			current->source()->getFrames(tsrgb, tsdepth);
			current->target()->getFrames(ttrgb, ttdepth);

			tsrgb.copyTo(srgb);
			ttrgb.copyTo(trgb);
			ttdepth.convertTo(tdepth, CV_8U, 255.0f / 10.0f);
			applyColorMap(tdepth, tdepth, cv::COLORMAP_JET);
			tsdepth.convertTo(sdepth, CV_8U, 255.0f / 10.0f);
			applyColorMap(sdepth, sdepth, cv::COLORMAP_JET);

			current->drawTarget(trgb);
			current->drawTarget(tdepth);
			current->drawSource(srgb);
			current->drawSource(sdepth);

			// Add most recent click position
			drawMarker(srgb, Point(lastSX, lastSY), Scalar(0,0,255), MARKER_TILTED_CROSS);
			drawMarker(trgb, Point(lastTX, lastTY), Scalar(0,0,255), MARKER_TILTED_CROSS);
			drawMarker(sdepth, Point(lastSX, lastSY), Scalar(0,0,255), MARKER_TILTED_CROSS);
			drawMarker(tdepth, Point(lastTX, lastTY), Scalar(0,0,255), MARKER_TILTED_CROSS);

			putText(srgb, string("Source: ") + current->source()->getURI(), Point(10,20), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1);
			putText(trgb, string("Target: ") + current->target()->getURI(), Point(10,20), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1);
			putText(srgb, string("Score: ") + std::to_string(lastScore), Point(10,40), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1);
			putText(trgb, string("Score: ") + std::to_string(lastScore), Point(10,40), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1);
			if (freeze) putText(srgb, string("Paused"), Point(10,50), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1);
			if (freeze) putText(trgb, string("Paused"), Point(10,50), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,0,255), 1);

			if (!trgb.empty()) imshow("Target", (depthtoggle) ? tdepth : trgb);
			if (!srgb.empty()) imshow("Source", (depthtoggle) ? sdepth : srgb);
		}
	
		int key = cv::waitKey(20);
		if (key == 27) break;
		else if (key >= 48 && key <= 57) {
			curtarget = key-48;
			if (curtarget >= sources.size()) curtarget = 0;
			current = corrs[sources[curtarget]->getURI()];
			lastScore = 1000.0;
		} else if (key == 'd') {
			depthtoggle = !depthtoggle;
		} else if (key == 'c') {
			current->clear();
			lastScore = 1000.0;
		} else if (key == 'n') {
			point_n++;
		} else if (key == 'p') {
			LOG(INFO) << "Captured..";
			lastScore = 1000.0;
			Mat f1,f2;
			current->capture(f1,f2);
		}else if (key == 'a') {
			Eigen::Matrix4d t;
			if (current->add(lastTX,lastTY,lastSX,lastSY)) {
				lastTX = lastTY = lastSX = lastSY = 0;
				lastScore = current->estimateTransform(t);
				current->setTransform(t);
			}
		} else if (key == 'u') {
			current->removeLast();
			Eigen::Matrix4d t;
			lastScore = current->estimateTransform(t);
			current->setTransform(t);
		} else if (key == 'i') {
			current->icp();
		} else if (key == 's') {
			// Save
			map<string, Eigen::Matrix4d> transforms;
			//transforms[sources[targsrc]->getURI()] = Eigen::Matrix4f().setIdentity();
			//transforms[sources[cursrc]->getURI()] = c.transform();

			for (auto x : corrs) {
				if (x.second == nullptr) {
					transforms[x.first] = Eigen::Matrix4d().setIdentity();
				} else {
					transforms[x.first] = x.second->transform();
				}
			}

			saveTransformations(root->value("output", string("./test.json")), transforms);
			LOG(INFO) << "Saved!";
		} else if (key == 't') {
			current->source()->setPose(current->transform());
		} else if (key == 'g') {
			if (!insearch) {
				insearch = true;

				ftl::pool.push([&lastScore,&insearch,current](int id) {
					LOG(INFO) << "START";
					lastScore = 1000.0;
					do {
						Eigen::Matrix4d tr;
						float s = current->findBestSubset(tr, 20, 100); // (int)(current->screenPoints().size() * 0.4f)
						LOG(INFO) << "SCORE = " << s;
						if (s < lastScore) {
							lastScore = s;
							current->setTransform(tr);
							//current->source()->setPose(current->transform());
						}
					} while (ftl::running && insearch && lastScore > 0.000001);
					insearch = false;
					LOG(INFO) << "DONE: " << lastScore;
				});
				/*for (int i=0; i<feat.size(); i++) {
					auto [sx,sy,tx,ty] = feat[i];
					current->add(tx,ty,sx,sy);
				}
				lastsScore = current->estimateTransform();*/
			} else {
				insearch = false;
			}
		} else if (key == 'f') {
			if (!insearch) {
				Mat f1,f2;
				current->capture(f1,f2);

				vector<tuple<int,int,int,int>> feat;
				if (ftl::registration::featuresSIFT(f1, f2, feat, 10)) {
					for (int j=0; j<feat.size(); j++) {
						auto [sx,sy,tx,ty] = feat[j];
						current->add(tx, ty, sx, sy);
					}
				}

				LOG(INFO) << "Found " << current->screenPoints().size() << " features";
			} else {
				LOG(ERROR) << "Can't add features whilst searching...";
			}
		} else if (key == 32) freeze = !freeze;
	}

	// store transformations in map<string Matrix4f>

	// TODO:	(later) extract features and correspondencies from complete cloud
	//			and do ICP to find best possible transformation

	// saveTransformations(const string &path, map<string, Matrix4f> &data)
}

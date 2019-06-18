#include "manual.hpp"
#include "correspondances.hpp"
#include "sfm.hpp"

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
#include <opencv2/xphoto.hpp>

#include <map>

using std::string;
using std::vector;
using std::pair;
using std::map;
using std::tuple;

using ftl::net::Universe;
using ftl::rgbd::Source;

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

void from_json(nlohmann::json &json, map<string, Eigen::Matrix4f> &transformations) {
	for (auto it = json.begin(); it != json.end(); ++it) {
		Eigen::Matrix4f m;
		auto data = m.data();
		for(size_t i = 0; i < 16; i++) { data[i] = it.value()[i]; }
		transformations[it.key()] = m;
	}
}

static void to_json(nlohmann::json &json, map<string, Eigen::Matrix4f> &transformations) {
	for (auto &item : transformations) {
		auto val = nlohmann::json::array();
		for(size_t i = 0; i < 16; i++) { val.push_back((float) item.second.data()[i]); }
		json[item.first] = val;
	}
}

static bool saveTransformations(const string &path, map<string, Eigen::Matrix4f> &data) {
	nlohmann::json data_json;
	to_json(data_json, data);
	std::ofstream file(path);

	if (!file.is_open()) {
		LOG(ERROR) << "Error writing transformations to file " << path;
		return false;
	}

	file << std::setw(4) << data_json;
	return true;
}

bool loadTransformations(const string &path, map<string, Eigen::Matrix4f> &data) {
	std::ifstream file(path);
	if (!file.is_open()) {
		LOG(ERROR) << "Error loading transformations from file " << path;
		return false;
	}
	
	nlohmann::json json_registration;
	file >> json_registration;
	from_json(json_registration, data);
	return true;
}

static void build_correspondances(const vector<Source*> &sources, map<string, Correspondances*> &cs, int origin, map<string, Eigen::Matrix4f> &old) {
	Correspondances *last = nullptr;

	cs[sources[origin]->getURI()] = nullptr;

	for (int i=origin-1; i>=0; i--) {
		if (last == nullptr) {
			auto *c = new Correspondances(sources[i], sources[origin]);
			last = c;
			cs[sources[i]->getURI()] = c;
			if (old.find(sources[i]->getURI()) != old.end()) {
				c->setTransform(old[sources[i]->getURI()]);
			}
		} else {
			auto *c = new Correspondances(last, sources[i]);
			last = c;
			cs[sources[i]->getURI()] = c;
			if (old.find(sources[i]->getURI()) != old.end()) {
				c->setTransform(old[sources[i]->getURI()]);
			}
		}
	}

	last = nullptr;

	for (int i=origin+1; i<sources.size(); i++) {
		if (last == nullptr) {
			auto *c = new Correspondances(sources[i], sources[origin]);
			last = c;
			cs[sources[i]->getURI()] = c;
		} else {
			auto *c = new Correspondances(last, sources[i]);
			last = c;
			cs[sources[i]->getURI()] = c;
		}
	}
}

void averageDepth(vector<Mat> &in, Mat &out) {
	const int rows = in[0].rows;
	const int cols = in[0].cols;

	// todo: create new only if out is empty (or wrong shape/type)
	out = Mat(rows, cols, CV_32FC1);

	for (int i = 0; i < rows * cols; ++i) {
		float sum = 0;
		int good_values = 0;
		for (int i_in = 0; i_in < in.size(); ++i_in) {
			float d = in[i_in].at<float>(i);
			if (d < 40) {
				good_values++;
				sum += d;
			}
		}

		if (good_values > 0) {
			out.at<float>(i) = sum / (float) good_values;
		} else {
			out.at<float>(i) = 41.0f;
		}
	}
}

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

	map<string, Eigen::Matrix4f> oldTransforms;
	loadTransformations(root->value("output", string("./test.json")), oldTransforms);

	//Correspondances c(sources[targsrc], sources[cursrc]);
	map<string, Correspondances*> corrs;
	build_correspondances(sources, corrs, root->value("origin", 0), oldTransforms);

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

	Correspondances *current = corrs[sources[curtarget]->getURI()];
	
	// Features for current frame...
	vector<tuple<int,int,int,int>> feat;
	vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> pclfeat;

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

			Ptr<xphoto::WhiteBalancer> wb;
			wb = xphoto::createSimpleWB();
			wb->balanceWhite(tsrgb, srgb);
			wb->balanceWhite(ttrgb, trgb);

			// Apply points and labels...
			auto &points = current->screenPoints();
			for (auto &p : points) {
				auto [tx,ty,sx,sy] = p;
				drawMarker(srgb, Point(sx,sy), Scalar(0,255,0), MARKER_TILTED_CROSS);
				drawMarker(sdepth, Point(sx,sy), Scalar(0,255,0), MARKER_TILTED_CROSS);
				drawMarker(trgb, Point(tx,ty), Scalar(0,255,0), MARKER_TILTED_CROSS);
				drawMarker(tdepth, Point(tx,ty), Scalar(0,255,0), MARKER_TILTED_CROSS);
			}

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
			feat.clear();
			pclfeat.clear();
			lastScore = 1000.0;
		} else if (key == 'd') {
			depthtoggle = !depthtoggle;
		} else if (key == 'c') {
			feat.clear();
			pclfeat.clear();
			lastScore = 1000.0;
		}else if (key == 'a') {
			if (current->add(lastTX,lastTY,lastSX,lastSY)) {
				lastTX = lastTY = lastSX = lastSY = 0;
				lastScore = current->estimateTransform();
			}
		} else if (key == 'u') {
			current->removeLast();
			lastScore = current->estimateTransform();
		} else if (key == 's') {
			// Save
			map<string, Eigen::Matrix4f> transforms;
			//transforms[sources[targsrc]->getURI()] = Eigen::Matrix4f().setIdentity();
			//transforms[sources[cursrc]->getURI()] = c.transform();

			for (auto x : corrs) {
				if (x.second == nullptr) {
					transforms[x.first] = Eigen::Matrix4f().setIdentity();
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
				LOG(INFO) << "Features matched = " << feat.size();

				ftl::pool.push([&lastScore,&insearch,current,&feat,&pclfeat](int id) {
					LOG(INFO) << "START";
					lastScore = 1000.0;
					do {
						Eigen::Matrix4f tr;
						float s = current->findBest(tr, feat, pclfeat, 20, 100);
						//LOG(INFO) << "SCORE = " << s;
						if (s < lastScore) {
							lastScore = s;
							current->setTransform(tr);
							current->source()->setPose(current->transform());
						}
					} while (ftl::running && insearch && lastScore > 0.000001);
					insearch = false;
					LOG(INFO) << "DONE: " << lastScore;
				});
				/*for (int i=0; i<feat.size(); i++) {
					auto [sx,sy,tx,ty] = feat[i];
					current->add(tx,ty,sx,sy);
				}
				lastScore = current->estimateTransform();*/
			} else {
				insearch = false;
			}
		} else if (key == 'f') {
			Mat f1, d1, f2, d2;

			size_t buffer_size = 10;
			size_t buffer_i = 0;
			vector<vector<Mat>> buffer(2, vector<Mat>(buffer_size));

			for (size_t i = 0; i < buffer_size; ++i) {
				current->source()->grab();
				current->target()->grab();
				current->source()->getFrames(f1, d1);
				current->target()->getFrames(f2, d2);
				buffer[0][i] = d1;
				buffer[1][i] = d2;

				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
			averageDepth(buffer[0], d1);
			averageDepth(buffer[1], d2);
			Mat d1_v, d2_v;
			d1.convertTo(d1_v, CV_8U, 255.0f / 10.0f);
			d2.convertTo(d2_v, CV_8U, 255.0f / 10.0f);
			applyColorMap(d1_v, d1_v, cv::COLORMAP_JET);
			applyColorMap(d2_v, d2_v, cv::COLORMAP_JET);

			cv::imshow("smooth d1", d1_v);
			cv::imshow("smooth d2", d2_v);

			Ptr<xphoto::WhiteBalancer> wb;
			wb = xphoto::createSimpleWB();
			wb->balanceWhite(f1, f1);
			wb->balanceWhite(f2, f2);

			vector<tuple<int,int,int,int>> tfeat;
			if (!insearch && ftl::registration::featuresSIFT(f1, f2, tfeat, 10)) {
				vector<tuple<int,int,int,int>> tfeat2;
				for (int j=0; j<tfeat.size(); j++) {
					auto [sx,sy,tx,ty] = tfeat[j];
					tfeat2.push_back(std::make_tuple(tx,ty,sx,sy));
				}
				current->setPoints(tfeat2);
				feat.insert(feat.end(), tfeat.begin(), tfeat.end());

				//vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> tpclfeat;
				current->convertToPCL(tfeat,pclfeat);
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

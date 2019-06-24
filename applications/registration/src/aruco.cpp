#include "aruco.hpp"
#include "common.hpp"
#include "correspondances.hpp"

#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>

#include <opencv2/aruco.hpp>

using ftl::registration::aruco;
using ftl::registration::Correspondances;
using std::map;
using std::string;
using std::vector;
using ftl::Configurable;
using ftl::net::Universe;
using ftl::rgbd::Source;
using std::pair;
using std::make_pair;

void ftl::registration::aruco(ftl::Configurable *root) {
	using namespace cv;
	
	Universe *net = ftl::create<Universe>(root, "net");

	net->start();
	net->waitConnections();

	auto sources = ftl::createArray<Source>(root, "sources", net);
 
	int curtarget = 0;
	bool freeze = false;
	bool depthtoggle = false;
	double lastScore = 1000.0;

	if (sources.size() == 0) return;

	cv::namedWindow("Target", cv::WINDOW_KEEPRATIO);
	cv::namedWindow("Source", cv::WINDOW_KEEPRATIO);

	map<string, Eigen::Matrix4d> oldTransforms;
	ftl::registration::loadTransformations(root->value("output", string("./test.json")), oldTransforms);

	//Correspondances c(sources[targsrc], sources[cursrc]);
	map<string, Correspondances*> corrs;
	ftl::registration::build_correspondances(sources, corrs, root->value("origin", 0), oldTransforms);

	Correspondances *current = corrs[sources[curtarget]->getURI()];

	map<int, vector<Point2f>> targ_corners;
	map<int, vector<Point2f>> src_corners;
	map<int, pair<Vec3d,Vec3d>> targ_trans;
	map<int, pair<Vec3d,Vec3d>> src_trans;

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

			vector<int> mids_targ, mids_src;
			vector<vector<Point2f>> mpoints_targ, mpoints_src;
			Ptr<aruco::DetectorParameters> parameters(new aruco::DetectorParameters);
			auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

			aruco::detectMarkers(trgb, dictionary, mpoints_targ, mids_targ, parameters);
			aruco::drawDetectedMarkers(trgb, mpoints_targ, mids_targ);

			// Cache the marker positions
			for (size_t i=0; i<mids_targ.size(); i++) {
				targ_corners[mids_targ[i]] = mpoints_targ[i];
			}

			vector<Vec3d> rvecs, tvecs;
			cv::Mat distCoef(cv::Size(14,1), CV_64F, cv::Scalar(0.0));
			cv::Mat targ_cam = current->target()->cameraMatrix();
			aruco::estimatePoseSingleMarkers(mpoints_targ, 0.1, targ_cam, distCoef, rvecs, tvecs);
			for (size_t i=0; i<rvecs.size(); i++) {
				targ_trans[mids_targ[i]] = make_pair(tvecs[i], rvecs[i]);
				aruco::drawAxis(trgb, current->target()->cameraMatrix(), distCoef, rvecs[i], tvecs[i], 0.1);
			}

			aruco::detectMarkers(srgb, dictionary, mpoints_src, mids_src, parameters);
			aruco::drawDetectedMarkers(srgb, mpoints_src, mids_src);

			rvecs.clear();
			tvecs.clear();
			cv::Mat src_cam = current->source()->cameraMatrix();
			aruco::estimatePoseSingleMarkers(mpoints_src, 0.1, src_cam, distCoef, rvecs, tvecs);
			for (size_t i=0; i<rvecs.size(); i++) {
				src_trans[mids_src[i]] = make_pair(tvecs[i], rvecs[i]);
				aruco::drawAxis(srgb, current->source()->cameraMatrix(), distCoef, rvecs[i], tvecs[i], 0.1);
			}

			// Cache the marker positions
			for (size_t i=0; i<mids_src.size(); i++) {
				src_corners[mids_src[i]] = mpoints_src[i];
			}

			current->drawTarget(trgb);
			current->drawTarget(tdepth);
			current->drawSource(srgb);
			current->drawSource(sdepth);

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
			//lastScore = 1000.0;
		} else if (key == 'd') {
			depthtoggle = !depthtoggle;
		} else if (key == 'i') {
			current->icp();
		} else if (key == 'e') {
			vector<Vec3d> targfeat;
			vector<Vec3d> srcfeat;

			for (auto i : targ_trans) {
				auto si = src_trans.find(i.first);
				if (si != src_trans.end()) {
					//Vec3d dt = std::get<0>(si->second) - std::get<0>(i.second);
					targfeat.push_back(std::get<0>(i.second));
					srcfeat.push_back(std::get<0>(si->second));
				}
			}

			Eigen::Matrix4d t;
			lastScore = current->estimateTransform(t, srcfeat, targfeat, true);
			current->setTransform(t);
			//lastScore = current->icp();
		} else if (key == 'f') {
			Mat f1,f2;
			current->capture(f1,f2);

			LOG(INFO) << "Captured frames";

			// Convert matching marker corners into correspondence points
			for (auto i : targ_corners) {
				auto si = src_corners.find(i.first);
				if (si != src_corners.end()) {
					for (size_t j=0; j<4; ++j) {
						current->add(i.second[j].x, i.second[j].y, si->second[j].x, si->second[j].y);
					}
				}
			}

			LOG(INFO) << "Estimating transform...";

			Eigen::Matrix4d t;
			lastScore = current->estimateTransform(t);
			current->setTransform(t);
			lastScore = current->icp();
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
		} else if (key == 32) freeze = !freeze;
	}
}

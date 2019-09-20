#pragma once
#ifndef _FTL_RGBD_SNAPSHOT_HPP_
#define _FTL_RGBD_SNAPSHOT_HPP_

#include <loguru.hpp>
#include <thread>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/camera.hpp>

#include <atomic>
#include <archive.h>
#include <archive_entry.h>

namespace ftl {
namespace rgbd {

// FIXME: NOT thread safe

class SnapshotWriter {
public:
	explicit SnapshotWriter(const std::string &filename);
	~SnapshotWriter();
	
	void addSource(const std::string &id, const ftl::rgbd::Camera &params, const Eigen::Matrix4d &extrinsic);
	void addSource(const std::string &id, const std::vector<double> &params, const cv::Mat &extrinsic);
	bool addRGBD(size_t source, const cv::Mat &rgb, const cv::Mat &depth, uint64_t time=0);

	bool addMat(const std::string &name, const cv::Mat &mat, const std::string &format, const std::vector<int> &params);
	bool addFile(const std::string &name, const std::vector<uchar> &buf);
	bool addFile(const std::string &name, const uchar *buf, const size_t len);
	
	void writeIndex();

private:
	std::vector<std::string> sources_;
	std::vector<std::vector<double>> params_;
	std::vector<cv::Mat> extrinsic_;
	std::vector<size_t> frame_idx_;
	std::vector<std::vector<std::string>> fname_rgb_;
	std::vector<std::vector<std::string>> fname_depth_;

	struct archive *archive_ = nullptr;
	struct archive_entry *entry_ = nullptr;
};

class SnapshotStreamWriter {
public:
	SnapshotStreamWriter(const std::string &filename, int delay);
	~SnapshotStreamWriter();
	void addSource(ftl::rgbd::Source* src);
	void start();
	void stop();

private:
	std::atomic<bool> run_;
	bool finished_;
	int delay_;

	std::vector<ftl::rgbd::Source*> sources_;
	SnapshotWriter writer_;
	std::thread thread_;

	void run();
};

class Snapshot {
public:
	size_t getSourcesCount();
	size_t getFramesCount();
	
	std::string getSourceURI(size_t camera);
	ftl::rgbd::Camera getParameters(size_t camera);
	void getPose(size_t camera, cv::Mat &out);
	void getPose(size_t camera, Eigen::Matrix4d &out);

	void getLeftRGB(size_t camera, size_t frame, cv::Mat &data);
	void getLeftDepth(size_t camera, size_t frame, cv::Mat &data);

	size_t n_frames;
	size_t n_cameras;

	std::vector<std::string> sources;
	std::vector<ftl::rgbd::Camera> parameters;
	std::vector<cv::Mat> extrinsic;
	std::vector<std::vector<cv::Mat>> rgb_left;
	std::vector<std::vector<cv::Mat>> depth_left;
};

class SnapshotReader {
public:
	explicit SnapshotReader(const std::string &filename);
	~SnapshotReader();

	Snapshot readArchive();

private:
	bool readEntry(std::vector<uchar> &data);

	bool getDepth(const std::string &name, cv::Mat &data);
	bool getRGB(const std::string &name, cv::Mat &data);

	std::map<std::string, std::vector<uchar>> files_;
	struct archive *archive_;
	struct archive_entry *entry_;
};

};
};

#endif  // _FTL_RGBD_SNAPSHOT_HPP_

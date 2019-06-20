#pragma once
#ifndef _FTL_RGBD_SNAPSHOT_HPP_
#define _FTL_RGBD_SNAPSHOT_HPP_

#include <loguru.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <ftl/rgbd/camera.hpp>

#include <archive.h>
#include <archive_entry.h>

namespace ftl {
namespace rgbd {

// FIXME: NOT thread safe

class SnapshotWriter {
public:
	explicit SnapshotWriter(const std::string &filename);
	~SnapshotWriter();
	
	bool addCameraRGBD(const std::string &name, const cv::Mat &rgb, const cv::Mat &depth, const Eigen::Matrix4d &pose, const ftl::rgbd::Camera &params);
	bool addMat(const std::string &name, const cv::Mat &mat, const std::string &format="tiff");
	bool addEigenMatrix4d(const std::string &name, const Eigen::Matrix4d &m, const std::string &format="pfm");
	bool addFile(const std::string &name, const std::vector<uchar> &buf);
	bool addFile(const std::string &name, const uchar *buf, const size_t len);

private:
	struct archive *archive_;
	struct archive_entry *entry_;
};

struct SnapshotEntry {
	cv::Mat rgb;
	cv::Mat depth;
	Eigen::Matrix4d pose;
	ftl::rgbd::Camera params;
	uint status;
	SnapshotEntry() : status(1+2+4+8) {};
};

class SnapshotReader {
public:
	explicit SnapshotReader(const std::string &filename);
	~SnapshotReader();
	
	bool getCameraRGBD(const std::string &id, cv::Mat &rgb, cv::Mat &depth, Eigen::Matrix4d &pose, ftl::rgbd::Camera &params);
	std::vector<std::string> getIds();

private:
	SnapshotEntry& getEntry(const std::string &id);
	bool readEntry(std::vector<uchar> &data);
	bool readArchive();

	std::map<std::string, SnapshotEntry> data_;
	struct archive *archive_;
	struct archive_entry *entry_;
};


};
};

#endif  // _FTL_RGBD_SNAPSHOT_HPP_

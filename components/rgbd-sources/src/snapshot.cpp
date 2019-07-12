#include <ftl/rgbd/snapshot.hpp>

#include <nlohmann/json.hpp>

using namespace ftl::rgbd;

using cv::Mat;
using Eigen::Matrix4d;

using cv::imencode;
using cv::imdecode;

using std::string;
using std::vector;

// TODO: move to camera_params
using ftl::rgbd::Camera;

void to_json(nlohmann::json& j, const Camera &p) {
	j = nlohmann::json{
		{"fx", p.fx},
		{"fy", p.fy},
		{"cx", p.cx},
		{"cy", p.cy},
		{"width", p.width},
		{"height", p.height},
		{"minDepth", p.minDepth},
		{"maxDepth", p.maxDepth}
	};
}

void from_json(const nlohmann::json& j, Camera &p) {
	j.at("fx").get_to(p.fx);
	j.at("fy").get_to(p.fy);
	j.at("cx").get_to(p.cx);
	j.at("cy").get_to(p.cy);
	j.at("width").get_to(p.width);
	j.at("height").get_to(p.height);
	j.at("minDepth").get_to(p.minDepth);
	j.at("maxDepth").get_to(p.maxDepth);
}
//

SnapshotWriter::SnapshotWriter(const string &filename) {
	archive_ = archive_write_new();
	if (!archive_) goto error3;
	entry_ = archive_entry_new();
	if (!entry_) goto error2;

	if (archive_write_set_format_pax_restricted(archive_) != ARCHIVE_OK)
		goto error1;
	
	// todo make compression optional (or remove it)
	if (archive_write_add_filter_gzip(archive_) != ARCHIVE_OK)
		goto error1;
	if (archive_write_open_filename(archive_, filename.c_str()) != ARCHIVE_OK)
		goto error1;
	
	return;
	
	error1:
	archive_entry_free(entry_);
	error2:
	LOG(ERROR) << archive_error_string(archive_);
	archive_write_free(archive_);
	error3:
	// throw exception; otherwise destructor might be called
	throw std::runtime_error("SnapshotWriter failed");
}

SnapshotWriter::~SnapshotWriter() {
	archive_entry_free(entry_);
	archive_write_close(archive_);
	archive_write_free(archive_);
}

bool SnapshotWriter::addFile(const string &name, const uchar *buf, const size_t len) {
	archive_entry_clear(entry_);
	archive_entry_set_pathname(entry_, name.c_str());
	archive_entry_set_size(entry_, len);
	archive_entry_set_filetype(entry_, AE_IFREG);
	archive_entry_set_perm(entry_, 0644);

	size_t l = len;
	if (archive_write_header(archive_, entry_) != ARCHIVE_OK) goto error;
	
	while (true) {
		ssize_t ret_w = archive_write_data(archive_, buf, l);
		if (ret_w == 0) { break; }
		if (ret_w < 0) { goto error; }
		else {
			l -= ret_w;
			buf = buf + ret_w;
		}
	}
	return true;
	
	error:
	LOG(ERROR) << archive_error_string(archive_);
	return false;
}

bool SnapshotWriter::addFile(const string &name, const vector<uchar> &buf) {
	return addFile(name, buf.data(), buf.size());
}

bool SnapshotWriter::addMat(const string &name, const Mat &mat, const std::string &format) {
	if (mat.rows == 0 || mat.cols == 0) {
		LOG(ERROR) << "empty mat";
		return false;
	}

	vector<uchar> buf;
	vector<int> params;
	bool retval = true;
	retval &= imencode("." + format, mat, buf, params);
	retval &= addFile(name + "." + format, buf);
	return retval;
}

bool SnapshotWriter::addEigenMatrix4d(const string &name, const Matrix4d &m, const string &format) {
	Mat tmp;
	cv::eigen2cv(m, tmp);
	return addMat(name, tmp, format);
}

bool SnapshotWriter::addCameraParams(const string &name, const Matrix4d &pose, const Camera &params) {
	bool retval = true;
	retval &= addEigenMatrix4d(name + "-POSE", pose);

	nlohmann::json j;
	to_json(j, params);
	string str_params = j.dump();
	retval &= addFile(name + "-PARAMS.json", (uchar*) str_params.c_str(), str_params.size());
	return retval;
}

bool SnapshotWriter::addCameraRGBD(const string &name, const Mat &rgb, const Mat &depth) {
	bool retval = true;
	cv::Mat tdepth;
	depth.convertTo(tdepth, CV_16UC1, 1000.0f);
	retval &= addMat(name + "-RGB", rgb, "jpg");
	retval &= addMat(name + "-D", tdepth, "png");
	return retval;
}

SnapshotStreamWriter::SnapshotStreamWriter(const string &filename, int delay) : 
		run_(false), finished_(false), delay_(delay), writer_(filename) {
		DCHECK(delay > 0);
	}

SnapshotStreamWriter::~SnapshotStreamWriter() {

}

void SnapshotStreamWriter::addSource(ftl::rgbd::Source *src) {
	writer_.addCameraParams(std::to_string(sources_.size()), src->getPose(), src->parameters());
	sources_.push_back(src);
}

void SnapshotStreamWriter::run() {
	vector<Mat> rgb(sources_.size());
	vector<Mat> depth(sources_.size());

	while(run_) {
		auto t_now = std::chrono::system_clock::now();
		auto t_wakeup = t_now + std::chrono::milliseconds(delay_);

		auto duration = t_now.time_since_epoch();
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

		for(size_t i = 0; i < sources_.size(); ++i) {
			sources_[i]->getFrames(rgb[i], depth[i]);
		}

		for(size_t i = 0; i < sources_.size(); ++i) {
			writer_.addCameraRGBD(std::to_string(ms) + "-" + std::to_string(i), rgb[i], depth[i]);
		}

		std::this_thread::sleep_until(t_wakeup);
	}

	run_ = false;
	finished_ = true;
}

void SnapshotStreamWriter::start() {
	if (run_ || finished_) return;
	run_ = true;
	thread_ = std::thread([this] { run(); });
}


void SnapshotStreamWriter::stop() {
	bool wasrunning = run_;
	run_ = false;
	if (wasrunning) thread_.join();
}



SnapshotReader::SnapshotReader(const string &filename) {
	archive_ = archive_read_new();
	if (!archive_) goto error2;
	archive_read_support_format_all(archive_);
	archive_read_support_filter_all(archive_);

	if (archive_read_open_filename(archive_, filename.c_str(), 4096) != ARCHIVE_OK)
		goto error1;
	
	readArchive();
	return;
	
	error1:
	LOG(ERROR) << archive_error_string(archive_);
	archive_read_free(archive_);
	error2:
	// throw exception; otherwise destructor might be called
	throw std::runtime_error("SnapshotReader failed");
}

SnapshotReader::~SnapshotReader() {
	archive_read_free(archive_);
}

bool SnapshotReader::readEntry(vector<uchar> &data) {
	if (!archive_entry_size_is_set(entry_)) {
		LOG(ERROR) << "entry size unknown";
		return false;
	}

	size_t size = archive_entry_size(entry_);
	size_t size_read = 0;
	data.resize(size);
	uchar *buf = data.data();

	while(true) {
		ssize_t size_read_new = archive_read_data(archive_, buf + size_read, size - size_read);
		if (size_read_new < 0) return false;
		if (size_read_new == 0) return true;
		size_read += size_read_new;
	}
}

SnapshotEntry& SnapshotReader::getEntry(const string &id) {
	/*if (data_.find(id) == data_.end()) {
		data_.emplace(id, SnapshotEntry{});
	}*/
	return data_[id];
}

/* read all entries to data_ */
bool SnapshotReader::readArchive() {
	int retval = ARCHIVE_OK;
	vector<uchar> data;

	while((retval = archive_read_next_header(archive_, &entry_)) == ARCHIVE_OK) {
		string path = string(archive_entry_pathname(entry_));
		if (path.rfind("-") == string::npos) {
			LOG(WARNING) << "unrecognized file " << path;
			continue;
		}
		string id = path.substr(0, path.find("-"));

		SnapshotEntry &snapshot = getEntry(id);

		// TODO: verify that input is valid
		// TODO: check that earlier results are not overwritten (status)

		if (path.rfind("-RGB.") != string::npos) {
			if (!readEntry(data)) continue;
			snapshot.rgb = cv::imdecode(data, cv::IMREAD_COLOR);
			snapshot.status &= ~1;
		}
		else if (path.rfind("-D.") != string::npos) {
			if (!readEntry(data)) continue;
			snapshot.depth = cv::imdecode(data, cv::IMREAD_ANYDEPTH);
			snapshot.status &= ~(1 << 1);
		}
		else if (path.rfind("-POSE.pfm") != string::npos) {
			if (!readEntry(data)) continue;
			Mat m_ = cv::imdecode(Mat(data), 0);
			if ((m_.rows != 4) || (m_.cols != 4)) continue;
			cv::Matx44d pose_(m_);
			cv::cv2eigen(pose_, snapshot.pose);
			snapshot.status &= ~(1 << 2);
		}
		else if (path.rfind("-PARAMS.json") != string::npos) {
			if (!readEntry(data)) continue;
			nlohmann::json j = nlohmann::json::parse(string((const char*) data.data(), data.size()));
			from_json(j, snapshot.params);
			snapshot.status &= ~(1 << 3);
		}
		else {
			LOG(WARNING) << "unknown file " << path;
		}
	}
	
	if (retval != ARCHIVE_EOF) {
		LOG(ERROR) << archive_error_string(archive_);
		return false;
	}
	
	return true;
}

vector<string> SnapshotReader::getIds() {
	vector<string> res;
	res.reserve(data_.size());
	for(auto itr = data_.begin(); itr != data_.end(); ++itr) {
		res.push_back(itr->first);
	}
	return res;
}

bool SnapshotReader::getCameraRGBD(const string &id, Mat &rgb, Mat &depth,
							 Matrix4d &pose, Camera &params) {
	if (data_.find(id) == data_.end()) {
		LOG(ERROR) << "entry not found: " << id;
		return false;
	}

	SnapshotEntry item = getEntry(id);

	if (item.status != 0) {
		LOG(ERROR) << "entry incomplete: " << id;
	}

	rgb = item.rgb;
	depth = item.depth;
	params = item.params;
	pose = item.pose;
	return true;
}
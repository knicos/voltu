#include <ftl/rgbd/snapshot.hpp>

#include <nlohmann/json.hpp>

using namespace ftl::rgbd;

using cv::Mat;
using Eigen::Matrix4d;

using cv::imencode;
using cv::imdecode;

using std::string;
using std::vector;

using cv::FileStorage;

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
/*
Mat getCameraMatrix(const ftl::rgbd::Camera &parameters) {
	Mat m = (cv::Mat_<double>(3,3) << parameters.fx, 0.0, -parameters.cx, 0.0, parameters.fy, -parameters.cy, 0.0, 0.0, 1.0);
	return m;
}
*/
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
	if (archive_) writeIndex();
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

bool SnapshotWriter::addMat(const string &name, const Mat &mat, const std::string &format, const vector<int> &params) {
	if (mat.rows == 0 || mat.cols == 0) {
		LOG(ERROR) << "empty mat";
		return false;
	}

	vector<uchar> buf;
	bool retval = true;
	retval &= imencode("." + format, mat, buf, params);
	retval &= addFile(name + "." + format, buf);
	return retval;
}

void SnapshotWriter::addSource(const std::string &id, const vector<double> &params, const cv::Mat &extrinsic) {
	frame_idx_.push_back(0);
	sources_.push_back(id);
	params_.push_back(params);
	extrinsic_.push_back(extrinsic);
	fname_rgb_.emplace_back();
	fname_depth_.emplace_back();
}

void SnapshotWriter::addSource(const std::string &id, const ftl::rgbd::Camera &params, const Eigen::Matrix4d &extrinsic) {
	vector<double> params_vec;
	Mat extrinsic_cv;
	cv::eigen2cv(extrinsic, extrinsic_cv);
	params_vec.push_back(params.fx);
	params_vec.push_back(params.fy);
	params_vec.push_back(params.cx);
	params_vec.push_back(params.cy);
	params_vec.push_back(params.width);
	params_vec.push_back(params.height);
	params_vec.push_back(params.minDepth);
	params_vec.push_back(params.maxDepth);
	params_vec.push_back(params.baseline);
	params_vec.push_back(params.doffs);
	addSource(id, params_vec, extrinsic_cv);
}


bool SnapshotWriter::addRGBD(size_t source, const cv::Mat &rgb, const cv::Mat &depth, uint64_t time) {
	// TODO: png option
	if (time != 0) { LOG(WARNING) << "time parameter not used (not implemented)"; }

	bool retval = true;
	string fname = std::to_string(source) + "/" + std::to_string(frame_idx_[source]++);
	
	fname_rgb_[source].push_back("RGB" + fname + ".jpg");
	retval &= addMat("RGB" + fname, rgb, "jpg", {});

	fname_depth_[source].push_back("DEPTH" + fname + ".tiff");
	retval &= addMat("DEPTH" + fname, depth, "tiff", {});

	return retval;
}

void SnapshotWriter::writeIndex() {
	FileStorage fs(".yml", FileStorage::WRITE + FileStorage::MEMORY);

	vector<string> channels = {"time", "rgb_left", "depth_left"};

	fs << "sources" << sources_;
	fs << "params" <<params_;
	fs << "extrinsic" << extrinsic_;
	fs << "channels" << channels;
	
	fs << "rgb_left" << fname_rgb_;
	fs << "depth_left" << fname_depth_;

	string buf = fs.releaseAndGetString();
	addFile("index.yml", (uchar*) buf.c_str(), buf.length());

	archive_entry_free(entry_);
	archive_write_close(archive_);
	archive_write_free(archive_);
	archive_ = nullptr;
	entry_ = nullptr;
}

SnapshotStreamWriter::SnapshotStreamWriter(const string &filename, int delay) : 
		run_(false), finished_(false), delay_(delay), writer_(filename) {
		DCHECK(delay > 0);
	}

SnapshotStreamWriter::~SnapshotStreamWriter() {
	
}

void SnapshotStreamWriter::addSource(ftl::rgbd::Source *src) {
	writer_.addSource(src->getURI(), src->parameters(), src->getPose());
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

		bool good = true;
		for(size_t i = 0; i < sources_.size(); ++i) {
			sources_[i]->getFrames(rgb[i], depth[i]);
			good &= !rgb[i].empty() && !depth[i].empty();
		}

		if (!good) {
			LOG(WARNING) << "Missing frames";
			continue;
		}

		for(size_t i = 0; i < sources_.size(); ++i) {
			writer_.addRGBD(i, rgb[i], depth[i]);
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

size_t Snapshot::getSourcesCount() { return sources.size(); }
size_t Snapshot::getFramesCount() { return depth_left[0].size(); }
	
string Snapshot::getSourceURI(size_t camera) { return sources[camera]; }
ftl::rgbd::Camera Snapshot::getParameters(size_t camera) { return parameters[camera]; }
void Snapshot::getPose(size_t camera, cv::Mat &out) { out = extrinsic[camera]; }
void Snapshot::getPose(size_t camera, Eigen::Matrix4d &out) {
	Mat mat;
	getPose(camera, mat);
	cv::cv2eigen(mat, out);
}
void Snapshot::getLeftRGB(size_t camera, size_t frame, cv::Mat &data) { data = rgb_left[camera][frame]; }
void Snapshot::getLeftDepth(size_t camera, size_t frame, cv::Mat &data) { data = depth_left[camera][frame]; }

SnapshotReader::SnapshotReader(const string &filename) {
	archive_ = archive_read_new();
	int retval = ARCHIVE_OK;
	string msg;

	if (!archive_) goto error2;
	archive_read_support_format_all(archive_);
	archive_read_support_filter_all(archive_);

	if (archive_read_open_filename(archive_, filename.c_str(), 4096) != ARCHIVE_OK)
		goto error1;
	
	while((retval = archive_read_next_header(archive_, &entry_)) == ARCHIVE_OK) {
		string path = string(archive_entry_pathname(entry_));
		vector<uchar> data;
		
		if (readEntry(data)) { files_[path] = data; }
	}

	if (retval != ARCHIVE_EOF) { goto error1; }

	return;
	
	error1:
	msg = archive_error_string(archive_);
	archive_read_free(archive_);
	throw std::runtime_error(msg);

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

bool SnapshotReader::getDepth(const std::string &name, cv::Mat &data) {
	if (files_.find(name) == files_.end()) {
		LOG(ERROR) << name << " not found in archive";
		return false;
	}

	const vector<uchar> &data_raw = files_[name];
	const string ext = name.substr(name.find_last_of(".") + 1);

	if (ext == "tiff") {
		data = cv::imdecode(data_raw, cv::IMREAD_ANYDEPTH);
	}
	else if (ext == "png") {
		data = cv::imdecode(data_raw, cv::IMREAD_ANYDEPTH);
		data.convertTo(data, CV_32FC1, 1.0f / 1000.0f);
	}
	else {
		LOG(ERROR) << "Unsupported file extension for depth image: " << ext;
		return false;
	}

	if (data.empty()) {
		LOG(ERROR) << "Error decoding file: " << name;
		return false;
	}
	
	return true;
}

bool SnapshotReader::getRGB(const std::string &name, cv::Mat &data) {
	if (files_.find(name) == files_.end()) {
		LOG(ERROR) << name << " not found in archive";
		return false;
	}

	const vector<uchar> &data_raw = files_[name];
	const string ext = name.substr(name.find_last_of(".") + 1);
	
	if (!(ext == "png" || ext == "jpg")) { 
		LOG(ERROR) << "Unsupported file extension for depth image: " << ext;
		return false;
	}

	data = cv::imdecode(data_raw, cv::IMREAD_COLOR);

	if (data.empty()) {
		LOG(ERROR) << "Error decoding file: " << name;
		return false;
	}
	
	return true;
}

Snapshot SnapshotReader::readArchive() {
	Snapshot result;
	
	if (files_.find("index.yml") != files_.end()) {
		LOG(INFO) << "Using new format snapshot archive";
		string input;
		{
			vector<uchar> data = files_["index.yml"]; 
			input = string((char*) data.data(), data.size());
		}
		FileStorage fs(input, FileStorage::READ | FileStorage::MEMORY);
		
		vector<string> &sources = result.sources;
		vector<ftl::rgbd::Camera> &params = result.parameters;
		vector<Mat> &extrinsic = result.extrinsic;
		
		vector<vector<Mat>> &rgb_left = result.rgb_left;
		vector<vector<Mat>> &depth_left = result.depth_left;

		vector<string> channels;

		fs["sources"] >> sources;
		fs["extrinsic"] >> extrinsic;
		fs["channels"] >> channels;

		cv::FileNode fn;
		fn = fs["params"];
		for (cv::FileNodeIterator it = fn.begin(); it != fn.end(); it++) {
			vector<double> p;
			*it >> p;

			ftl::rgbd::Camera camera;
			camera.fx = p[0];
			camera.fy = p[1];
			camera.cx = p[2];
			camera.cy = p[3];
			camera.width = p[4];
			camera.height = p[5];
			camera.minDepth = p[6];
			camera.maxDepth = p[7];
			camera.baseline = p[8];
			camera.doffs = p[9];
			params.push_back(camera);
		}

		vector<string> files;
		for (auto const &channel : channels) {
			files.clear();

			if (channel == "time") {
				//fs["time"] >> times;
			}
			else if (channel == "rgb_left") {
				fn = fs["rgb_left"];
				files.clear();
				for (cv::FileNodeIterator it = fn.begin(); it != fn.end(); it++) {
					*it >> files;
					auto &images = rgb_left.emplace_back();
					for (const string& file : files) {
						Mat &img = images.emplace_back();
						getRGB(file, img);
					}
				}
			}
			else if (channel == "depth_left") {
				fn = fs["depth_left"];
				files.clear();
				for (cv::FileNodeIterator it = fn.begin(); it != fn.end(); it++) {
					*it >> files;
					auto &images = depth_left.emplace_back();
					for (const string& file : files) {
						Mat &img = images.emplace_back();
						getDepth(file, img);
					}
				}
			}
			else {
				LOG(ERROR) << "Unsupported channel: " << channel;
			}
		}

		fs.release();
	}
	else {
		LOG(INFO) << "Using old format snapshot archive";

		result.n_cameras = 1;
		result.n_frames = 1;
		Mat &rgb = result.rgb_left.emplace_back().emplace_back();
		Mat &depth = result.depth_left.emplace_back().emplace_back();
		Mat &pose = result.extrinsic.emplace_back();
		Camera &params = result.parameters.emplace_back();

		for (auto const& [path, data] : files_) {
			if (path.rfind("-") == string::npos) {
				LOG(WARNING) << "unrecognized file " << path;
				continue;
			}
			string id = path.substr(0, path.find("-"));

			// TODO: verify that input is valid
			// TODO: check that earlier results are not overwritten (status)
			
			if (path.rfind("-RGB.") != string::npos) {
				getRGB(path, rgb);
			}
			else if (path.rfind("-D.") != string::npos) {
				getDepth(path, depth);
			}
			else if (path.rfind("-POSE.pfm") != string::npos) {
				Mat m_ = cv::imdecode(Mat(data), cv::IMREAD_ANYDEPTH);
				if ((m_.rows != 4) || (m_.cols != 4)) continue;
				cv::Matx44d pose_(m_);
				pose = m_;
			}
			else if (path.rfind("-PARAMS.json") != string::npos) {
				nlohmann::json j = nlohmann::json::parse(string((const char*) data.data(), data.size()));
				from_json(j, params);
			}
			else {
				LOG(WARNING) << "unknown file " << path;
			}
		}
	}

	return result;
}

#include <ftl/configuration.hpp>
#include <ftl/streams/filestream.hpp>
#include <ftl/operators/disparity.hpp>
#include <ftl/codecs/opencv_encoder.hpp>
#include <ftl/streams/injectors.hpp>

#include <ftl/data/framepool.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <nlohmann/json.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::codecs::Channel;
using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using std::string;

static bool loadMiddleburyCalib(const std::string &filename, ftl::rgbd::Camera &p1, ftl::rgbd::Camera &p2, int &ndisp, double scaling) {
	FILE* fp = fopen(filename.c_str(), "r");
	
	float cam0[3][3] = {};
	float cam1[3][3];
	float doffs = 0.0f;
	float baseline = 0.0f;
	int width = 0;
	int height = 0;
	//int ndisp;
	int isint;
	int vmin;
	int vmax;
	float dyavg;
	float dymax;

	if (fp != nullptr) {
		char buff[512];
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "cam0 = [%f %f %f; %f %f %f; %f %f %f]\n", &cam0[0][0], &cam0[0][1], &cam0[0][2], &cam0[1][0], &cam0[1][1], &cam0[1][2], &cam0[2][0], &cam0[2][1], &cam0[2][2]);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "cam1 = [%f %f %f; %f %f %f; %f %f %f]\n", &cam1[0][0], &cam1[0][1], &cam1[0][2], &cam1[1][0], &cam1[1][1], &cam1[1][2], &cam1[2][0], &cam1[2][1], &cam1[2][2]);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "doffs = %f\n", &doffs);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "baseline = %f\n", &baseline);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "width = %d\n", &width);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "height = %d\n", &height);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "ndisp = %d\n", &ndisp);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "isint = %d\n", &isint);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "vmin = %d\n", &vmin);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "vmax = %d\n", &vmax);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "dyavg = %f\n", &dyavg);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "dymax = %f\n", &dymax);
		fclose(fp);

		p1.fx = cam0[0][0] * scaling;
		p1.fy = p1.fx;
		p1.cx = -cam0[0][2] * scaling;
		p1.cy = -cam0[1][2] * scaling;
		p1.width = width * scaling;
		p1.height = height * scaling;
		p1.baseline = baseline / 1000.0f;
		p1.doffs = doffs * scaling;
		p1.maxDepth = p1.baseline * p1.fx / (float(vmin) + p1.doffs);
		p1.minDepth = p1.baseline * p1.fx / (float(vmax) + p1.doffs);
		p1.doffs = -p1.doffs;

		p2 = p1;
		p2.fx = cam1[0][0] * scaling;
		p2.fy = p2.fx;
		p2.cx = -cam1[0][2] * scaling;
		p2.cy = -cam1[1][2] * scaling;

		return true;
	}

	return false;
}

static void skip_comment(FILE *fp) {
    // skip comment lines in the headers of pnm files

    char c;
    while ((c=getc(fp)) == '#')
        while (getc(fp) != '\n') ;
    ungetc(c, fp);
}

static void skip_space(FILE *fp) {
    // skip white space in the headers or pnm files

    char c;
    do {
        c = getc(fp);
    } while (c == '\n' || c == ' ' || c == '\t' || c == '\r');
    ungetc(c, fp);
}

static void read_header(FILE *fp, const char *imtype, char c1, char c2, 
                 int *width, int *height, int *nbands, int thirdArg)
{
    // read the header of a pnmfile and initialize width and height

    char c;
  
	if (getc(fp) != c1 || getc(fp) != c2)
		LOG(FATAL) << "ReadFilePGM: wrong magic code for " << imtype << " file";
	skip_space(fp);
	skip_comment(fp);
	skip_space(fp);
	if (fscanf(fp, "%d", width) <= 0) {
		LOG(FATAL) << "PFM file error";
	};
	skip_space(fp);
	if (fscanf(fp, "%d", height) <= 0) {
		LOG(FATAL) << "PFM file error";
	}
	if (thirdArg) {
		skip_space(fp);
		if (fscanf(fp, "%d", nbands) <= 0) {
			LOG(FATAL) << "PFM file error";
		}
	}
    // skip SINGLE newline character after reading image height (or third arg)
	c = getc(fp);
    if (c == '\r')      // <cr> in some files before newline
        c = getc(fp);
    if (c != '\n') {
        if (c == ' ' || c == '\t' || c == '\r')
            LOG(FATAL) << "newline expected in file after image height";
        else
            LOG(FATAL) << "whitespace expected in file after image height";
  }
}

// check whether machine is little endian
static int littleendian() {
    int intval = 1;
    uchar *uval = (uchar *)&intval;
    return uval[0] == 1;
}

// 1-band PFM image, see http://netpbm.sourceforge.net/doc/pfm.html
// 3-band not yet supported
static void readFilePFM(cv::Mat &img, const string &filename)
{
    // Open the file and read the header
    FILE *fp = fopen(filename.c_str(), "rb");
    if (fp == 0)
        LOG(FATAL) << "ReadFilePFM: could not open \"" << filename << "\"";

    int width, height, nBands;
    read_header(fp, "PFM", 'P', 'f', &width, &height, &nBands, 0);

    skip_space(fp);

    float scalef;
    if (fscanf(fp, "%f", &scalef) <= 0) {  // scale factor (if negative, little endian)
		LOG(FATAL) << "Invalid PFM file";
	}

    // skip SINGLE newline character after reading third arg
    char c = getc(fp);
    if (c == '\r')      // <cr> in some files before newline
        c = getc(fp);
    if (c != '\n') {
        if (c == ' ' || c == '\t' || c == '\r')
            LOG(FATAL) << "newline expected in file after scale factor";
        else
            LOG(FATAL) << "whitespace expected in file after scale factor";
    }
    
    // Allocate the image if necessary
    img = cv::Mat(height, width, CV_32FC1);
    // Set the image shape
    //Size sh = img.size();

    int littleEndianFile = (scalef < 0);
    int littleEndianMachine = littleendian();
    int needSwap = (littleEndianFile != littleEndianMachine);
    //printf("endian file = %d, endian machine = %d, need swap = %d\n", 
    //       littleEndianFile, littleEndianMachine, needSwap);

    for (int y = height-1; y >= 0; y--) { // PFM stores rows top-to-bottom!!!!
	int n = width;
	float* ptr = &img.at<float>(y, 0, 0);
	if ((int)fread(ptr, sizeof(float), n, fp) != n)
	    LOG(FATAL) << "ReadFilePFM(" << filename << "): file is too short";
	
	if (needSwap) { // if endianness doesn't agree, swap bytes
	    uchar* ptr = (uchar *)&img.at<uchar>(y, 0, 0);
	    int x = 0;
	    uchar tmp = 0;
	    while (x < n) {
		tmp = ptr[0]; ptr[0] = ptr[3]; ptr[3] = tmp;
		tmp = ptr[1]; ptr[1] = ptr[2]; ptr[2] = tmp;
		ptr += 4;
		x++;
	    }
	}
    }
    if (fclose(fp))
        LOG(FATAL) << "ReadFilePGM(" << filename << "): error closing file";
}

int main(int argc, char **argv) {
	auto *root = ftl::configure(argc, argv, "tools_default");

	ftl::stream::File *out = ftl::create<ftl::stream::File>(root, "output");
	out->set("filename", root->value("out", std::string("./out.ftl")));
	out->setMode(ftl::stream::File::Mode::Write);
	out->begin(false);

	int height = root->value("height", 1080);

	// For each middlebury test folder
	auto paths = (*root->get<nlohmann::json>("paths"));

	ftl::data::Pool pool(1,1);
	ftl::data::Frame dframe = pool.allocate(ftl::data::FrameID(0,0), 10);
	ftl::rgbd::Frame &frame = dframe.cast<ftl::rgbd::Frame>();
	frame.store();

	ftl::operators::DisparityToDepth disp2depth(ftl::create<ftl::Configurable>(root, "disparity"));

	ftl::codecs::OpenCVEncoder encoder(ftl::codecs::definition_t::Any, ftl::codecs::definition_t::Any);

	int i=0;
	for (auto &x : paths.items()) {
		std::string dir = x.value().get<std::string>();

		std::vector<std::string> dirents = ftl::directory_listing(dir);
		for (auto &path : dirents) {
			// Validate the folder as middlebury
			ftl::rgbd::Camera intrin1;
			ftl::rgbd::Camera intrin2;
			int ndisp = 0;
			if (!loadMiddleburyCalib(path+"/calib.txt", intrin1, intrin2, ndisp, 1.0f)) {
				LOG(ERROR) << "Could not load middlebury calibration: " << path;
				continue;
			}

			frame.reset();

			// Load the colour images into a frame.
			frame.create<cv::Mat>(Channel::Colour) = cv::imread(path+"/im0.png", cv::IMREAD_COLOR);
			frame.create<cv::Mat>(Channel::Colour2) = cv::imread(path+"/im1.png", cv::IMREAD_COLOR);

			// Colour convert
			auto &c1 = frame.get<cv::Mat>(Channel::Colour);
			auto &c2 = frame.get<cv::Mat>(Channel::Colour2);
			cv::cvtColor(c1,c1, cv::COLOR_BGR2RGBA);
			cv::cvtColor(c2,c2, cv::COLOR_BGR2RGBA);

			// Load the ground truth
			//frame.create<cv::Mat>(Channel::Disparity) = cv::imread(path+"/disp0.pfm", cv::IMREAD_UNCHANGED);
			readFilePFM(frame.create<cv::Mat>(Channel::Disparity), path+"/disp0.pfm");
			cv::Mat &disp = frame.set<cv::Mat>(Channel::Disparity);
			float aspect = float(disp.cols) / float(disp.rows);
			float scaling = float(height) / float(disp.rows);
			cv::resize(disp, disp, cv::Size(int(aspect*float(height)),height), 0.0, 0.0, cv::INTER_NEAREST);
			cv::resize(c1, c1, cv::Size(int(aspect*float(height)),height));
			cv::resize(c2, c2, cv::Size(int(aspect*float(height)),height));

			int original_width = c1.cols;
			int desired_width = ftl::codecs::getWidth(ftl::codecs::findDefinition(height));
			int border_size = (desired_width - c1.cols) / 2;
			cv::copyMakeBorder(c1, c1, 0, 0, border_size, desired_width - border_size - c1.cols, cv::BORDER_CONSTANT, cv::Scalar(0));
			cv::copyMakeBorder(c2, c2, 0, 0, border_size, desired_width - border_size - c2.cols, cv::BORDER_CONSTANT, cv::Scalar(0));
			cv::copyMakeBorder(disp, disp, 0, 0, border_size, desired_width - border_size - disp.cols, cv::BORDER_CONSTANT, cv::Scalar(0));

			// TODO: Adjust principle points (cx)

			LOG(INFO) << "Disparity scaling: " << scaling;
			LOG(INFO) << "Depth range: " << intrin1.minDepth << " to " << intrin1.maxDepth;
			LOG(INFO) << "Resolution: " << c1.cols << "x" << c1.rows;
			disp.convertTo(disp, CV_32F, scaling);

			intrin1 = intrin1.scaled(original_width, c1.rows);
			intrin2 = intrin2.scaled(original_width, c2.rows);
			intrin1.cx -= border_size;
			intrin2.cx -= border_size;
			intrin1.width = c1.cols;
			intrin2.width = c2.cols;

			frame.setLeft() = intrin1;
			frame.setRight() = intrin2;
			//ftl::stream::injectCalibration(out, frame, 0, 0, i, false);
			//ftl::stream::injectCalibration(out, frame, 0, 0, i, true);

			// Convert disparity to depth
			frame.upload(Channel::Disparity);
			frame.upload(Channel::Colour);
			frame.upload(Channel::Colour2);


			disp2depth.apply(frame, frame, 0);

			// Encode the frame into the output stream
			ftl::codecs::StreamPacket spkt;
			ftl::codecs::Packet pkt;

			spkt.timestamp = 0;
			spkt.frame_number = i;
			spkt.streamID = 0;
			spkt.version = 4;
			pkt.codec = codec_t::Any;
			pkt.bitrate = 0;
			pkt.flags = 0;
			pkt.frame_count = 1;

			spkt.channel = Channel::Colour;
			if (!encoder.encode(frame.get<cv::cuda::GpuMat>(Channel::Colour), pkt)) {
				LOG(ERROR) << "Encode failed for colour";
			}
			out->post(spkt, pkt);

			pkt.codec = codec_t::Any;
			spkt.channel = Channel::Colour2;
			if (!encoder.encode(frame.get<cv::cuda::GpuMat>(Channel::Colour2), pkt)) {
				LOG(ERROR) << "Encode failed for colour2";
			}
			out->post(spkt, pkt);

			spkt.channel = Channel::GroundTruth;
			pkt.flags = ftl::codecs::kFlagFloat;
			pkt.codec = codec_t::Any;
			if (!encoder.encode(frame.get<cv::cuda::GpuMat>(Channel::Depth), pkt)) {
				LOG(ERROR) << "Encode failed for depth";
			}
			out->post(spkt, pkt);

			++i;
		}
	}

	out->end();

	return 0;
}
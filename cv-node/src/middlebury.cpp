#include <ftl/middlebury.hpp>
#include <glog/logging.h>
#include <ftl/disparity.hpp>

#include <string>
#include <algorithm>

using cv::Mat;
using cv::Size;
using std::string;
using std::min;
using std::max;

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
	fscanf(fp, "%d", width);
	skip_space(fp);
	fscanf(fp, "%d", height);
	if (thirdArg) {
		skip_space(fp);
		fscanf(fp, "%d", nbands);
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
void ftl::middlebury::readFilePFM(Mat &img, const string &filename)
{
    // Open the file and read the header
    FILE *fp = fopen(filename.c_str(), "rb");
    if (fp == 0)
        LOG(FATAL) << "ReadFilePFM: could not open " << filename;

    int width, height, nBands;
    read_header(fp, "PFM", 'P', 'f', &width, &height, &nBands, 0);

    skip_space(fp);

    float scalef;
    fscanf(fp, "%f", &scalef);  // scale factor (if negative, little endian)

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
    img = Mat(height, width, CV_32FC1);
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

// 1-band PFM image, see http://netpbm.sourceforge.net/doc/pfm.html
// 3-band not yet supported
void ftl::middlebury::writeFilePFM(const Mat &img, const char* filename, float scalefactor)
{
    // Write a PFM file
    Size sh = img.size();
    int nBands = img.channels();
    if (nBands != 1)
	LOG(FATAL) << "WriteFilePFM(" << filename << "): can only write 1-band image as pfm for now";
	
    // Open the file
    FILE *stream = fopen(filename, "wb");
    if (stream == 0)
        LOG(FATAL) << "WriteFilePFM: could not open " << filename;

    // sign of scalefact indicates endianness, see pfms specs
    if (littleendian())
	scalefactor = -scalefactor;

    // write the header: 3 lines: Pf, dimensions, scale factor (negative val == little endian)
    fprintf(stream, "Pf\n%d %d\n%f\n", sh.width, sh.height, scalefactor);

    int n = sh.width;
    // write rows -- pfm stores rows in inverse order!
    for (int y = sh.height-1; y >= 0; y--) {
	const float* ptr = &img.at<float>(0, y, 0);
	if ((int)fwrite(ptr, sizeof(float), n, stream) != n)
	    LOG(FATAL) << "WriteFilePFM(" << filename << "): file is too short";
    }
    
    // close file
    if (fclose(stream))
        LOG(FATAL) << "WriteFilePFM(" << filename << "): error closing file";
}

void ftl::middlebury::evaldisp(const Mat &disp, const Mat &gtdisp, const Mat &mask, float badthresh, int maxdisp, int rounddisp)
{
    Size sh = gtdisp.size();
    Size sh2 = disp.size();
    Size msh = mask.size();
    int width = sh.width, height = sh.height;
    int width2 = sh2.width, height2 = sh2.height;
    int scale = width / width2;

    if ((!(scale == 1 || scale == 2 || scale == 4))
	|| (scale * width2 != width)
	|| (scale * height2 != height)) {
	printf("   disp size = %4d x %4d\n", width2, height2);
	printf("GT disp size = %4d x %4d\n", width,  height);
	LOG(ERROR) << "GT disp size must be exactly 1, 2, or 4 * disp size";
    }

    int usemask = (msh.width > 0 && msh.height > 0);
    if (usemask && (msh != sh))
	LOG(ERROR) << "mask image must have same size as GT";

    int n = 0;
    int bad = 0;
    int invalid = 0;
    float serr = 0;
    for (int y = 0; y < height; y++) {
	for (int x = 0; x < width; x++) {
	    float gt = gtdisp.at<float>(y, x, 0);
	    if (gt == INFINITY) // unknown
		continue;
	    float d = scale * disp.at<float>(y / scale, x / scale, 0);
	    int valid = (d != INFINITY);
	    if (valid) {
		float maxd = scale * maxdisp; // max disp range
		d = max(0.0f, min(maxd, d)); // clip disps to max disp range
	    }
	    if (valid && rounddisp)
		d = round(d);
	    float err = fabs(d - gt);
	    if (usemask && mask.at<float>(y, x, 0) != 255) { // don't evaluate pixel
	    } else {
		n++;
		if (valid) {
		    serr += err;
		    if (err > badthresh) {
			bad++;
		    }
		} else {// invalid (i.e. hole in sparse disp map)
		    invalid++;
		}
	    }
	}
    }
    float badpercent =  100.0*bad/n;
    float invalidpercent =  100.0*invalid/n;
    float totalbadpercent =  100.0*(bad+invalid)/n;
    float avgErr = serr / (n - invalid); // CHANGED 10/14/2014 -- was: serr / n
    printf("mask  bad%.1f  invalid  totbad   avgErr\n", badthresh);
    printf("%4.1f  %6.2f  %6.2f   %6.2f  %6.2f\n",   100.0*n/(width * height), 
	   badpercent, invalidpercent, totalbadpercent, avgErr);
}

void ftl::middlebury::test(nlohmann::json &config) {
	// Load dataset images
	Mat l = cv::imread((string)config["middlebury"]["dataset"] + "/im0.png");
	Mat r = cv::imread((string)config["middlebury"]["dataset"] + "/im1.png");
	
	// Load ground truth
	Mat gt;
	readFilePFM(gt, (string)config["middlebury"]["dataset"] + "/disp0.pfm");
	
	if ((float)config["middlebury"]["scale"] != 1.0f) {
		float scale = (float)config["middlebury"]["scale"];
		//cv::resize(gt, gt, cv::Size(gt.cols * scale,gt.rows * scale), 0, 0, cv::INTER_LINEAR);
		cv::resize(l, l, cv::Size(l.cols * scale,l.rows * scale), 0, 0, cv::INTER_LINEAR);
		cv::resize(r, r, cv::Size(r.cols * scale,r.rows * scale), 0, 0, cv::INTER_LINEAR);
	}
	
	// Run algorithm
	auto disparity = ftl::Disparity::create(config["disparity"]);
	cvtColor(l,  l, cv::COLOR_BGR2GRAY);
    cvtColor(r, r, cv::COLOR_BGR2GRAY);
        
    Mat disp;
    disparity->compute(l,r,disp);
	disp.convertTo(disp, CV_32F);
	
	// Display results
	evaldisp(disp, gt, Mat(), (float)config["middlebury"]["threshold"], (int)config["disparity"]["maximum"], 0);
	
	if (gt.cols > 1600) {
		cv::resize(gt, gt, cv::Size(gt.cols * 0.25,gt.rows * 0.25), 0, 0, cv::INTER_LINEAR);
	}
	if (disp.cols > 1600) {
		cv::resize(disp, disp, cv::Size(disp.cols * 0.25,disp.rows * 0.25), 0, 0, cv::INTER_LINEAR);
	}
	
	double mindisp;
	double maxdisp;
	Mat mask;
	threshold(disp,mask,10000.0, 255, cv::THRESH_BINARY_INV);
	normalize(mask, mask, 0, 255, cv::NORM_MINMAX, CV_8U);
	cv::minMaxLoc(disp, &mindisp, &maxdisp, 0, 0, mask);

	gt = gt / 330.0; // TODO Read from calib.txt
	disp = disp / maxdisp;
	imshow("Ground Truth", gt);
	imshow("Disparity", disp);
	
	while (cv::waitKey(10) != 27);
	
	/*cv::putText(yourImageMat, 
            "Here is some text",
            cv::Point(5,5), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1, // Line Thickness (Optional)
            cv::CV_AA); // Anti-alias (Optional)*/
}


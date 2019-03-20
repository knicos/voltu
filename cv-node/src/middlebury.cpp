#include <ftl/middlebury.hpp>

using cv::Mat;
using cv::Size;

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
		throw CError("ReadFilePGM: wrong magic code for %s file", imtype);
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
            throw CError("newline expected in file after image height");
        else
            throw CError("whitespace expected in file after image height");
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
void ftl::middlebury::readFilePFM(Mat &img, const char* filename)
{
    // Open the file and read the header
    FILE *fp = fopen(filename, "rb");
    if (fp == 0)
        throw CError("ReadFilePFM: could not open %s", filename);

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
            throw CError("newline expected in file after scale factor");
        else
            throw CError("whitespace expected in file after scale factor");
    }

    // Set the image shape
    Size sh(width, height, 1);
    
    // Allocate the image if necessary
    img.ReAllocate(sh);

    int littleEndianFile = (scalef < 0);
    int littleEndianMachine = littleendian();
    int needSwap = (littleEndianFile != littleEndianMachine);
    //printf("endian file = %d, endian machine = %d, need swap = %d\n", 
    //       littleEndianFile, littleEndianMachine, needSwap);

    for (int y = height-1; y >= 0; y--) { // PFM stores rows top-to-bottom!!!!
	int n = width;
	float* ptr = (float *) img.PixelAddress(0, y, 0);
	if ((int)fread(ptr, sizeof(float), n, fp) != n)
	    throw CError("ReadFilePFM(%s): file is too short", filename);
	
	if (needSwap) { // if endianness doesn't agree, swap bytes
	    uchar* ptr = (uchar *) img.PixelAddress(0, y, 0);
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
        throw CError("ReadFilePGM(%s): error closing file", filename);
}

// 1-band PFM image, see http://netpbm.sourceforge.net/doc/pfm.html
// 3-band not yet supported
void ftl::middlebury::writeFilePFM(const Mat &img, const char* filename, float scalefactor=1/255.0)
{
    // Write a PFM file
    CShape sh = img.Shape();
    int nBands = sh.nBands;
    if (nBands != 1)
	throw CError("WriteFilePFM(%s): can only write 1-band image as pfm for now", filename);
	
    // Open the file
    FILE *stream = fopen(filename, "wb");
    if (stream == 0)
        throw CError("WriteFilePFM: could not open %s", filename);

    // sign of scalefact indicates endianness, see pfms specs
    if (littleendian())
	scalefactor = -scalefactor;

    // write the header: 3 lines: Pf, dimensions, scale factor (negative val == little endian)
    fprintf(stream, "Pf\n%d %d\n%f\n", sh.width, sh.height, scalefactor);

    int n = sh.width;
    // write rows -- pfm stores rows in inverse order!
    for (int y = sh.height-1; y >= 0; y--) {
	float* ptr = (float *)img.PixelAddress(0, y, 0);
	if ((int)fwrite(ptr, sizeof(float), n, stream) != n)
	    throw CError("WriteFilePFM(%s): file is too short", filename);
    }
    
    // close file
    if (fclose(stream))
        throw CError("WriteFilePFM(%s): error closing file", filename);
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
	    float gt = gtdisp.at(x, y, 0);
	    if (gt == INFINITY) // unknown
		continue;
	    float d = scale * disp.at(x / scale, y / scale, 0);
	    int valid = (d != INFINITY);
	    if (valid) {
		float maxd = scale * maxdisp; // max disp range
		d = __max(0, __min(maxd, d)); // clip disps to max disp range
	    }
	    if (valid && rounddisp)
		d = round(d);
	    float err = fabs(d - gt);
	    if (usemask && mask.at(x, y, 0) != 255) { // don't evaluate pixel
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
    //printf("mask  bad%.1f  invalid  totbad   avgErr\n", badthresh);
    printf("%4.1f  %6.2f  %6.2f   %6.2f  %6.2f\n",   100.0*n/(width * height), 
	   badpercent, invalidpercent, totalbadpercent, avgErr);
}

void ftl::middlebury::test(nlohmann::json &config) {
	// Load dataset images
	Mat l = imread((string)config["middlebury"]["dataset"] + "/im0.png");
	Mat r = imread((string)config["middlebury"]["dataset"] + "/im1.png");
	
	// Load ground truth
	Mat gt;
	readFilePFM(gt, (string)config["middlebury"]["dataset"] + "disp0.pfm");
	
	// Run algorithm
	auto disparity = Disparity::create(config["disparity"]);
	cvtColor(l,  l, COLOR_BGR2GRAY);
    cvtColor(r, r, COLOR_BGR2GRAY);
        
    Mat disp;
    disparity->compute(l,r,disp);
	disp.convertTo(disp, CV_32F);
	
	// Display results
	evaldisp(disp, gt, Mat(), (float)config["middlebury"]["threshold"], (int)config["disparity"]["maximum"], 0);
	
	imshow("Ground Truth", gt);
	imshow("Disparity", disp);
	
	/*cv::putText(yourImageMat, 
            "Here is some text",
            cv::Point(5,5), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1, // Line Thickness (Optional)
            cv::CV_AA); // Anti-alias (Optional)*/
}


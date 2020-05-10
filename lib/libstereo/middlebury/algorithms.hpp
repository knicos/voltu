#pragma once

#include <map>

#include "middlebury.hpp"
#include "stereo.hpp"

struct Algorithm {
	virtual void run(const MiddleburyData &data, cv::Mat &disparity)=0;
	float P1 = 4.0f;
	float P2 = 80.0f;
	bool subpixel = true;
	bool lr_consistency = true;
};

namespace Impl {

	struct CensusSGM : public Algorithm {
		CensusSGM() { P1 = 30.0f; P2 = 132.0f; }  // Tuned to total error 2.0

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.debug = false;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct ECensusSGM : public Algorithm {
		ECensusSGM() { P1 = 8.0f; P2 = 40.0f; }  // Tuned to total error 2.0

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoExCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.debug = false;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.alpha = 0.5f;
			stereo.params.beta = 1.0f;
			stereo.params.var_window = 7;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct GCensusSGM : public Algorithm {
		GCensusSGM() { P1 = 20.0f; P2 = 110.0f; }  // Tuned to total error 2.0

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.pattern = CensusPattern::GENERALISED;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct MeanCensusSGM : public Algorithm {
		MeanCensusSGM() { P1 = 12.0f; P2 = 32.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoMeanCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct StableSGM : public Algorithm {
		StableSGM() { P1 = 1.0f; P2 = 8.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoStableSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;
			stereo.params.wsize = 9;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct HStableSGM : public Algorithm {
		HStableSGM() { P1 = 3.0f; P2 = 24.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoHStableSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;
			stereo.params.wsize = 7;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.params.alpha = 0.5f;
			stereo.params.beta = 1.0f;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct BRefCensusSGM : public Algorithm {
		BRefCensusSGM() { P1 = 12.0f; P2 = 32.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoBRefCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct HCensusSGM : public Algorithm {
		HCensusSGM() { P1 = 6.0f; P2 = 26.0f; }  // Tuned to total error 2.0

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoHierCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;
			stereo.params.var_window = 5;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.params.alpha = 0.5f;
			stereo.params.beta = 1.0f;

			stereo.compute(data.imL, data.imR, disparity);

			/*cv::cuda::GpuMat gdisp;
			cv::cuda::GpuMat gdisp2;
			cv::cuda::GpuMat gl;
			gl.upload(data.imL);
			gdisp.create(gl.size(), CV_32F);
			stereo.compute(data.imL, data.imR, gdisp);

			gdisp.convertTo(gdisp2, CV_16S);
			auto blf = cv::cuda::createDisparityBilateralFilter(stereo.params.d_max-stereo.params.d_min+1,3,10);
			blf->apply(gdisp2, gl, gdisp2);
			gdisp2.convertTo(gdisp, CV_32F);
			gdisp.download(disparity);*/
		}
	};

	struct HWCensusSGM : public Algorithm {
		HWCensusSGM() { P1 = 8.0f; P2 = 38.0f; }  // Tuned to total error 2.0

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoHierWeightCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;
			stereo.params.var_window = 5;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.params.alpha = 0.5f;
			stereo.params.beta = 1.0f;

			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct HGCensusSGM : public Algorithm {
		HGCensusSGM() { P1 = 36.0f; P2 = 96.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoHierCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.params.alpha = 0.5f;
			stereo.params.beta = 1.0f;
			stereo.params.pattern = CensusPattern::GENERALISED;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct CPCensusSGM : public Algorithm {
		CPCensusSGM() { P1 = 4.0f; P2 = 60.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			 StereoCPCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.params.debug = false;
			stereo.params.uniqueness = 20;
			stereo.params.filtercosts = false;
			stereo.params.alpha = 0.2f;
			stereo.params.beta = 1.0f;
			stereo.params.var_window = 7;
			for (int i=0; i<10; ++i) {
				if (i == 10-1) {
					stereo.params.uniqueness = std::numeric_limits<unsigned short>::max();
					stereo.params.filtercosts = false;
				}
				stereo.compute(data.imL, data.imR, disparity);
			}
		}
	};

	struct TCensusSGM : public Algorithm {
		TCensusSGM() { P1 = 24.0f; P2 = 64.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoTCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct WCensusSGM : public Algorithm {
		WCensusSGM() { P1 = 12.0f; P2 = 32.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoWCensusSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.debug = false;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct MiSGM : public Algorithm {
		MiSGM() { P1 = 4.0f; P2 = 18.0f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoMiSgm stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			//stereo.params.lr_consistency = lr_consistency;

			stereo.params.debug = false;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.compute(data.imL, data.imR, disparity);
			stereo.setPrior(disparity);
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct WADCensusSGM : public Algorithm {
		WADCensusSGM() { P1 = 0.2f; P2 = 0.5f; }

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoWADCensus stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			//stereo.params.lr_consistency = lr_consistency;

			stereo.params.l1 = 1.0;
			stereo.params.l2 = 0.0;
			stereo.params.alpha = 0.4;
			stereo.params.beta = 1.0;
			stereo.params.wsize = 9;
			stereo.params.var_window = 15;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.params.debug = false;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

	struct VarCensusSGM : public Algorithm {
		VarCensusSGM() { P1 = 12.0f; P2 = 52.0f; }  // Tuned to total error 2.0

		virtual void run(const MiddleburyData &data, cv::Mat &disparity) override {
			StereoVarCensus stereo;
			stereo.params.P1 = P1;
			stereo.params.P2 = P2;
			stereo.params.subpixel = subpixel;
			stereo.params.lr_consistency = lr_consistency;

			stereo.params.alpha = 0.33;
			stereo.params.beta = 1.0;
			stereo.params.var_window = 27;
			stereo.params.debug = false;
			stereo.params.d_min = data.calib.vmin;
			stereo.params.d_max = data.calib.vmax;
			stereo.compute(data.imL, data.imR, disparity);
		}
	};

}

static const std::map<std::string, Algorithm*> algorithms = {
	{ "censussgm", new Impl::CensusSGM() },
	{ "mcensussgm", new Impl::MeanCensusSGM() },
	{ "gcensussgm", new Impl::GCensusSGM() },
	{ "ecensussgm", new Impl::ECensusSGM() },
	{ "stablesgm", new Impl::StableSGM() },
	{ "hstablesgm", new Impl::HStableSGM() },
	{ "brefcensus", new Impl::BRefCensusSGM() },
	{ "hgcensussgm", new Impl::HGCensusSGM() },
	{ "hcensussgm", new Impl::HCensusSGM() },
	{ "hwcensussgm", new Impl::HWCensusSGM() },
	{ "cpcensussgm",  new Impl::CPCensusSGM() },
	{ "tcensussgm",  new Impl::TCensusSGM() },
	{ "wcensussgm",  new Impl::WCensusSGM() },
	{ "misgm",  new Impl::MiSGM() },
	{ "varcensus",  new Impl::VarCensusSGM() },
};

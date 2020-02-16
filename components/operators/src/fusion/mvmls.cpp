#include <ftl/operators/mvmls.hpp>
#include "smoothing_cuda.hpp"
#include <ftl/utility/matrix_conversion.hpp>
#include "mvmls_cuda.hpp"

#include <opencv2/cudaarithm.hpp>

using ftl::operators::MultiViewMLS;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;
using ftl::rgbd::Format;

MultiViewMLS::MultiViewMLS(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

MultiViewMLS::~MultiViewMLS() {

}

bool MultiViewMLS::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) {
    cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

   // float thresh = config()->value("mls_threshold", 0.4f);
	float disconPixels = config()->value("discon_pixels", 100.0f);  // Max before definitely not same surface
	
	float col_smooth = config()->value("mls_colour_smoothing", 30.0f);
	int iters = config()->value("mls_iterations", 3);
	int radius = config()->value("mls_radius",5);
	//bool aggre = config()->value("aggregation", true);
    //int win = config()->value("cost_function",1);
    int win = config()->value("window_size",16);
    bool do_corr = config()->value("merge_corresponding", true);
	bool do_aggr = config()->value("merge_mls", false);
	bool cull_zero = config()->value("cull_no_confidence", false);
    //bool show_best_source = config()->value("show_pixel_source", false);

    ftl::cuda::MvMLSParams params;
    params.colour_smooth = config()->value("colour_smooth", 150.0f);
    params.spatial_smooth = config()->value("spatial_smooth", 1.0f);

	bool show_consistency = config()->value("show_consistency", false);
	bool show_adjustment = config()->value("show_adjustment", false);

    if (in.frames.size() < 1) return false;
    auto size = in.frames[0].get<GpuMat>(Channel::Depth).size();

    // Make sure we have enough buffers
    while (normals_horiz_.size() < in.frames.size()) {
        normals_horiz_.push_back(new ftl::cuda::TextureObject<half4>(size.height, size.width));
        centroid_horiz_.push_back(new ftl::cuda::TextureObject<float4>(size.height, size.width));
        centroid_vert_.push_back(new ftl::cuda::TextureObject<float4>(size.width, size.height));
        contributions_.push_back(new ftl::cuda::TextureObject<float>(size.width, size.height));
    }

    // Make sure all buffers are at correct resolution and are allocated
    for (size_t i=0; i<in.frames.size(); ++i) {
        auto &f = in.frames[i];
	    auto size = f.get<GpuMat>(Channel::Depth).size();
	    centroid_horiz_[i]->create(size.height, size.width);
	    normals_horiz_[i]->create(size.height, size.width);
	    centroid_vert_[i]->create(size.width, size.height);
        contributions_[i]->create(size.width, size.height);

        if (!f.hasChannel(Channel::Normals)) {
            throw FTL_Error("Required normals channel missing for MLS");
        }
        if (!f.hasChannel(Channel::Support2) && !f.hasChannel(Channel::Support1)) {
            throw FTL_Error("Required cross support channel missing for MLS");
        }

        // Create required channels
        f.create<GpuMat>(Channel::Confidence, Format<float>(size));
        f.createTexture<float>(Channel::Confidence);
        f.create<GpuMat>(Channel::Screen, Format<short2>(size));
        f.createTexture<short2>(Channel::Screen);

        f.get<GpuMat>(Channel::Confidence).setTo(cv::Scalar(0.0f), cvstream);
    }

    //for (int iter=0; iter<iters; ++iter) {
		// Step 1:
		// Calculate correspondences and adjust depth values
		// Step 2:
        // Find corresponding points and perform aggregation of any correspondences
        // For each camera combination
        if (do_corr) {
            for (size_t i=0; i<in.frames.size(); ++i) {
                auto &f1 = in.frames[i];
                //f1.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0.0f), cvstream);
                //f1.get<GpuMat>(Channel::Confidence).setTo(cv::Scalar(0.0f), cvstream);

                Eigen::Vector4d d1(0.0, 0.0, 1.0, 0.0);
                d1 = f1.getPose() * d1;

                for (size_t j=i+1; j<in.frames.size(); ++j) {
                    if (i == j) continue;

                    //LOG(INFO) << "Running phase1";

                    auto &f2 = in.frames[j];
                    //auto s1 = in.sources[i];
                    //auto s2 = in.sources[j];

                    // Are cameras facing similar enough direction?
                    Eigen::Vector4d d2(0.0, 0.0, 1.0, 0.0);
                    d2 = f2.getPose() * d2;
                    // No, so skip this combination
                    if (d1.dot(d2) <= 0.0) continue;

                    auto pose = MatrixConversion::toCUDA(f1.getPose().cast<float>().inverse() * f2.getPose().cast<float>());
                    auto pose2 = MatrixConversion::toCUDA(f2.getPose().cast<float>().inverse() * f1.getPose().cast<float>());

                    //auto transform = pose2 * pose1;

                    //Calculate screen positions of estimated corresponding points
                    //if (iter == 0) {
                        ftl::cuda::correspondence(
                            f1.getTexture<float>(Channel::Depth),
                            f2.getTexture<float>(Channel::Depth),
                            f1.getTexture<short2>(Channel::Screen),
                            f1.getTexture<float>(Channel::Confidence),
                            f1.getTexture<uint8_t>(Channel::Mask),
                            pose2,
                            f1.getLeftCamera(),
                            f2.getLeftCamera(),
                            params,
                            win,
                            stream
                        );
                        ftl::cuda::correspondence(
                            f2.getTexture<float>(Channel::Depth),
                            f1.getTexture<float>(Channel::Depth),
                            f2.getTexture<short2>(Channel::Screen),
                            f2.getTexture<float>(Channel::Confidence),
                            f2.getTexture<uint8_t>(Channel::Mask),
                            pose,
                            f2.getLeftCamera(),
                            f1.getLeftCamera(),
                            params,
                            win,
                            stream
                        );

						if (show_consistency) {
							ftl::cuda::show_cor_error(f1.getTexture<uchar4>(Channel::Colour), f1.getTexture<short2>(Channel::Screen), f2.getTexture<short2>(Channel::Screen), stream);
							ftl::cuda::show_cor_error(f2.getTexture<uchar4>(Channel::Colour), f2.getTexture<short2>(Channel::Screen), f1.getTexture<short2>(Channel::Screen), stream);
						}

						if (show_adjustment) {
							ftl::cuda::show_depth_adjustment(f1.getTexture<uchar4>(Channel::Colour), f1.getTexture<float>(Channel::Confidence), stream);
							ftl::cuda::show_depth_adjustment(f2.getTexture<uchar4>(Channel::Colour), f2.getTexture<float>(Channel::Confidence), stream);
						}

						// Actually perform the adjustment
						cv::cuda::add(f1.get<GpuMat>(Channel::Depth), f1.get<GpuMat>(Channel::Confidence), f1.get<GpuMat>(Channel::Depth), cv::noArray(), -1, cvstream);
						cv::cuda::add(f2.get<GpuMat>(Channel::Depth), f2.get<GpuMat>(Channel::Confidence), f2.get<GpuMat>(Channel::Depth), cv::noArray(), -1, cvstream);
                    //} //else {
                        /*ftl::cuda::correspondence(
                            f1.getTexture<float>(Channel::Depth),
                            f2.getTexture<float>(Channel::Depth),
                            f1.getTexture<uchar4>(Channel::Colour),
                            f2.getTexture<uchar4>(Channel::Colour),
                            // TODO: Add normals and other things...
                            f1.getTexture<short2>(Channel::Screen),
                            f1.getTexture<float>(Channel::Confidence),
                            f1.getTexture<uint8_t>(Channel::Mask),
                            pose2,
                            f1.getLeftCamera(),
                            f2.getLeftCamera(),
                            params,
                            win,
                            stream
                        );
                        ftl::cuda::correspondence(
                            f2.getTexture<float>(Channel::Depth),
                            f1.getTexture<float>(Channel::Depth),
                            f2.getTexture<uchar4>(Channel::Colour),
                            f1.getTexture<uchar4>(Channel::Colour),
                            // TODO: Add normals and other things...
                            f2.getTexture<short2>(Channel::Screen),
                            f2.getTexture<float>(Channel::Confidence),
                            f2.getTexture<uint8_t>(Channel::Mask),
                            pose,
                            f2.getLeftCamera(),
                            f1.getLeftCamera(),
                            params,
                            win,
                            stream
                        );*/
                    //}

                    // Also calculate best source for each point
                    /*ftl::cuda::best_sources(
                        f1.getTexture<uchar4>(Channel::Support1),
                        f2.getTexture<uchar4>(Channel::Support1),
                        f1.getTexture<float>(Channel::Depth),
                        f2.getTexture<float>(Channel::Depth),
                        f1.getTexture<short2>(Channel::Screen),
                        transform,
                        s1->parameters(),
                        s2->parameters(),
                        i, j, stream
                    );*/
				}
			}

            // Reduce window size for next iteration
            //win = max(win>>1, 4);
		}

	for (int iter=0; iter<iters; ++iter) {	

        // Find best source for every pixel
        /*for (size_t i=0; i<in.frames.size(); ++i) {
            auto &f1 = in.frames[i];
            //f1.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0.0f), cvstream);
            //f1.get<GpuMat>(Channel::Confidence).setTo(cv::Scalar(0.0f), cvstream);

            Eigen::Vector4d d1(0.0, 0.0, 1.0, 0.0);
            d1 = in.sources[i]->getPose() * d1;

            for (size_t j=0; j<in.frames.size(); ++j) {
                if (i == j) continue;

                //LOG(INFO) << "Running phase1";

                auto &f2 = in.frames[j];
                auto s1 = in.sources[i];
                auto s2 = in.sources[j];

                // Are cameras facing similar enough direction?
                Eigen::Vector4d d2(0.0, 0.0, 1.0, 0.0);
                d2 = in.sources[j]->getPose() * d2;
                // No, so skip this combination
                if (d1.dot(d2) <= 0.0) continue;

                auto pose1 = MatrixConversion::toCUDA(s1->getPose().cast<float>());
                auto pose1_inv = MatrixConversion::toCUDA(s1->getPose().cast<float>().inverse());
                auto pose2 = MatrixConversion::toCUDA(s2->getPose().cast<float>().inverse());
                auto pose2_inv = MatrixConversion::toCUDA(s2->getPose().cast<float>());

                auto transform = pose2 * pose1;

                // Also calculate best source for each point
                ftl::cuda::best_sources(
                    f1.getTexture<float4>(Channel::Normals),
                    f2.getTexture<float4>(Channel::Normals),
                    f1.getTexture<uchar4>(Channel::Support1),
                    f2.getTexture<uchar4>(Channel::Support1),
                    f1.getTexture<float>(Channel::Depth),
                    f2.getTexture<float>(Channel::Depth),
                    f1.getTexture<short2>(Channel::Screen),
                    transform,
                    s1->parameters(),
                    s2->parameters(),
                    i, j, stream
                );
            }
        }*/

		if (radius > 0) {
			// Step 2:
			// Do the horizontal and vertical MLS aggregations for each source
			// But don't do the final move step.
			for (size_t i=0; i<in.frames.size(); ++i) {
				auto &f = in.frames[i];
				//auto *s = in.sources[i];

				// Clear data
				//cv::cuda::GpuMat data(contributions_[i]->height(), contributions_[i]->width(), CV_32F, contributions_[i]->devicePtr(), contributions_[i]->pixelPitch());
				//data.setTo(cv::Scalar(0.0f), cvstream);

				if (cull_zero && iter == iters-1) {
					ftl::cuda::zero_confidence(
						f.getTexture<float>(Channel::Confidence),
						f.getTexture<float>(Channel::Depth),
						stream
					);
				}

				float thresh = (1.0f / f.getLeft().fx) * disconPixels;

				ftl::cuda::mls_aggr_horiz(
					f.createTexture<uchar4>((f.hasChannel(Channel::Support2)) ? Channel::Support2 : Channel::Support1),
					f.createTexture<half4>(Channel::Normals),
					*normals_horiz_[i],
					f.createTexture<float>(Channel::Depth),
					*centroid_horiz_[i],
					f.createTexture<uchar4>(Channel::Colour),
					thresh,
					col_smooth,
					radius,
					f.getLeftCamera(),
					stream
				);

				ftl::cuda::mls_aggr_vert(
					f.getTexture<uchar4>((f.hasChannel(Channel::Support2)) ? Channel::Support2 : Channel::Support1),
					*normals_horiz_[i],
					f.getTexture<half4>(Channel::Normals),
					*centroid_horiz_[i],
					*centroid_vert_[i],
					thresh,
					col_smooth,
					radius,
					f.getLeftCamera(),
					stream
				);
			}

			//return true;


			// Step 3:
			// Find corresponding points and perform aggregation of any correspondences
			// For each camera combination
			if (do_aggr) {
				for (size_t i=0; i<in.frames.size(); ++i) {
					auto &f1 = in.frames[i];
					//f1.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0.0f), cvstream);
					//f1.get<GpuMat>(Channel::Confidence).setTo(cv::Scalar(0.0f), cvstream);

					Eigen::Vector4d d1(0.0, 0.0, 1.0, 0.0);
					d1 = f1.getPose() * d1;

					for (size_t j=0; j<in.frames.size(); ++j) {
						if (i == j) continue;

						//LOG(INFO) << "Running phase1";

						auto &f2 = in.frames[j];
						//auto s1 = in.sources[i];
						//auto s2 = in.sources[j];

						// Are cameras facing similar enough direction?
						Eigen::Vector4d d2(0.0, 0.0, 1.0, 0.0);
						d2 = f2.getPose() * d2;
						// No, so skip this combination
						if (d1.dot(d2) <= 0.0) continue;

						auto pose1 = MatrixConversion::toCUDA(f1.getPose().cast<float>());
						//auto pose1_inv = MatrixConversion::toCUDA(f1.getPose().cast<float>().inverse());
						auto pose2 = MatrixConversion::toCUDA(f2.getPose().cast<float>().inverse());
						//auto pose2_inv = MatrixConversion::toCUDA(f2.getPose().cast<float>());

						auto transform = pose2 * pose1;

						// For the corresponding points, combine normals and centroids
						ftl::cuda::aggregate_sources(
							f1.getTexture<half4>(Channel::Normals),
							f2.getTexture<half4>(Channel::Normals),
							*centroid_vert_[i],
							*centroid_vert_[j],
							f1.getTexture<float>(Channel::Depth),
							//contributions_[i],
							//contributions_[j],
							//f1.getTexture<short2>(Channel::Screen),
							transform,
							f1.getLeftCamera(),
							f2.getLeftCamera(),
							stream
						);

						//LOG(INFO) << "Correspondences done... " << i;
					}
				}
			}

			// Step 3:
			// Normalise aggregations and move the points
			for (size_t i=0; i<in.frames.size(); ++i) {
				auto &f = in.frames[i];
				//auto *s = in.sources[i];
				auto size = f.get<GpuMat>(Channel::Depth).size();

				/*if (do_corr) {
					ftl::cuda::normalise_aggregations(
						f.getTexture<float4>(Channel::Normals),
						centroid_vert_[i],
						contributions_[i],
						stream
					);
				}*/

				ftl::cuda::mls_adjust_depth(
					f.getTexture<half4>(Channel::Normals),
					*centroid_vert_[i],
					f.createTexture<float>(Channel::Depth2, ftl::rgbd::Format<float>(size)),
					f.getLeftCamera(),
					stream
				);

				f.swapChannels(Channel::Depth, Channel::Depth2);

				/*if (show_best_source) {
					ftl::cuda::vis_best_sources(
						f.getTexture<short2>(Channel::Screen),
						f.getTexture<uchar4>(Channel::Colour),
						i,
						7, stream
					);
				}*/
			}
		}
    }

    return true;
}

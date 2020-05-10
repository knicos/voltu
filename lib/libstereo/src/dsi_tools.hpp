#ifndef _FTL_LIBSTEREO_DSI_TOOLS_HPP_
#define _FTL_LIBSTEREO_DSI_TOOLS_HPP_

#include "util.hpp"
#include "array2d.hpp"

namespace algorithms {
    template <typename DSI>
    struct SliceDisparity {
        typename DSI::DataType in;
        typename Array2D<float>::Data disparity;
        typename Array2D<typename DSI::Type>::Data out;

        __cuda__ void operator()(ushort2 thread, ushort2 stride, ushort2 size) {
            for (int y=thread.y; y<size.y; y+=stride.y) {
                for (int x=thread.x; x<size.x; x+=stride.x) {
                    auto d = disparity(y,x);
                    out(y,x) = (d >= in.disp_min && d <= in.disp_max) ? in(y,x,d) : 255;
                }
            }
        }
    };
}

template <typename DSI>
void dsi_slice(const DSI &in, int d, Array2D<typename DSI::Type> &out) {

}

/* Extract cost image from DSI using the disparity map as index. */
template <typename DSI>
void dsi_slice(const DSI &in, Array2D<float> &disp, Array2D<typename DSI::Type> &out) {
    algorithms::SliceDisparity<DSI> sd = {in.data(), disp.data(), out.data()};
    parallel2D(sd, in.width(), in.height());
}

void show_dsi_slice(cv::InputArray in);

#endif
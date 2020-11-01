#ifndef _CUDATL_COLOURS_HPP_
#define _CUDATL_COLOURS_HPP_

#include <cuda_runtime.h>

namespace cudatl {

/* From NVIDIA Npp */
template <typename T>
__device__ uchar3 rgb2hsv(T r, T g, T b) {
	const float nNormalizedR = float(r) * 0.003921569F; // / 255.0F
	const float nNormalizedG = float(g) * 0.003921569F;
	const float nNormalizedB = float(b) * 0.003921569F;
	float nS;
	float nH;
	// Value
	float nV = fmaxf(nNormalizedR, nNormalizedG);
	nV = fmaxf(nV, nNormalizedB);
	// Saturation
	float nTemp = fminf(nNormalizedR, nNormalizedG);
	nTemp = fminf(nTemp, nNormalizedB);
	float nDivisor = nV - nTemp;
	if (nV == 0.0F) // achromatics case
	{
		nS = 0.0F;
		nH = 0.0F;
	}    
	else // chromatics case
		nS = nDivisor / nV;
	// Hue:
	const float nCr = (nV - nNormalizedR) / nDivisor;
	const float nCg = (nV - nNormalizedG) / nDivisor;
	const float nCb = (nV - nNormalizedB) / nDivisor;
	if (nNormalizedR == nV)
		nH = nCb - nCg;
	else if (nNormalizedG == nV)
		nH = 2.0F + nCr - nCb;
	else if (nNormalizedB == nV)
		nH = 4.0F + nCg - nCr;
	nH = nH * 0.166667F; // / 6.0F       
	if (nH < 0.0F)
		nH = nH + 1.0F;

	return make_uchar3(nH * 255.0f, nS * 255.0f, nV* 255.0f);
}

template <typename T>
__device__ inline uchar3 bgr2hsv(T bgr) {
	return rgb2hsv(bgr.z, bgr.y, bgr.x);
}

template <typename T>
__device__ inline uchar3 hsv2rgb(T h, T s, T v) {
	float nNormalizedH = float(h) * 0.003921569F; // / 255.0F
	const float nNormalizedS = float(s) * 0.003921569F;
	const float nNormalizedV = float(v) * 0.003921569F;
	float nR;
	float nG;
	float nB;
	if (nNormalizedS == 0.0F)
	{
		nR = nG = nB = nNormalizedV;
	}
	else
	{
		if (nNormalizedH == 1.0F)
			nNormalizedH = 0.0F;
		else
			nNormalizedH = nNormalizedH * 6.0F; // / 0.1667F
	}
	const float nI = floorf(nNormalizedH);
	const float nF = nNormalizedH - nI;
	const float nM = nNormalizedV * (1.0F - nNormalizedS);
	const float nN = nNormalizedV * (1.0F - nNormalizedS * nF);
	const float nK = nNormalizedV * (1.0F - nNormalizedS * (1.0F - nF));
	if (nI == 0.0F)
		{ nR = nNormalizedV; nG = nK; nB = nM; }
	else if (nI == 1.0F)
		{ nR = nN; nG = nNormalizedV; nB = nM; }
	else if (nI == 2.0F)
		{ nR = nM; nG = nNormalizedV; nB = nK; }
	else if (nI == 3.0F)
		{ nR = nM; nG = nN; nB = nNormalizedV; }
	else if (nI == 4.0F)
		{ nR = nK; nG = nM; nB = nNormalizedV; }
	else if (nI == 5.0F)
		{ nR = nNormalizedV; nG = nM; nB = nN; }
	return make_uchar3(nR * 255.0f, nG * 255.0f, nB * 255.0f);
}

}

#endif
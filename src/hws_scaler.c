#include "hws_scaler.h"

void SetNoVideoMem(uint8_t *pDest, int w, int h)
{
	int x, y;
	uint8_t *pST;
	uint8_t *pNS;
	pST = (uint8_t *)pDest;
	//printk("SetNoVideoMem[%d-%d]\n",w,h);

	for (x = 0; x < w / 2; x++) {
		pST[0] = 41;
		pST[1] = 240;
		pST[2] = 41;
		pST[3] = 109;
		pST += 4;
	}

	pNS = pDest + w * 2;
	for (y = 1; y < h; y++) {
		memcpy(pNS, pDest, w * 2);
		pNS = pNS + w * 2;
	}
}

void All_VideoScaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
			    int out_w, int out_h)
{
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	BYTE *pDstYUV;
	int missX = 0;
	int missY = 0;
	int dumyX = 0;
	int dumyY = 0;
	int dwJ;
	int y;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	if (in_w > out_w) {
		missX = (in_w - out_w) / 2;
	} else {
		dumyX = (out_w - in_w) / 2;
	}
	if (in_h > out_h) {
		missY = (in_h - out_h) / 2;
	} else {
		dumyY = (out_h - in_h) / 2;
	}
	if (dumyY > 0) {
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < (dumyY * out_w); dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
		pDestBuf = pDestBuf + dumyY * out_w * 2;
	}
	if (missY > 0) {
		pSrcBuf += missY * in_w * 2;
	}
	//----------

	for (y = 0; y < (out_h - (dumyY * 2)); y++) {
		if (dumyX > 0) {
			pDstYUV = pDestBuf;
			for (dwJ = 0; dwJ < dumyX; dwJ++) {
				pDstYUV[0] = 0x10;
				pDstYUV[1] = 0x80;
				pDstYUV += 2;
			}
			pDestBuf += dumyX * 2;
		}
		memcpy(pDestBuf, pSrcBuf + missX * 2,
		       (out_w - (dumyX * 2)) * 2);
		pSrcBuf += in_w * 2;
		pDestBuf += (out_w - (dumyX * 2)) * 2;
		if (dumyX > 0) {
			pDstYUV = pDestBuf;
			for (dwJ = 0; dwJ < dumyX; dwJ++) {
				pDstYUV[0] = 0x10;
				pDstYUV[1] = 0x80;
				pDstYUV += 2;
			}
			pDestBuf += dumyX * 2;
		}
	}
	if (dumyY > 0) {
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < (dumyY * out_w); dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
	}
}

//-----------
void SetDeInterlace(BYTE *pSrc, BYTE *pOut, int nw, int nh,
			   int interlace)
{
	int dwSrcPitch = nw * 2;
	int h;
	//DbgPrint("[MV]SetDeInterlace W=%d H=%d interlace=%d \n",nw,nh,interlace);
	if (interlace == 0) {
		memcpy(pOut, pSrc, nw * nh * 2);
	} else {
		if (nh >= 1080) {
			nh = nh / 2;
		}
		for (h = 0; h < nh; h++) {
			memcpy(pOut, pSrc, dwSrcPitch);
			pOut += dwSrcPitch;
			memcpy(pOut, pSrc, dwSrcPitch);
			pSrc += dwSrcPitch;
			pOut += dwSrcPitch;
		}
	}
}


void VideoRotate90deg(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
			     int out_w, int out_h)

{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	BYTE *pSrcData;
	BYTE *pDestData;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//DbgPrint("[MV]VideoRotate90deg %dX%d->%dX%d\n",in_w,in_h,out_w,out_h);
	for (y = 0; y < in_h; y++) {
		pSrcData = pSrcBuf;
		pDestData = pDestBuf;
		for (x = 0; x < in_w / 2; x++) {
			if ((y % 2) == 0) {
				pDestData[0] = pSrcData[0];
				pDestData[1] = pSrcData[1];
				pDestData[out_w * 2 + 0] = pSrcData[2];
				pDestData[out_w * 2 + 1] = pSrcData[1];
			} else {
				pDestData[0] = pSrcData[0];
				pDestData[1] = pSrcData[3];
				pDestData[out_w * 2 + 0] = pSrcData[2];
				pDestData[out_w * 2 + 1] = pSrcData[3];
			}
			pDestData += out_w * 4;
			pSrcData += 4;
		}
		pSrcBuf += in_w * 2;
		pDestBuf += 2;
	}
}

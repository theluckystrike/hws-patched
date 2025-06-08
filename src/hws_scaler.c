#include "hws_scaler.h"


static void All_VideoScaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
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

static void VideoScaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w,
			int out_h)
{
	if ((in_w == 1920) && (in_h == 1080) && (out_w == 1280) &&
	    (out_h == 720)) {
		FHD_To_HD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1920) && (in_h == 1080) && (out_w == 800) &&
		   (out_h == 600)) {
		FHD_To_800X600_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 720) && (out_w == 1920) &&
		   (out_h == 1080)) {
		HD_To_FHD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 720) && (out_w == 800) &&
		   (out_h == 600)) {
		HD_To_800X600_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 720) && (in_h == 576) && (out_w == 720) &&
		   (out_h == 480)) {
		SD_PAL_To_SD_NTSC_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 720) && (in_h == 480) && (out_w == 720) &&
		   (out_h == 576)) {
		SD_NTSC_To_SD_PAL_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 720) && (in_h == 576) && (out_w == 1920) &&
		   (out_h == 1080)) {
		SD_PAL_To_FHD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 720) && (in_h == 480) && (out_w == 1920) &&
		   (out_h == 1080)) {
		SD_NTSC_To_FHD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 720) && (in_h == 576) && (out_w == 1280) &&
		   (out_h == 720)) {
		SD_PAL_To_HD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 720) && (in_h == 480) && (out_w == 1280) &&
		   (out_h == 720)) {
		SD_NTSC_To_HD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 1024) && (out_w == 1920) &&
		   (out_h == 1080)) {
		V1280X1024_To_FHD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 1024) && (out_w == 1280) &&
		   (out_h == 720)) {
		V1280X1024_To_HD_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 1024) && (out_w == 800) &&
		   (out_h == 600)) {
		V1280X1024_To_800X600_Scaler(pSrc, pOut, in_w, in_h, out_w,
					     out_h);
	} else if ((in_w == 1920) && (in_h == 1080) && (out_w == 720) &&
		   (out_h == 480)) {
		FHD_To_SD_NTSC_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1920) && (in_h == 1080) && (out_w == 720) &&
		   (out_h == 576)) {
		FHD_To_SD_PAL_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 720) && (out_w == 720) &&
		   (out_h == 480)) {
		HD_To_SD_NTSC_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 720) && (out_w == 720) &&
		   (out_h == 576)) {
		HD_To_SD_PAL_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 1024) && (out_w == 720) &&
		   (out_h == 480)) {
		V1280X1024_NTSC_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else if ((in_w == 1280) && (in_h == 1024) && (out_w == 720) &&
		   (out_h == 576)) {
		V1280X1024_PAL_Scaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	} else {
		All_VideoScaler(pSrc, pOut, in_w, in_h, out_w, out_h);
	}
}

static void FillNV12ToYUY2(BYTE *pSrc, BYTE *pOut, int nw, int nh,
			   int interlace)
{
	BYTE *pVideoMem = pOut; // g_pD1TempBuffer;
	//DWORD dwDstPitch = nw;
	DWORD dwSrcPitch;
	DWORD dwPacket;
	BYTE *pSrc1;
	BYTE *pDstYUV = (BYTE *)pVideoMem;
	BYTE *pNextLineSrc1;
	DWORD dwJ;
	BYTE bLineSel = 0;
	BOOL bUVSel;
	DWORD dwI;
	dwSrcPitch = nw * 12 / 8;
	dwPacket = dwSrcPitch / 24;

	pSrc1 = (BYTE *)pSrc;
	pDstYUV = (BYTE *)pVideoMem;

	if (interlace == 0) {
		for (dwI = 0; dwI < nh; dwI++) {
			switch (bLineSel) {
			case 0: {
				bUVSel = 0;
				pNextLineSrc1 = (BYTE *)pSrc1 + dwSrcPitch +
						nw; //  line2
				for (dwJ = 0; dwJ < dwPacket; dwJ++) {
					//-----------y0 ~y3
					pDstYUV[0] = pSrc1[0];
					pDstYUV[2] = pSrc1[1];
					pDstYUV[4] = pSrc1[2];
					pDstYUV[6] = pSrc1[3];
					//y4~y7
					pDstYUV[8] = pSrc1[8];
					pDstYUV[10] = pSrc1[9];
					pDstYUV[12] = pSrc1[10];
					pDstYUV[14] = pSrc1[11];
					//----y8~y11
					pDstYUV[16] = pSrc1[12];
					pDstYUV[18] = pSrc1[13];
					pDstYUV[20] = pSrc1[14];
					pDstYUV[22] = pSrc1[15];
					//y12~y15
					pDstYUV[24] = pSrc1[20];
					pDstYUV[26] = pSrc1[21];
					pDstYUV[28] = pSrc1[22];
					pDstYUV[30] = pSrc1[23];

					if (bUVSel) {
						pDstYUV[3] = pSrc1[4];
						pDstYUV[7] = pSrc1[5];
						pDstYUV[11] = pSrc1[6];
						pDstYUV[15] = pSrc1[7];

						pDstYUV[19] = pSrc1[16];
						pDstYUV[23] = pSrc1[17];
						pDstYUV[27] = pSrc1[18];
						pDstYUV[31] = pSrc1[19];

						pDstYUV[1] = pNextLineSrc1[4];
						pDstYUV[5] = pNextLineSrc1[5];
						pDstYUV[9] = pNextLineSrc1[6];
						pDstYUV[13] = pNextLineSrc1[7];

						pDstYUV[17] = pNextLineSrc1[16];
						pDstYUV[21] = pNextLineSrc1[17];
						pDstYUV[25] = pNextLineSrc1[18];
						pDstYUV[29] = pNextLineSrc1[19];

					} else {
						pDstYUV[3] = pNextLineSrc1[4];
						pDstYUV[7] = pNextLineSrc1[5];
						pDstYUV[11] = pNextLineSrc1[6];
						pDstYUV[15] = pNextLineSrc1[7];

						pDstYUV[19] = pNextLineSrc1[16];
						pDstYUV[23] = pNextLineSrc1[17];
						pDstYUV[27] = pNextLineSrc1[18];
						pDstYUV[31] = pNextLineSrc1[19];

						pDstYUV[1] = pSrc1[4];
						pDstYUV[5] = pSrc1[5];
						pDstYUV[9] = pSrc1[6];
						pDstYUV[13] = pSrc1[7];

						pDstYUV[17] = pSrc1[16];
						pDstYUV[21] = pSrc1[17];
						pDstYUV[25] = pSrc1[18];
						pDstYUV[29] = pSrc1[19];
					}
					pSrc1 += 24;
					pNextLineSrc1 += 24;
					pDstYUV += 32;
				}
				break;
			}
			case 1: {
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				//-----modify the curr line
				for (dwJ = 0; dwJ < nw; dwJ++) {
					pDstYUV[0] = pSrc1[0];
					pDstYUV += 2;
					pSrc1++;
				}

				break;
			}
			case 2: {
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				//-----modify the curr line
				for (dwJ = 0; dwJ < dwPacket; dwJ++) {
					//-----------y0 ~y3
					pDstYUV[0] = pSrc1[0];
					pDstYUV[2] = pSrc1[1];
					pDstYUV[4] = pSrc1[2];
					pDstYUV[6] = pSrc1[3];
					//y4~y7
					pDstYUV[8] = pSrc1[8];
					pDstYUV[10] = pSrc1[9];
					pDstYUV[12] = pSrc1[10];
					pDstYUV[14] = pSrc1[11];
					//----y8~y11
					pDstYUV[16] = pSrc1[12];
					pDstYUV[18] = pSrc1[13];
					pDstYUV[20] = pSrc1[14];
					pDstYUV[22] = pSrc1[15];
					//y12~y15
					pDstYUV[24] = pSrc1[20];
					pDstYUV[26] = pSrc1[21];
					pDstYUV[28] = pSrc1[22];
					pDstYUV[30] = pSrc1[23];
					pDstYUV += 32;
					pSrc1 += 24;
				}
				break;
			}
			case 3: {
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				//-----modify the curr line
				for (dwJ = 0; dwJ < nw; dwJ++) {
					pDstYUV[0] = pSrc1[0];
					pDstYUV += 2;
					pSrc1++;
				}
				break;
			}
			}
			bLineSel++;
			if (bLineSel >= 4) {
				bLineSel = 0;
			}
		}
	} else {
		//DbgPrint("nh");
		nh = nh / 2;
		for (dwI = 0; dwI < nh; dwI++) {
			switch (bLineSel) {
			case 0: {
				bUVSel = 0;
				pNextLineSrc1 = (BYTE *)pSrc1 + dwSrcPitch +
						nw; //  line2
				for (dwJ = 0; dwJ < dwPacket; dwJ++) {
					//-----------y0 ~y3
					pDstYUV[0] = pSrc1[0];
					pDstYUV[2] = pSrc1[1];
					pDstYUV[4] = pSrc1[2];
					pDstYUV[6] = pSrc1[3];
					//y4~y7
					pDstYUV[8] = pSrc1[8];
					pDstYUV[10] = pSrc1[9];
					pDstYUV[12] = pSrc1[10];
					pDstYUV[14] = pSrc1[11];
					//----y8~y11
					pDstYUV[16] = pSrc1[12];
					pDstYUV[18] = pSrc1[13];
					pDstYUV[20] = pSrc1[14];
					pDstYUV[22] = pSrc1[15];
					//y12~y15
					pDstYUV[24] = pSrc1[20];
					pDstYUV[26] = pSrc1[21];
					pDstYUV[28] = pSrc1[22];
					pDstYUV[30] = pSrc1[23];

					if (bUVSel) {
						pDstYUV[3] = pSrc1[4];
						pDstYUV[7] = pSrc1[5];
						pDstYUV[11] = pSrc1[6];
						pDstYUV[15] = pSrc1[7];

						pDstYUV[19] = pSrc1[16];
						pDstYUV[23] = pSrc1[17];
						pDstYUV[27] = pSrc1[18];
						pDstYUV[31] = pSrc1[19];

						pDstYUV[1] = pNextLineSrc1[4];
						pDstYUV[5] = pNextLineSrc1[5];
						pDstYUV[9] = pNextLineSrc1[6];
						pDstYUV[13] = pNextLineSrc1[7];

						pDstYUV[17] = pNextLineSrc1[16];
						pDstYUV[21] = pNextLineSrc1[17];
						pDstYUV[25] = pNextLineSrc1[18];
						pDstYUV[29] = pNextLineSrc1[19];

					} else {
						pDstYUV[3] = pNextLineSrc1[4];
						pDstYUV[7] = pNextLineSrc1[5];
						pDstYUV[11] = pNextLineSrc1[6];
						pDstYUV[15] = pNextLineSrc1[7];

						pDstYUV[19] = pNextLineSrc1[16];
						pDstYUV[23] = pNextLineSrc1[17];
						pDstYUV[27] = pNextLineSrc1[18];
						pDstYUV[31] = pNextLineSrc1[19];

						pDstYUV[1] = pSrc1[4];
						pDstYUV[5] = pSrc1[5];
						pDstYUV[9] = pSrc1[6];
						pDstYUV[13] = pSrc1[7];

						pDstYUV[17] = pSrc1[16];
						pDstYUV[21] = pSrc1[17];
						pDstYUV[25] = pSrc1[18];
						pDstYUV[29] = pSrc1[19];
					}
					pSrc1 += 24;
					pNextLineSrc1 += 24;
					pDstYUV += 32;
				}
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				pDstYUV = pDstYUV + nw * 2;
				break;
			}
			case 1: {
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				//-----modify the curr line
				for (dwJ = 0; dwJ < nw; dwJ++) {
					pDstYUV[0] = pSrc1[0];
					pDstYUV += 2;
					pSrc1++;
				}
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				pDstYUV = pDstYUV + nw * 2;
				break;
			}
			case 2: {
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				//-----modify the curr line
				for (dwJ = 0; dwJ < dwPacket; dwJ++) {
					//-----------y0 ~y3
					pDstYUV[0] = pSrc1[0];
					pDstYUV[2] = pSrc1[1];
					pDstYUV[4] = pSrc1[2];
					pDstYUV[6] = pSrc1[3];
					//y4~y7
					pDstYUV[8] = pSrc1[8];
					pDstYUV[10] = pSrc1[9];
					pDstYUV[12] = pSrc1[10];
					pDstYUV[14] = pSrc1[11];
					//----y8~y11
					pDstYUV[16] = pSrc1[12];
					pDstYUV[18] = pSrc1[13];
					pDstYUV[20] = pSrc1[14];
					pDstYUV[22] = pSrc1[15];
					//y12~y15
					pDstYUV[24] = pSrc1[20];
					pDstYUV[26] = pSrc1[21];
					pDstYUV[28] = pSrc1[22];
					pDstYUV[30] = pSrc1[23];
					pDstYUV += 32;
					pSrc1 += 24;
				}
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				pDstYUV = pDstYUV + nw * 2;
				break;
			}
			case 3: {
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				//-----modify the curr line
				for (dwJ = 0; dwJ < nw; dwJ++) {
					pDstYUV[0] = pSrc1[0];
					pDstYUV += 2;
					pSrc1++;
				}
				memcpy(pDstYUV, pDstYUV - (nw * 2),
				       nw * 2); // copy up line
				pDstYUV = pDstYUV + nw * 2;
				break;
			}
			}
			bLineSel++;
			if (bLineSel >= 4) {
				bLineSel = 0;
			}
		}
	}
}
//-----------
static void SetDeInterlace(BYTE *pSrc, BYTE *pOut, int nw, int nh,
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
//----------------------------------------------------------------
static void FillYUU2(BYTE *pSrc, BYTE *pOut, int nw, int nh, int interlace)
{
	BYTE *pVideoMem = pOut; // g_pD1TempBuffer;
	//	DWORD dwDstPitch = nw;
	DWORD dwSrcPitch;
	DWORD dwI, dwJ;
	DWORD dwPacket;
	BYTE *pSrc1 = (BYTE *)pSrc;
	BYTE *pDstYUV = (BYTE *)pVideoMem;
	BYTE *pNextLineSrc1;
	BOOL bUVSel;

	dwSrcPitch = nw * 12 / 8;
	dwPacket = dwSrcPitch / 24;
	pNextLineSrc1 = (BYTE *)pSrc1 + dwSrcPitch;
	if (interlace == 0) {
		for (dwI = 0; dwI < nh; dwI++) {
			bUVSel = (dwI & 0x1);
			for (dwJ = 0; dwJ < dwPacket; dwJ++) {
				//-----------y0 ~y3
				pDstYUV[0] = pSrc1[0];
				pDstYUV[2] = pSrc1[1];
				pDstYUV[4] = pSrc1[2];
				pDstYUV[6] = pSrc1[3];
				//y4~y7
				pDstYUV[8] = pSrc1[8];
				pDstYUV[10] = pSrc1[9];
				pDstYUV[12] = pSrc1[10];
				pDstYUV[14] = pSrc1[11];
				//----y8~y11
				pDstYUV[16] = pSrc1[12];
				pDstYUV[18] = pSrc1[13];
				pDstYUV[20] = pSrc1[14];
				pDstYUV[22] = pSrc1[15];
				//y12~y15
				pDstYUV[24] = pSrc1[20];
				pDstYUV[26] = pSrc1[21];
				pDstYUV[28] = pSrc1[22];
				pDstYUV[30] = pSrc1[23];

				if (bUVSel) {
					pDstYUV[3] = pSrc1[4];
					pDstYUV[7] = pSrc1[5];
					pDstYUV[11] = pSrc1[6];
					pDstYUV[15] = pSrc1[7];

					pDstYUV[19] = pSrc1[16];
					pDstYUV[23] = pSrc1[17];
					pDstYUV[27] = pSrc1[18];
					pDstYUV[31] = pSrc1[19];

					pDstYUV[1] = pNextLineSrc1[4];
					pDstYUV[5] = pNextLineSrc1[5];
					pDstYUV[9] = pNextLineSrc1[6];
					pDstYUV[13] = pNextLineSrc1[7];

					pDstYUV[17] = pNextLineSrc1[16];
					pDstYUV[21] = pNextLineSrc1[17];
					pDstYUV[25] = pNextLineSrc1[18];
					pDstYUV[29] = pNextLineSrc1[19];

				} else {
					pDstYUV[3] = pNextLineSrc1[4];
					pDstYUV[7] = pNextLineSrc1[5];
					pDstYUV[11] = pNextLineSrc1[6];
					pDstYUV[15] = pNextLineSrc1[7];

					pDstYUV[19] = pNextLineSrc1[16];
					pDstYUV[23] = pNextLineSrc1[17];
					pDstYUV[27] = pNextLineSrc1[18];
					pDstYUV[31] = pNextLineSrc1[19];

					pDstYUV[1] = pSrc1[4];
					pDstYUV[5] = pSrc1[5];
					pDstYUV[9] = pSrc1[6];
					pDstYUV[13] = pSrc1[7];

					pDstYUV[17] = pSrc1[16];
					pDstYUV[21] = pSrc1[17];
					pDstYUV[25] = pSrc1[18];
					pDstYUV[29] = pSrc1[19];
				}

				pSrc1 += 24;
				pNextLineSrc1 += 24;
				pDstYUV += 32;
			}
		}
		//--------------------------------
#if 1
		pDstYUV -= 1 * nw * 2;
		for (dwI = 0; dwI < 1; dwI++) {
			for (dwJ = 0; dwJ < nw; dwJ++) {
				pDstYUV[0] = 0x10;
				pDstYUV[1] = 0x80;
				pDstYUV += 2;
			}
		}

#endif
		//---------------------------------
	} else {
		for (dwI = 0; dwI < (nh / 2); dwI++) {
			bUVSel = (dwI & 0x1);
			for (dwJ = 0; dwJ < dwPacket; dwJ++) {
				//-----------y0 ~y3
				pDstYUV[0] = pSrc1[0];
				pDstYUV[2] = pSrc1[1];
				pDstYUV[4] = pSrc1[2];
				pDstYUV[6] = pSrc1[3];
				//y4~y7
				pDstYUV[8] = pSrc1[8];
				pDstYUV[10] = pSrc1[9];
				pDstYUV[12] = pSrc1[10];
				pDstYUV[14] = pSrc1[11];
				//----y8~y11
				pDstYUV[16] = pSrc1[12];
				pDstYUV[18] = pSrc1[13];
				pDstYUV[20] = pSrc1[14];
				pDstYUV[22] = pSrc1[15];
				//y12~y15
				pDstYUV[24] = pSrc1[20];
				pDstYUV[26] = pSrc1[21];
				pDstYUV[28] = pSrc1[22];
				pDstYUV[30] = pSrc1[23];

				if (bUVSel) {
					pDstYUV[3] = pSrc1[4];
					pDstYUV[7] = pSrc1[5];
					pDstYUV[11] = pSrc1[6];
					pDstYUV[15] = pSrc1[7];

					pDstYUV[19] = pSrc1[16];
					pDstYUV[23] = pSrc1[17];
					pDstYUV[27] = pSrc1[18];
					pDstYUV[31] = pSrc1[19];

					pDstYUV[1] = pNextLineSrc1[4];
					pDstYUV[5] = pNextLineSrc1[5];
					pDstYUV[9] = pNextLineSrc1[6];
					pDstYUV[13] = pNextLineSrc1[7];

					pDstYUV[17] = pNextLineSrc1[16];
					pDstYUV[21] = pNextLineSrc1[17];
					pDstYUV[25] = pNextLineSrc1[18];
					pDstYUV[29] = pNextLineSrc1[19];

				} else {
					pDstYUV[3] = pNextLineSrc1[4];
					pDstYUV[7] = pNextLineSrc1[5];
					pDstYUV[11] = pNextLineSrc1[6];
					pDstYUV[15] = pNextLineSrc1[7];

					pDstYUV[19] = pNextLineSrc1[16];
					pDstYUV[23] = pNextLineSrc1[17];
					pDstYUV[27] = pNextLineSrc1[18];
					pDstYUV[31] = pNextLineSrc1[19];

					pDstYUV[1] = pSrc1[4];
					pDstYUV[5] = pSrc1[5];
					pDstYUV[9] = pSrc1[6];
					pDstYUV[13] = pSrc1[7];

					pDstYUV[17] = pSrc1[16];
					pDstYUV[21] = pSrc1[17];
					pDstYUV[25] = pSrc1[18];
					pDstYUV[29] = pSrc1[19];
				}

				pSrc1 += 24;
				pNextLineSrc1 += 24;
				pDstYUV += 32;
			}
			//------------------------------
			memcpy(pDstYUV, (pDstYUV - nw * 2), nw * 2);
			pDstYUV = pDstYUV + nw * 2;
			//-----------------------------------
		}
//--------------------------------
#if 1
		pDstYUV -= 2 * nw * 2;
		for (dwI = 0; dwI < 2; dwI++) {
			for (dwJ = 0; dwJ < nw; dwJ++) {
				pDstYUV[0] = 0x10;
				pDstYUV[1] = 0x80;
				pDstYUV += 2;
			}
		}
#endif
		//---------------------------------
	}
}

static void FHD_To_800X600_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				  int out_w, int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	//   BYTE*pDestBuf;
	int *pSrcData;
	int *pDestData;
	pSrcBuf = pSrc;
	pDestData = (int *)pOut;
	for (y = 0; y < in_h; y++) {
		switch (y % 9) {
		case 0:
		case 2:
		case 4:
		case 6:
		case 8: {
			pSrcBuf = pSrc + (y * in_w * 2) + (240 * 2);
			for (x = 0; x < (in_w - 480);) {
				pSrcData = (int *)pSrcBuf;
				pDestData[0] = pSrcData[0];
				pDestData[1] = pSrcData[2];
				pDestData[2] = pSrcData[4];
				pDestData[3] = pSrcData[6];
				pDestData[4] = pSrcData[8];
				pSrcBuf += 18 * 2;
				pDestData += 5;
				x = x + 18;
			}
			break;
		}
		case 1:
		case 3:
		case 5:
		case 7: {
			break;
		}
		}
	}
}
static void HD_To_800X600_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				 int out_w, int out_h)
{
	int y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	//int *pSrcData;
	//   int *pDestData;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	for (y = 0; y < (in_h - 120); y++) {
		memcpy(pDestBuf, (pSrcBuf + 60 * in_w * 2), out_w * 2);
		pDestBuf += out_w * 2;
		pSrcBuf += in_w * 2;
	}
}
static void SD_NTSC_To_SD_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				     int out_w, int out_h)
{
	//int x,y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	//int *pSrcData;
	//int *pDestData;
	DWORD dwI;
	DWORD dwJ;
	BYTE *pDstYUV;

	pSrcBuf = pSrc;
	pDestBuf = pOut;

	// PAL 720X576  NTSC 720X480
	pDstYUV = pDestBuf;
	for (dwI = 0; dwI < 48; dwI++) {
		for (dwJ = 0; dwJ < out_w; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
	}
	pDestBuf += 48 * in_w * 2;
	memcpy(pDestBuf, pSrcBuf, in_h * in_w * 2);
	pDestBuf += in_h * in_w * 2;
	pDstYUV = pDestBuf;
	for (dwI = 0; dwI < 48; dwI++) {
		for (dwJ = 0; dwJ < out_w; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
	}
}

static void SD_PAL_To_SD_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				     int out_w, int out_h)
{
	// int x,y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	//   int *pSrcData;
	//int *pDestData;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	// PAL 720X576  NTSC 720X480
	memcpy(pDestBuf, (pSrcBuf + 48 * in_w * 2), out_h * out_w * 2);
}
static void SD_NTSC_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				  int out_w, int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	int dwJ;
	BYTE *pDstYUV;

	pSrcBuf = pSrc;
	pDestBuf = pOut;

	// PAL 720X480   x*2 =1440,(y-36)*2= 1080

	//--- 60 line
	for (y = 0; y < 60; y++) {
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < out_w; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
		pDestBuf += out_w * 2;
	}
	//---
	for (y = 0; y < (out_h - 120);) {
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < 240; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
		pDestBuf += 240 * 2;
		for (x = 0; x < 720;) {
			pDestBuf[0] = pSrcBuf[0]; //y
			pDestBuf[1] = pSrcBuf[1]; //cb

			pDestBuf[2] = pSrcBuf[0]; //y
			pDestBuf[3] = pSrcBuf[3]; //cr

			pDestBuf[4] = pSrcBuf[2]; //y
			pDestBuf[5] = pSrcBuf[1]; //cb

			pDestBuf[6] = pSrcBuf[2]; //y
			pDestBuf[7] = pSrcBuf[3]; //cr

			pSrcBuf += 4;
			pDestBuf += 8;
			x = x + 2;
		}
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < 240; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
		pDestBuf += 240 * 2;
		//------------copy line
		memcpy(pDestBuf, pDestBuf - (out_w * 2), out_w * 2);
		pDestBuf += out_w * 2;
		y = y + 2;
		//------------------
	}
	//-- 60 line
	for (y = 0; y < 60; y++) {
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < out_w; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
		pDestBuf += out_w * 2;
	}
	//---
}

static void SD_PAL_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				 int out_w, int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	int dwJ;
	BYTE *pDstYUV;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	// PAL 720X576   x*2 =1440,(y-36)*2= 1080
	pSrcBuf = pSrcBuf + 18 * in_w * 2;
	for (y = 0; y < out_h;) {
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < 240; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
		pDestBuf += 240 * 2;
		for (x = 0; x < 720;) {
			pDestBuf[0] = pSrcBuf[0]; //y
			pDestBuf[1] = pSrcBuf[1]; //cb

			pDestBuf[2] = pSrcBuf[0]; //y
			pDestBuf[3] = pSrcBuf[3]; //cr

			pDestBuf[4] = pSrcBuf[2]; //y
			pDestBuf[5] = pSrcBuf[1]; //cb

			pDestBuf[6] = pSrcBuf[2]; //y
			pDestBuf[7] = pSrcBuf[3]; //cr

			pSrcBuf += 4;
			pDestBuf += 8;
			x = x + 2;
		}
		pDstYUV = pDestBuf;
		for (dwJ = 0; dwJ < 240; dwJ++) {
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV += 2;
		}
		pDestBuf += 240 * 2;
		//------------copy line
		memcpy(pDestBuf, pDestBuf - (out_w * 2), out_w * 2);
		pDestBuf += out_w * 2;
		y = y + 2;
		//------------------
	}
}

static void SD_PAL_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				int out_w, int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	// int dwJ;
	//   BYTE*pDstYUV;
	pSrcBuf = pSrc;
	pDestBuf = pOut;

	// PAL 720X576   640X360-> 1280*720
	pSrcBuf = pSrcBuf + 108 * in_w * 2;
	for (y = 0; y < out_h;) {
		pSrcBuf += 40 * 2;
		for (x = 0; x < 640;) {
			pDestBuf[0] = pSrcBuf[0]; //y
			pDestBuf[1] = pSrcBuf[1]; //cb

			pDestBuf[2] = pSrcBuf[0]; //y
			pDestBuf[3] = pSrcBuf[3]; //cr

			pDestBuf[4] = pSrcBuf[2]; //y
			pDestBuf[5] = pSrcBuf[1]; //cb

			pDestBuf[6] = pSrcBuf[2]; //y
			pDestBuf[7] = pSrcBuf[3]; //cr

			pSrcBuf += 4;
			pDestBuf += 8;
			x = x + 2;
		}
		pSrcBuf += 40 * 2;
		//------------copy line
		memcpy(pDestBuf, pDestBuf - (out_w * 2), out_w * 2);
		pDestBuf += out_w * 2;
		y = y + 2;
		//------------------
	}
}
static void V1280X1024_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				     int out_w, int out_h)
{
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	pSrcBuf = pSrcBuf + 152 * in_w * 2;
	HD_To_FHD_Scaler(pSrcBuf, pDestBuf, 1280, 720, out_w, out_h);
}
void V1280X1024_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
			     int out_w, int out_h)
{
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	pSrcBuf = pSrcBuf + 152 * in_w * 2;
	memcpy(pDestBuf, pSrcBuf, out_h * out_w * 2);
}
void V1280X1024_To_800X600_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
				  int out_w, int out_h)
{
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	pSrcBuf = pSrcBuf + 152 * in_w * 2;
	HD_To_800X600_Scaler(pSrcBuf, pDestBuf, 1280, 720, out_w, out_h);
}

void SD_NTSC_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w,
			  int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//   int dwJ;
	//BYTE*pDstYUV;
	// PAL 720X480   640X360-> 1280*720
	pSrcBuf = pSrcBuf + 60 * in_w * 2;
	for (y = 0; y < out_h;) {
		pSrcBuf += 40 * 2;
		for (x = 0; x < 640;) {
			pDestBuf[0] = pSrcBuf[0]; //y
			pDestBuf[1] = pSrcBuf[1]; //cb

			pDestBuf[2] = pSrcBuf[0]; //y
			pDestBuf[3] = pSrcBuf[3]; //cr

			pDestBuf[4] = pSrcBuf[2]; //y
			pDestBuf[5] = pSrcBuf[1]; //cb

			pDestBuf[6] = pSrcBuf[2]; //y
			pDestBuf[7] = pSrcBuf[3]; //cr

			pSrcBuf += 4;
			pDestBuf += 8;
			x = x + 2;
		}
		pSrcBuf += 40 * 2;
		//------------copy line
		memcpy(pDestBuf, pDestBuf - (out_w * 2), out_w * 2);
		pDestBuf += out_w * 2;
		y = y + 2;
		//------------------
	}
}

void FHD_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w,
		      int out_h)

{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	int *pSrcData;
	int *pDestData;
	pSrcBuf = pSrc;
	pDestData = (int *)pOut;
	for (y = 0; y < in_h; y++) {
		if (y % 3 != 2) {
			for (x = 0; x < in_w;) {
				pSrcData = (int *)pSrcBuf;
				*pDestData = *pSrcData;
				if (x % 2 == 1) {
					pDestBuf = (BYTE *)pDestData;
					pDestBuf[1] = pSrcBuf[3];
					pDestBuf[3] = pSrcBuf[1];
				}
				pDestData += 1;
				pSrcBuf += 6;
				x = x + 3;
			}
		} else {
			pSrcBuf += in_w * 2;
		}
	}
}
void HD_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w,
		      int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	//BYTE*pDestBuf;
	DWORD *pSrcData;
	DWORD *pDestData;
	int out_w_size;
	//BYTE*pSrcTmp;
	BYTE *pDestTmp;
	out_w_size = out_w * 2;
	pDestData = (DWORD *)pOut;
	pSrcData = (DWORD *)pSrc;
	for (y = 0; y < out_h; y++) {
		if ((y % 3) == 2) {
			pSrcBuf = (BYTE *)pDestData;
			memcpy(pSrcBuf, (BYTE *)(pSrcBuf - out_w_size),
			       out_w_size);
			pDestData += out_w_size / 4;
		} else {
			for (x = 0; x < out_w;) {
				*pDestData = *pSrcData;
				pDestData++;
				pSrcData++;

				*pDestData = *pSrcData;
				pDestTmp = (BYTE *)pDestData;
				pDestTmp[2] = pDestTmp[0];
				pDestData++;

				*pDestData = *pSrcData;
				pDestTmp = (BYTE *)pDestData;
				pDestTmp[0] = pDestTmp[2];
				pSrcData++;
				pDestData++;
				x = x + 6;
			}
		}
	}
}
void FHD_To_SD_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
			   int out_w, int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	BYTE *pSrcData;
	//   BYTE *pDestData;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//-- 1920X1080 0->720X480
	pSrcBuf = pSrcBuf + 60 * in_w * 2;
	for (y = 0; y < out_h;) {
		pSrcData = pSrcBuf + 240 * 2;
		for (x = 0; x < 720;) {
			pDestBuf[0] = pSrcData[0]; //y
			pDestBuf[1] = pSrcData[1]; //cb

			pDestBuf[2] = pSrcData[4]; //y
			pDestBuf[3] = pSrcData[3]; //cr

			x = x + 2;
			pSrcData += 8;
			pDestBuf += 4;
		}
		pSrcBuf += in_w * 4;
		y = y + 1;
	}
}
void FHD_To_SD_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w,
			  int out_h)
{
	int x, y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	BYTE *pSrcData;
	BYTE *pDstYUV;
	//   BYTE *pDestData;
	int dwJ;
	pSrcBuf = pSrc;
	pDestBuf = pOut;

	//-- 1920X1080 0->720X576
	//-------------------
	pDstYUV = pDestBuf;
	for (dwJ = 0; dwJ < 18 * out_w; dwJ++) {
		pDstYUV[0] = 0x10;
		pDstYUV[1] = 0x80;
		pDstYUV += 2;
	}
	//------------------
	pDestBuf += 18 * out_w * 2;
	for (y = 0; y < 540;) {
		pSrcData = pSrcBuf + 240 * 2;
		for (x = 0; x < 720;) {
			pDestBuf[0] = pSrcData[0]; //y
			pDestBuf[1] = pSrcData[1]; //cb

			pDestBuf[2] = pSrcData[4]; //y
			pDestBuf[3] = pSrcData[3]; //cr

			x = x + 2;
			pSrcData += 8;
			pDestBuf += 4;
		}
		pSrcBuf += in_w * 4;
		y = y + 1;
	}
	//------------
	pDstYUV = pDestBuf;
	for (dwJ = 0; dwJ < 18 * out_w; dwJ++) {
		pDstYUV[0] = 0x10;
		pDstYUV[1] = 0x80;
		pDstYUV += 2;
	}
	//------------
}

void HD_To_SD_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w,
			  int out_h)
{
	int y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	//int *pSrcData;
	// int *pDestData;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//-- 1280X720 ->720X480
	pSrcBuf = pSrcBuf + 120 * in_w * 2;
	for (y = 0; y < out_h;) {
		memcpy(pDestBuf, pSrcBuf + 280 * 2, out_w * 2);
		pSrcBuf += in_w * 2;
		pDestBuf += out_w * 2;
		y = y + 1;
	}
}
void HD_To_SD_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w,
			 int out_h)
{
	int y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	//int dwJ;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//BYTE*pDstYUV;
	//-- 1920X1080 0->720X576
	pSrcBuf = pSrcBuf + 72 * in_w * 2;
	for (y = 0; y < out_h;) {
		memcpy(pDestBuf, pSrcBuf + 280 * 2, out_w * 2);
		pSrcBuf += in_w * 2;
		pDestBuf += out_w * 2;
		y = y + 1;
	}
}
void V1280X1024_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
			    int out_w, int out_h)
{
	int y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//   BYTE*pDstYUV;
	//-- 1280X1024 0->720X480
	pSrcBuf = pSrcBuf + 272 * in_w * 2;
	for (y = 0; y < out_h;) {
		memcpy(pDestBuf, pSrcBuf + 280 * 2, out_w * 2);
		pSrcBuf += in_w * 2;
		pDestBuf += out_w * 2;
		y = y + 1;
	}
}

void V1280X1024_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
			   int out_w, int out_h)
{
	int y;
	BYTE *pSrcBuf;
	BYTE *pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//   BYTE*pDstYUV;
	//-- 1280x1024 0->720X576
	pSrcBuf = pSrcBuf + 224 * in_w * 2;
	for (y = 0; y < out_h;) {
		memcpy(pDestBuf, pSrcBuf + 280 * 2, out_w * 2);
		pSrcBuf += in_w * 2;
		pDestBuf += out_w * 2;
		y = y + 1;
	}
}
static void VideoRotate90deg(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,
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

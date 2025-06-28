#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "../scaler/util/fill_nv12_yuy2.h"   /* new */
#include "../scaler/util/old_proto.h"        /* just defines BYTE/u8 */


static void LegacyFillNV12ToYUY2(BYTE *pSrc, BYTE *pOut, int nw, int nh,
			   int interlace);

static void run_case(int w, int h, bool interlaced)
{
        size_t nv12_sz = w * h * 3 / 2;   /* Y + UV */
        size_t yuy2_sz = w * h * 2;

        u8 *src  = malloc(nv12_sz);
        u8 *gold = malloc(yuy2_sz);
        u8 *test = malloc(yuy2_sz);

        /* fill src with deterministic pattern */
        for (size_t i = 0; i < nv12_sz; ++i)
                src[i] = (u8)(i * 31 + 17);

        legacy_FillNV12ToYUY2(src, gold, w, h, interlaced);
        nv12_to_yuy2        (src, test, w, h, interlaced);

        if (memcmp(gold, test, yuy2_sz) != 0) {
                fprintf(stderr,
                        "Mismatch  w=%d h=%d interlaced=%d\n", w, h, interlaced);
                abort();
        }

        free(src); free(gold); free(test);
}

int main(void)
{
        run_case(640, 360, false);  /* small progressive */
        run_case(640, 360, true);   /* small interlaced  */
        run_case(1920, 1080, false);
        run_case(1920, 1080, true);
        puts("nv12_to_yuy2 bit-exact ✅");
        return 0;
}


static void LegacyFillNV12ToYUY2(BYTE *pSrc, BYTE *pOut, int nw, int nh,
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

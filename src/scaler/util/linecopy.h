static inline void copy_luma_16(u8 *dst, const u8 *src16)
{
        /* y0-y3, y4-y7 … */
        dst[0]=src16[0]; dst[2]=src16[1]; dst[4]=src16[2]; dst[6]=src16[3];
        dst[8]=src16[8]; dst[10]=src16[9]; dst[12]=src16[10]; dst[14]=src16[11];
        dst[16]=src16[12]; dst[18]=src16[13]; dst[20]=src16[14]; dst[22]=src16[15];
        dst[24]=src16[20]; dst[26]=src16[21]; dst[28]=src16[22]; dst[30]=src16[23];
}

static inline void copy_chroma_8(u8 *dst, const u8 *uv8, bool swap)
{
        if (swap) {
                /* first line gets UV from current, second from next */
                dst[3]=uv8[4]; dst[7]=uv8[5]; dst[11]=uv8[6]; dst[15]=uv8[7];
                dst[19]=uv8[16]; dst[23]=uv8[17]; dst[27]=uv8[18]; dst[31]=uv8[19];
        } else {
                dst[1]=uv8[4]; dst[5]=uv8[5]; dst[9]=uv8[6]; dst[13]=uv8[7];
                dst[17]=uv8[16]; dst[21]=uv8[17]; dst[25]=uv8[18]; dst[29]=uv8[19];
        }
}

static inline void fill_black_line(u8 *dst, u16 w)
{
        for (u16 i=0; i<w; i++) { dst[0]=0x10; dst[1]=0x80; dst+=2; }
}


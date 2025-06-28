/* scaler/presets/preset_table.h */
struct scaler_preset {
        u16 in_w, in_h;
        u16 out_w, out_h;
        void (*fn)(u8 *src, u8 *dst,
                   int in_w,int in_h,int out_w,int out_h);
};

extern const struct scaler_preset scaler_presets[];


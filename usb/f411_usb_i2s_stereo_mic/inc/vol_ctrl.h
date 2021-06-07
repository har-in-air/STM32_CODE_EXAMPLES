#ifndef VOL_CTRL_H_
#define VOL_CTRL_H_

void vol_init(void);
void vol_set_volume(int16_t volume);
void vol_process_buffer(int16_t *iobuffer, int32_t nSamples);

#endif

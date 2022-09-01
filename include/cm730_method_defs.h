int d_setup();
int d_close();
int d_open();
int getch();
int d_on(int val);
int d_off(int val);
int kbhit(void);
int d_ping(int val);
int d_update_imu();
int d_update_imu_setup();
int d_update_imu_slow();
int d_update_ft();
int d_update_ft_setup();
double d_int2double(uint16_t val);
double d_uint2double(uint16_t val);
double d_ft_char2double(uint8_t val, int* err);
uint8_t d_read1byte(uint8_t id, uint8_t address);
int d_flush();



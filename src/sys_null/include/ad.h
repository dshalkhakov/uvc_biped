
// not defined in SDK:
typedef enum {
	ONESHOT = 1,
	SWEEP = 2,
} ad_mode_t;

extern bool ad_init(int port, ad_mode_t mode);
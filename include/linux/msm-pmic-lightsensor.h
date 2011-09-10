#ifndef __MSM_PMIC_LIGHTSENSOR_H__
#define __MSM_PMIC_LIGHTSENSOR_H__
	
struct lightsensor_conf_t {
	unsigned short poll_period_ms;
	unsigned short threshold;
	unsigned char  resolution;
};

enum lightsensor_detect_t {
	LIGHTSENSOR_VALUE_MIN = 0,
	LIGHTSENSOR_VALUE_MAX = 0xff
};

#define LIGHTSENSOR_IOC_MAGIC			'A'
#define LIGHTSENSOR_GET_CONFIG		_IOW(LIGHTSENSOR_IOC_MAGIC, 0, struct lightsensor_conf_t)
#define LIGHTSENSOR_SET_CONFIG		_IOW(LIGHTSENSOR_IOC_MAGIC, 1, struct lightsensor_conf_t)
#define LIGHTSENSOR_SET_NO_WAIT		_IOW(LIGHTSENSOR_IOC_MAGIC, 2, char)
#define LIGHTSENSOR_CLEAR_NO_WAIT	_IOW(LIGHTSENSOR_IOC_MAGIC, 3, char)

#define LIGHTSENSOR_IOC_MAXNR						3

#define LIGHTSENSOR_DEFAULT_POLL_PERIOD	500
#define LIGHTSENSOR_MIN_POLL_PERIOD			100
/* 
 * Resolution is 1/(2^(8-N)) of LIGHTSENSOR_VALUE_MAX, 
 * 0 means full resolution, 
 * 8 and any value above  means using of threshold 
 */		
#define LIGHTSENSOR_RESOLUTION_DEFAULT	 2
#define LIGHTSENSOR_RESOLUTION_THRESHOLD 8
#define LIGHTSENSOR_THRESHOLD_DEFAULT		 LIGHTSENSOR_VALUE_MAX/2

	
#endif /* __MSM_PMIC_LIGHTSENSOR_H__ */

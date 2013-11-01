
#ifndef _dhd_config_
#define _dhd_config_

#include <bcmdevs.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <wlioctl.h>

#define FW_PATH_AUTO_SELECT 1
extern char firmware_path[MOD_PARAM_PATHLEN];

typedef struct dhd_conf {
	char fw_path[MOD_PARAM_PATHLEN];		/* Firmware path */
	char nv_path[MOD_PARAM_PATHLEN];		/* NVRAM path */
	wl_country_t cspec;		/* Country cod */
	uint band;			/* Band, b:2.4G only, otherwise for auto */
	uint roam_off;		/* Roaming, 0:enable, 1:disable */
	uint roam_off_suspend;		/* Roaming in suspend, 0:enable, 1:disable */
	int roam_trigger[2];		/* The RSSI threshold to trigger roaming */
	int roam_scan_period[2];	/* Roaming scan period */
	int roam_delta[2];			/* Roaming candidate qualification delta */
	int fullroamperiod;			/* Full Roaming period */
	uint filter_out_all_packets;	/* Filter out all packets in early suspend */
	uint keep_alive_period;		/* The perioid to send keep alive packet */
} dhd_conf_t;

void dhd_conf_set_fw_name_by_chip(dhd_pub_t *dhd, char *dst, char *src);
void dhd_conf_set_hw_oob_intr(bcmsdh_info_t *sdh, uint chip);
void dhd_conf_set_fw_path(dhd_pub_t *dhd, char *fw_path);
void dhd_conf_set_nv_path(dhd_pub_t *dhd, char *nv_path);
int dhd_conf_set_country(dhd_pub_t *dhd);
int dhd_conf_get_country(dhd_pub_t *dhd);
int dhd_conf_set_band(dhd_pub_t *dhd);
uint dhd_conf_get_band(dhd_pub_t *dhd);
int dhd_conf_set_roam(dhd_pub_t *dhd);
void dhd_conf_set_bw(dhd_pub_t *dhd);
int dhd_conf_download_config(dhd_pub_t *dhd);
int dhd_conf_preinit(dhd_pub_t *dhd);
int dhd_conf_attach(dhd_pub_t *dhd);
void dhd_conf_detach(dhd_pub_t *dhd);

#endif /* _dhd_config_ */

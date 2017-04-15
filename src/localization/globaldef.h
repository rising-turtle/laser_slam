#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H

struct _PMScan;
class CPolarMatch;

extern const int g_num_of_global_particles;
extern const int g_num_of_particles;
extern const int g_num_of_weight_particles;
extern const double g_goodness_of_localization;
extern const double g_meandis_of_localization;
extern const float g_resample_threshold;

extern const float g_free_threshold;
extern const float g_occu_threshold;
extern const float g_unknown_value;
extern const float g_pixResolution;

extern const float g_maxCorrDistance;
extern const unsigned int g_scan_decimation;

extern const float g_threshold_resample;
extern const float g_noisy_particle_dis;
extern const float g_noisy_particle_ang;

extern const double g_sigma_x;
extern const double g_sigma_y;
extern const double g_sigma_th;

extern bool constructPSMfromRawSeed(char*, struct _PMScan&, CPolarMatch*);
extern void m2cm(struct _PMScan*);
extern void cm2m(struct _PMScan*);

#endif

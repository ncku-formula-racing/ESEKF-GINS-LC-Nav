/**
 * i2nav_test.c -- Benchmark against i2Nav awesome-gins-datasets
 *
 * Dataset: https://github.com/i2Nav-WHU/awesome-gins-datasets
 *
 * Getting the dataset (tests/i2nav-datasets/ is gitignored):
 *   git clone https://github.com/i2Nav-WHU/awesome-gins-datasets \
 *       tests/i2nav-datasets
 *   cd tests/i2nav-datasets/ICM20602 && 7z x ICM20602.7z && cd -
 *   (7z: `brew install p7zip` / `apt install p7zip-full`)
 *
 * Usage:
 *   ./build/tests/i2nav_test <imu.txt> <truth.nav> [gnss_hz]
 *
 * e.g.:
 *   ./build/tests/i2nav_test \
 *       tests/i2nav-datasets/ICM20602/ICM20602/ICM20602.txt \
 *       tests/i2nav-datasets/ICM20602/ICM20602/truth.nav 1 \
 *       > result.csv          # summary statistics land on stderr
 *
 * IMU file (*.txt), 7 columns:
 *   sow  dthx  dthy  dthz  dvx  dvy  dvz
 *   Incremental: dth[rad], dv[m/s]; body frame FRD
 *
 * Ground-truth file (*.nav), 11 columns:
 *   week  sow  lat  lon  h  vN  vE  vD  roll  pitch  yaw
 *   Velocities in NED [m/s], attitudes in [deg]
 *
 * Output CSV (stdout) -- all velocities in NED frame:
 *   t, gt_vN, ekf_vN, gnss_vN, ins_vN,
 *      gt_vE, ekf_vE, gnss_vE, ins_vE,
 *      gt_wz_deg, ekf_wz_deg
 *
 * Summary statistics printed to stderr:
 *   RMSE, MAE, max-error for EKF / GNSS-only / INS-only
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nav/nav.h"

// ---------------------------------------------------------------------------
// Capacity -- ADIS16465 at 200 Hz * 1617 s = 323400 rows
// ---------------------------------------------------------------------------
#define MAX_IMU_ROWS 340000
#define MAX_NAV_ROWS 340000

// ---------------------------------------------------------------------------
// Data rows
// ---------------------------------------------------------------------------
typedef struct {
    double time;
    double dth[3];  // incremental angle (rad)
    double dv[3];   // incremental velocity (m/s)
} ImuRow;

typedef struct {
    double time;
    double lat, lon, h;
    double vN, vE, vD;
    double roll, pitch, yaw;  // deg
} NavRow;

// ---------------------------------------------------------------------------
// File parsers
// ---------------------------------------------------------------------------
static int load_imu(const char *path, ImuRow *rows, int max_rows) {
    FILE *fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "Cannot open IMU file: %s\n", path);
        return -1;
    }
    int n = 0;
    char line[256];
    while (fgets(line, sizeof(line), fp) && n < max_rows) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        ImuRow *r = &rows[n];
        int ret =
            sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &r->time, &r->dth[0],
                   &r->dth[1], &r->dth[2], &r->dv[0], &r->dv[1], &r->dv[2]);
        if (ret == 7) n++;
    }
    fclose(fp);
    return n;
}

static int load_nav(const char *path, NavRow *rows, int max_rows) {
    FILE *fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "Cannot open NAV file: %s\n", path);
        return -1;
    }
    int n = 0;
    char line[512];
    while (fgets(line, sizeof(line), fp) && n < max_rows) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        NavRow *r = &rows[n];
        // Format: week  sow  lat  lon  h  vN  vE  vD  roll  pitch  yaw
        double week;
        int ret = sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                         &week, &r->time, &r->lat, &r->lon, &r->h, &r->vN,
                         &r->vE, &r->vD, &r->roll, &r->pitch, &r->yaw);
        if (ret == 11) n++;
    }
    fclose(fp);
    return n;
}

// ---------------------------------------------------------------------------
// Euler (deg, ZYX) -> quaternion [w,x,y,z]
// ---------------------------------------------------------------------------
static void euler_to_quat(double roll_deg, double pitch_deg, double yaw_deg,
                          float32_t *q) {
    double r = roll_deg * M_PI / 180.0;
    double p = pitch_deg * M_PI / 180.0;
    double y = yaw_deg * M_PI / 180.0;
    double cr = cos(r * 0.5), sr = sin(r * 0.5);
    double cp = cos(p * 0.5), sp = sin(p * 0.5);
    double cy = cos(y * 0.5), sy = sin(y * 0.5);
    q[0] = (float)(cr * cp * cy + sr * sp * sy);
    q[1] = (float)(sr * cp * cy - cr * sp * sy);
    q[2] = (float)(cr * sp * cy + sr * cp * sy);
    q[3] = (float)(cr * cp * sy - sr * sp * cy);
}

// ---------------------------------------------------------------------------
// Running statistics accumulator
// ---------------------------------------------------------------------------
typedef struct {
    double sum_sq_N, sum_sq_E;    // for RMSE
    double sum_abs_N, sum_abs_E;  // for MAE
    double max_N, max_E;          // peak error
    long count;
} Stats;

static void stats_update(Stats *s, double err_N, double err_E) {
    double aN = fabs(err_N), aE = fabs(err_E);
    s->sum_sq_N += err_N * err_N;
    s->sum_sq_E += err_E * err_E;
    s->sum_abs_N += aN;
    s->sum_abs_E += aE;
    if (aN > s->max_N) s->max_N = aN;
    if (aE > s->max_E) s->max_E = aE;
    s->count++;
}

static void stats_print(const char *label, const Stats *s) {
    if (s->count == 0) return;
    double n = (double)s->count;
    fprintf(stderr, "\n[%s] n=%ld\n", label, s->count);
    fprintf(stderr, "  RMSE  vN=%.4f  vE=%.4f  m/s\n", sqrt(s->sum_sq_N / n),
            sqrt(s->sum_sq_E / n));
    fprintf(stderr, "  MAE   vN=%.4f  vE=%.4f  m/s\n", s->sum_abs_N / n,
            s->sum_abs_E / n);
    fprintf(stderr, "  Max   vN=%.4f  vE=%.4f  m/s\n", s->max_N, s->max_E);
}

// ---------------------------------------------------------------------------
// NAV tuning -- ADIS16465-grade tactical IMU (i2Nav datasets, 200 Hz)
// ---------------------------------------------------------------------------
// Q is expressed as continuous-time PSD (variance per second), independent of
// IMU rate.  The library multiplies by dt internally each predict step.
//
//   White-noise driven states (dv, dtheta):  Q_psd = N^2
//     N is noise density (m/s/sqrt(s) for accel, rad/sqrt(s) for gyro)
//
//   Random-walk driven biases:               Q_psd = sigma_BI^2 / tau_c
//     sigma_BI = bias instability,  tau_c = bias correlation time (~1 hr)
//
// Datasheet -> noise density:
//   ARW [deg/sqrt(hr)] -> N_g [rad/sqrt(s)] = ARW * pi/180 / sqrt(3600)
//   VRW [m/s/sqrt(hr)] -> N_a [m/s/sqrt(s)] = VRW / 60
//
// Physical baselines:
//                    ADIS16465              ICM20602              Q_psd (ICM)
//   VRW              0.05  m/s/sqrt(hr)     0.2 m/s/sqrt(hr)      ~ 1.1e-5
//   ARW              0.05  deg/sqrt(hr)     0.5 deg/sqrt(hr)      ~ 2e-8
//   sigma_BI_a       50 ug                  1 mg                  ~ 2.7e-8
//   sigma_BI_g       0.05 deg/hr            5 deg/hr              ~ 2e-11
//
// The values below are ~50x physical baseline (over-sized to absorb F
// linearization, Euler integration error, float32 round-off, faster-than-tau_c
// bias drift).  Sized for the noisiest IMU in our benchmark (ICM20602).
// ---------------------------------------------------------------------------
static const NAV_Config kNavCfg = {
    .P0_diag = {0.1f, 0.1f, 0.1f,      // dv  sigma ~ 0.32 m/s
                1e-3f, 1e-3f, 1e-3f,   // dtheta  sigma ~ 0.03 rad ~ 1.8 deg
                1e-4f, 1e-4f, 1e-4f,   // db_a sigma ~ 0.01 m/s^2 ~ 1 mg
                1e-5f, 1e-5f, 1e-5f},  // db_g sigma ~ 3 mrad/s ~ 0.18 deg/s
    // Q_psd values = (old Q_diag tuned at 200 Hz) / dt = old Q * 200.
    .Q_psd_diag = {6e-4f, 6e-4f, 6e-4f,        // dv   (m/s)^2 / s
                   1.8e-5f, 1.8e-5f, 1.8e-5f,  // dtheta  rad^2 / s
                   2e-6f, 2e-6f, 2e-6f,        // db_a (m/s^2)^2 / s
                   2e-8f, 2e-8f, 2e-6f},       // db_g rad^2 / s^3 -- z larger
    .R_diag = {0.01f, 0.01f},  // GNSS vN/vE sigma^2 = (0.1 m/s)^2
    .gnss_chi2_gate = 9.21f,   // chi-square 2 DOF, 99%
};

// ---------------------------------------------------------------------------
// NAV tuning -- MTi-630 (Movella 2023: gyro 0.007 deg/s/sqrt(Hz), 8 deg/h bias)
// Swap this for kNavCfg in NAV_Init calls below when using real hardware.
//
//   sigma_g = 0.007 * sqrt(500) * pi/180 = 2.73e-3 rad/s -> Q_th = 7.5e-6
//   sigma_bg = 8 deg/h = 3.88e-5 rad/s                   -> Q_bg = 1.5e-9
//   sigma_a = 60e-6 * 9.8 * sqrt(500) = 0.013 m/s^2      -> Q_v  = 1.7e-4
//   R: PX1120S velocity accuracy 0.1 m/s -> R = 0.01
// ---------------------------------------------------------------------------
__attribute__((unused)) static const NAV_Config kMTi630Cfg = {
    .P0_diag = {0.1f, 0.1f, 0.1f, 1e-3f, 1e-3f, 1e-3f, 1e-4f, 1e-4f, 1e-4f,
                1e-5f, 1e-5f, 1e-5f},
    // Q_psd from MTi-630 datasheet (Movella 2023):
    //   accel noise 60 ug/sqrt(Hz) -> sigma_a^2 = (60e-6 * 9.81)^2 ~ 3.5e-7
    //                              -> Q_psd_v = sigma_a^2 (m^2/s^3)
    //   gyro noise 0.007 deg/s/sqrt(Hz) -> sigma_g^2 ~ 1.5e-8 rad^2/s
    //   gyro bias instability 8 deg/h, tau_c ~ 1 hr -> Q_psd_bg ~ 4e-13
    //   rad^2/s^3
    .Q_psd_diag = {3.5e-7f, 3.5e-7f, 3.5e-7f, 1.5e-8f, 1.5e-8f, 1.5e-8f, 1e-8f,
                   1e-8f, 1e-8f, 1e-12f, 1e-12f, 1e-12f},
    .R_diag = {0.01f, 0.01f},
    .gnss_chi2_gate = 9.21f,  // chi-square 2 DOF, 99%
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <imu.txt> <truth.nav> [gnss_hz]\n", argv[0]);
        return 1;
    }
    double gnss_hz = (argc >= 4) ? atof(argv[3]) : 1.0;
    if (gnss_hz <= 0.0) {
        fprintf(stderr, "gnss_hz must be > 0 (got %g)\n", gnss_hz);
        return 1;
    }

    // ------------------------------------------------------------------
    // Load data
    // ------------------------------------------------------------------
    ImuRow *imu_data = malloc(MAX_IMU_ROWS * sizeof(ImuRow));
    NavRow *nav_data = malloc(MAX_NAV_ROWS * sizeof(NavRow));
    if (!imu_data || !nav_data) {
        fprintf(stderr, "malloc failed\n");
        free(imu_data);
        free(nav_data);
        return 1;
    }
    if (!imu_data || !nav_data) {
        fprintf(stderr, "OOM\n");
        return 1;
    }

    int imu_n = load_imu(argv[1], imu_data, MAX_IMU_ROWS);
    int nav_n = load_nav(argv[2], nav_data, MAX_NAV_ROWS);

    if (imu_n < 2 || nav_n < 1) {
        fprintf(stderr, "Not enough data (imu=%d nav=%d)\n", imu_n, nav_n);
        free(imu_data);
        free(nav_data);
        return 1;
    }
    fprintf(stderr, "Loaded %d IMU rows, %d NAV rows\n", imu_n, nav_n);

    // ------------------------------------------------------------------
    // Initialise from first ground-truth row
    // ------------------------------------------------------------------
    const NavRow *gt0 = &nav_data[0];
    float32_t vel0[3] = {(float)gt0->vN, (float)gt0->vE, (float)gt0->vD};
    float32_t quat0[4];
    euler_to_quat(gt0->roll, gt0->pitch, gt0->yaw, quat0);

    // Skip IMU rows before ground-truth start
    int imu_start = 0;
    while (imu_start < imu_n - 1 && imu_data[imu_start].time < gt0->time)
        imu_start++;

    // EKF/INS context (static: ~50 KB, avoid stack overflow)
    static NAV_Context ctx_ekf;
    static NAV_Context ctx_ins;  // INS-only: same init, no EKF update
    NAV_Init(&ctx_ekf, vel0, quat0, 9.80665f, &kNavCfg);
    NAV_Init(&ctx_ins, vel0, quat0, 9.80665f, &kNavCfg);

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    double gnss_interval = 1.0 / gnss_hz;
    double last_gnss_time = gt0->time - gnss_interval;
    int nav_idx = 0;

    // Stats accumulators
    Stats st_ekf = {0}, st_gnss = {0}, st_ins = {0};

    // GNSS-only state: last received GNSS measurement
    float gnss_vN_hold = (float)gt0->vN;
    float gnss_vE_hold = (float)gt0->vE;

    // For yaw-rate ground truth (finite difference on yaw)
    double prev_yaw_deg = gt0->yaw;
    double prev_nav_time = gt0->time;

    printf(
        "t,"
        "gt_vN,ekf_vN,gnss_vN,ins_vN,"
        "gt_vE,ekf_vE,gnss_vE,ins_vE,"
        "gt_wz_dps,ekf_wz_dps\n");

    float32_t last_gyro[3] = {0};

    for (int i = imu_start; i < imu_n - 1; i++) {
        const ImuRow *cur = &imu_data[i];
        const ImuRow *next = &imu_data[i + 1];

        double dt = next->time - cur->time;
        if (dt <= 0.0 || dt > 1.0) continue;

        // Incremental -> rate
        float32_t accel[3], gyro[3];
        for (int j = 0; j < 3; j++) {
            accel[j] = (float)(cur->dv[j] / dt);
            gyro[j] = (float)(cur->dth[j] / dt);
        }

        // Propagate both EKF and INS-only
        NAV_FeedIMU(&ctx_ekf, accel, gyro, (float)dt);
        NAV_FeedIMU(&ctx_ins, accel, gyro, (float)dt);
        memcpy(last_gyro, gyro, sizeof(gyro));

        // Advance ground-truth pointer
        while (nav_idx < nav_n - 1 && nav_data[nav_idx + 1].time <= cur->time)
            nav_idx++;

        // GNSS update (EKF only)
        if (cur->time - last_gnss_time >= gnss_interval) {
            last_gnss_time = cur->time;
            gnss_vN_hold = (float)nav_data[nav_idx].vN;
            gnss_vE_hold = (float)nav_data[nav_idx].vE;
            NAV_FeedGNSS_Vel(&ctx_ekf, gnss_vN_hold, gnss_vE_hold);
        }

        // Log at 10 Hz
        static double last_log = -1.0;
        if (cur->time - last_log < 0.1) continue;
        last_log = cur->time;

        NAV_Output out_ekf;
        NAV_GetOutput(&ctx_ekf, last_gyro, &out_ekf);

        const NavRow *gt = &nav_data[nav_idx];

        // Ground-truth yaw rate (finite difference, wrap +/-180 deg)
        double dyaw = gt->yaw - prev_yaw_deg;
        if (dyaw > 180.0) dyaw -= 360.0;
        if (dyaw < -180.0) dyaw += 360.0;
        double dt_nav = gt->time - prev_nav_time;
        double gt_wz_dps = (dt_nav > 0.0) ? (dyaw / dt_nav) : 0.0;
        prev_yaw_deg = gt->yaw;
        prev_nav_time = gt->time;

        // EKF wz in deg/s (body z = yaw rate for level vehicle)
        double ekf_wz_dps = (double)out_ekf.wz * 180.0 / M_PI;

        printf(
            "%.3f,"
            "%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f\n",
            cur->time, gt->vN, (double)ctx_ekf.ins.vel[0], (double)gnss_vN_hold,
            (double)ctx_ins.ins.vel[0], gt->vE, (double)ctx_ekf.ins.vel[1],
            (double)gnss_vE_hold, (double)ctx_ins.ins.vel[1], gt_wz_dps,
            ekf_wz_dps);

        // Accumulate stats
        stats_update(&st_ekf, gt->vN - ctx_ekf.ins.vel[0],
                     gt->vE - ctx_ekf.ins.vel[1]);
        stats_update(&st_gnss, gt->vN - gnss_vN_hold, gt->vE - gnss_vE_hold);
        stats_update(&st_ins, gt->vN - ctx_ins.ins.vel[0],
                     gt->vE - ctx_ins.ins.vel[1]);
    }

    // ------------------------------------------------------------------
    // Summary
    // ------------------------------------------------------------------
    fprintf(stderr, "\n========== Velocity Error Summary ==========");
    stats_print("EKF  (GNSS/INS fused)", &st_ekf);
    stats_print("GNSS-only (zero-order hold)", &st_gnss);
    stats_print("INS-only  (pure integration)", &st_ins);
    fprintf(stderr, "\n");

    free(imu_data);
    free(nav_data);
    return 0;
}

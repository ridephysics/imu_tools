#include <unistd.h>
#include <imu_filter_madgwick/imu_filter.h>
#include <imu_filter_madgwick/stateless_orientation.h>
#include <QtMath>

#define rdvalue(dst) ({  \
    typeof(dst) __dst = (dst); \
    size_t __dstsz = sizeof(*__dst); \
    if (rdoff + __dstsz > sizeof(buf)) { \
        fprintf(stderr, "tried to read too much from internal buffer\n"); \
        return -1; \
    } \
    memcpy(__dst, &buf[rdoff], __dstsz); \
    rdoff += __dstsz; \
})

#define wrvalue(src) ({ \
    typeof(src) __src = (src); \
    size_t __srcsz = sizeof(__src); \
    int rc = fwrite(&__src, __srcsz, 1, stdout); \
    if (rc != 1) { \
        fprintf(stderr, "can't write data to stdout\n"); \
        return -1; \
    } \
})

int main(void) {
    static uint8_t buf[sizeof(uint64_t) + sizeof(double)*9 + sizeof(uint64_t) + sizeof(double)*2 + sizeof(double)*4];
    size_t bufpos = 0;
    ImuFilter filter;
    WorldFrame::WorldFrame worldframe = WorldFrame::ENU;
    float last_time;
    ssize_t nbytes;
    size_t i;

    filter.setWorldFrame(worldframe);
    filter.setAlgorithmGain(1.0);
    filter.setDriftBiasGain(0.0);

    for (;;) {
        uint64_t t_mpu;
        uint64_t t_bmp;
        double accel[3];
        double gyro[3];
        double mag[3];
        double temperature;
        double pressure;
        double quat[4];
        size_t rdoff = 0;

        nbytes = read(fileno(stdin), buf + bufpos, sizeof(buf) - bufpos);
        if (nbytes < 0) {
            if (errno == EINTR)
                continue;

            perror("read");
            return -1;
        }

        if (nbytes == 0)
            break;

        bufpos += nbytes;
        if (bufpos < sizeof(buf))
            continue;
        bufpos = 0;

        rdvalue(&t_mpu);
        for (i = 0; i < 3; i++) {
            rdvalue(&accel[i]);
        }
        for (i = 0; i < 3; i++) {
            rdvalue(&gyro[i]);
        }
        for (i = 0; i < 3; i++) {
            rdvalue(&mag[i]);
        }

        rdvalue(&t_bmp);

        rdvalue(&temperature);
        rdvalue(&pressure);

        rdvalue(&quat[0]);
        rdvalue(&quat[1]);
        rdvalue(&quat[2]);
        rdvalue(&quat[3]);

        Q_ASSERT(rdoff == sizeof(buf));

        float time = t_mpu / 1000000.0;
        float dt = time - last_time;
        last_time = time;

        filter.madgwickAHRSupdate(
            qDegreesToRadians(gyro[0]), qDegreesToRadians(gyro[1]), qDegreesToRadians(gyro[2]),
            accel[0] * 9.80665, accel[1] * 9.80665, accel[2] * 9.80665,
            mag[0] * 0.000001, mag[1] * 0.000001, mag[2] * 0.000001,
            dt);

        filter.getOrientation(quat[0], quat[1], quat[2], quat[3]);

        wrvalue(t_mpu);
        for (i = 0; i < 3; i++) {
            wrvalue(accel[i]);
        }
        for (i = 0; i < 3; i++) {
            wrvalue(gyro[i]);
        }
        for (i = 0; i < 3; i++) {
            wrvalue(mag[i]);
        }

        wrvalue(t_bmp);
        wrvalue(temperature);
        wrvalue(pressure);

        for (i = 0; i < 4; i++) {
            wrvalue(quat[i]);
        }
    }

    if (bufpos) {
        fprintf(stderr, "short read\n");
        return -1;
    }

    return 0;
}

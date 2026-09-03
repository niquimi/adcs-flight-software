#include "command_packets.h"
#include "fdir/fdir_manager.h"
#include "flight_software.h"
#include "health/health_monitor.h"
#include "mode_director.h"
#include "sensor_packet.h"
#include "types.h"

#include <iostream>

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " " #cond   \
                      << "\n";                                                 \
            ++g_fails;                                                         \
        }                                                                      \
    } while (0)

SensorPacket sane(float timestamp_s) {
    SensorPacket p{};
    p.timestamp_s = timestamp_s;
    p.gyro_x = 0.01f;
    p.mag_x = 30000.f;
    p.css_pz = 2.0f;
    p.batteryLevel = 0.80f;
    return p;
}

void test_director_safe_beats_force_mode() {
    ModeDirector director;
    ModeDirector::Input in{};
    in.current = ModeId::Pointing;
    in.timestamp_s = 100.f;
    in.boot_standby_duration_s = 0.f;
    in.rate_radps = 0.001f;
    in.ref_ok = true;
    in.rates_settled = true;
    in.force_safe = true;
    in.allow_exit_safe = false;
    in.mode_forced = true;
    in.forced_mode = ModeId::Pointing;
    CHECK(director.selectNextMode(in) == ModeId::Safe);
}

void test_health_dt_back() {
    HealthMonitor health;
    health.evaluateSensors(sane(1.0f));
    const HealthReport r = health.evaluateSensors(sane(0.5f));
    CHECK(r.dt_back);
    CHECK((r.flags() & 0x01) != 0);

    FdirManager fdir;
    fdir.reset();
    const FdirReport fr = fdir.evaluate(r);
    CHECK(fr.force_safe);
    CHECK(!fr.gate.use_gyro);
    CHECK(!fr.gate.use_mag);
    CHECK(!fr.gate.use_css);
}

void test_health_dt_skip() {
    HealthMonitor health;
    health.evaluateSensors(sane(0.0f));
    const HealthReport r = health.evaluateSensors(sane(2.0f));
    CHECK(r.dt_skip);
    CHECK(!r.dt_back);
    CHECK((r.flags() & 0x02) != 0);

    FdirManager fdir;
    fdir.reset();
    const FdirReport fr = fdir.evaluate(r);
    CHECK(!fr.force_safe);
    CHECK(!fr.gate.use_gyro);
    CHECK(fr.gate.use_mag);
    CHECK(fr.gate.use_css);
}

void test_health_gyro_oor() {
    HealthMonitor health;
    SensorPacket p = sane(1.0f);
    p.gyro_x = 5.0f;
    const HealthReport r = health.evaluateSensors(p);
    CHECK(r.gyro_oor);
    CHECK((r.flags() & 0x04) != 0);

    FdirManager fdir;
    fdir.reset();
    const FdirReport fr = fdir.evaluate(r);
    CHECK(fr.force_safe);
    CHECK(!fr.gate.use_gyro);
    CHECK(fr.gate.use_mag);
    CHECK(fr.gate.use_css);
}

void test_fsw_gyro_oor_not_pointing() {
    FlightSoftware fsw;
    fsw.setBootStandbyDuration(0.f);
    fsw.applyTelecommand(TC_FORCE_MODE, static_cast<std::uint8_t>(ModeId::Pointing), 0);

    SensorPacket p = sane(1.0f);
    p.gyro_x = 5.0f;
    const AttitudeCommand cmd = fsw.step(p);

    CHECK(cmd.active_mode == ModeId::Safe);
    CHECK((cmd.health_flags & 0x04) != 0);
    CHECK(cmd.last_tc_opcode == TC_FORCE_MODE);
    CHECK(cmd.last_tc_arg0 == static_cast<std::uint8_t>(ModeId::Pointing));
}

}  // namespace

int main() {
    test_director_safe_beats_force_mode();
    test_health_dt_back();
    test_health_dt_skip();
    test_health_gyro_oor();
    test_fsw_gyro_oor_not_pointing();

    if (g_fails != 0) {
        std::cerr << g_fails << " check(s) failed\n";
        return 1;
    }
    std::cout << "fsw_tests: all passed\n";
    return 0;
}

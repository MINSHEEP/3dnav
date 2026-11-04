#include "small_point_lio/odometry_guard_policy.h"
#include <iostream>
#include <limits>
#include <stdexcept>

using small_point_lio::IntervalCompensationLease;
using small_point_lio::exceeds_position_step;

static void require(bool value) {
    if (!value) throw std::runtime_error("interval guard regression");
}

int main() {
    for (auto dt : {.498, .500}) {
        const double step = (dt < .5 ? .359 : .418) * dt;
        require(!exceeds_position_step(dt, step, .15, .8, true));
        require(exceeds_position_step(dt, step, .15, .8, false));
    }
    require(exceeds_position_step(.1, .20, .15, .8, true));
    require(exceeds_position_step(.5, .31, .15, .8, true));
    require(exceeds_position_step(.5, .21, .15, .3, true));
    require(exceeds_position_step(.1, std::numeric_limits<double>::quiet_NaN(), .15, .8, true));
    require(exceeds_position_step(-.1, .01, .15, .8, true));
    // Full legacy equivalence over valid sample intervals.
    for (int i=1; i<=100; ++i)
        for (int j=0; j<=50; ++j) {
            double dt=i*.01, step=j*.01;
            require(exceeds_position_step(dt,step,.15,.8,false) == (dt<=.5 && step>.15));
        }
    IntervalCompensationLease lease;
    require(!lease.enabled(10000000000LL));
    lease.observe(true,10000000000LL);
    require(lease.enabled(10500000000LL));
    require(!lease.enabled(9999999999LL));
    require(!lease.enabled(11000000000LL));
    lease.observe(true,12000000000LL);
    lease.observe(false,12100000000LL);
    require(!lease.enabled(12100000000LL));
    std::cout << "interval guard: recorded steps, jumps, legacy equivalence and lease passed\n";
}

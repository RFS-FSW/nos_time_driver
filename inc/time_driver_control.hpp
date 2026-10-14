#ifndef NOS3_TIME_DRIVER_CONTROL_HPP
#define NOS3_TIME_DRIVER_CONTROL_HPP
#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

namespace Nos3 {
// Pure command transition; rejected commands leave both output values unchanged.
// UINT_MAX is reserved for UNPAUSE and may never be a finite target tick.
inline bool apply_time_command(const std::string& text, int64_t sim_period,
    double epoch, unsigned now, int64_t& real_period, unsigned& pause, std::string& reply)
{
    if (text.size() > 256 || text.find('\0') != std::string::npos ||
        sim_period <= 0 || real_period <= 0 || !std::isfinite(epoch)) return false;
    std::istringstream input(text);
    std::string op, arg, extra;
    if (!(input >> op)) return false;
    std::transform(op.begin(), op.end(), op.begin(),
        [](unsigned char c) { return std::toupper(c); });
    const bool numeric = op == "RUN" || op == "UNTIL";
    if (numeric) {
        if (!(input >> arg) || input >> extra) return false;
    } else if (input >> extra) return false;
    if (op == "HELP") {
        reply = "PAUSE, UNPAUSE, DECREASE, INCREASE, RUN <seconds>, UNTIL <epoch seconds>";
    } else if (op == "PAUSE") pause = now;
    else if (op == "UNPAUSE") pause = UINT_MAX;
    else if (op == "DECREASE") {
        const int64_t minimum = std::max<int64_t>(1, sim_period / 200 + (sim_period % 200 != 0));
        real_period = std::max(minimum, real_period / 2);
    } else if (op == "INCREASE") {
        const int64_t maximum = sim_period > INT64_MAX / 200 ? INT64_MAX : sim_period * 200;
        real_period = real_period > maximum / 2 ? maximum : real_period * 2;
    } else if (numeric) {
        long double value;
        try {
            size_t used;
            value = std::stold(arg, &used);
            if (used != arg.size() || !std::isfinite(value) || value < 0) return false;
        } catch (const std::exception&) { return false; }
        long double ticks = op == "RUN"
            ? now + value * 1000000.0L / sim_period
            : (value - epoch) * 1000000.0L / sim_period;
        if (!std::isfinite(ticks) || ticks < now || ticks > UINT_MAX - 1.0L) return false;
        pause = static_cast<unsigned>(ticks);
    } else return false;
    if (op != "HELP") reply = op;
    return true;
}
}
#endif

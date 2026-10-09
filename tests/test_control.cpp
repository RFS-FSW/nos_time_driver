#include <time_driver_control.hpp>
#include <cassert>
#include <iostream>
int main() {
    int64_t period = 10000;
    unsigned pause = UINT_MAX;
    std::string reply;
    auto command = [&](const std::string& c) { return Nos3::apply_time_command(c, 10000, 100, 100, period, pause, reply); };
    for (int i=0;i<1000;++i) { assert(command("DECREASE")); assert(period >= 50); volatile auto rate=1000000/period;(void)rate; }
    assert(period==50);
    for (int i=0;i<1000;++i) { assert(command("INCREASE")); assert(period <= 2000000); }
    assert(period==2000000);
    assert(command("RUN 0"));assert(pause==100);
    assert(command("RUN 1.25"));assert(pause==225);
    assert(command("UNTIL 102"));assert(pause==200);
    for (const auto c : {"RUN nonsense","RUN 1garbage","RUN NaN","RUN INF","RUN -1",
            "RUN 1e5000","RUN 1e40","UNTIL 99","PAUSEjunk","INCREASE junk","RUN 1 2","RUN",""}) {
        auto before=period;auto target=pause;
        assert(!command(c));assert(period==before && pause==target);
    }
    assert(!command(std::string("RUN 1\0junk",10)));
    assert(command("pause")); assert(pause==100);
    assert(command("UNPAUSE"));assert(pause==UINT_MAX);
    period=INT64_MAX;
    assert(Nos3::apply_time_command("INCREASE",INT64_MAX,0,0,period,pause,reply));
    assert(period==INT64_MAX);
    assert(!Nos3::apply_time_command("RUN 1",0,0,0,period,pause,reply));
    assert(!Nos3::apply_time_command("RUN 1",1,0,UINT_MAX-2,period,pause,reply));
    std::cout << "time control boundary and malformed-input checks PASS\n";
}

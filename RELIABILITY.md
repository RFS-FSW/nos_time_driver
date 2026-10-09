# Bounded time commands

Both terminal keys and NOS command messages use the same validated transition.
RUN and UNTIL require one complete finite nonnegative number and reject targets
in the past or beyond the finite unsigned tick range. Invalid commands leave
the state unchanged. The UINT_MAX value is reserved for UNPAUSE.

DECREASE and INCREASE saturate at a positive period and a bounded speed range;
they cannot halve the period to zero or overflow on doubling. Terminal numeric
entry is nonblocking (Enter submits, Escape cancels). Command callbacks and the
run loop share a mutex. Tick pacing uses steady_clock, and the counter stops
before overflow. Real periods retain microsecond precision.

Validation: build against the NOS SDK as part of NOS3X; independently compile
and run tests/test_control.cpp with C++11 and UBSan:
    g++ -std=c++11 -fsanitize=undefined -fno-sanitize-recover=all -Iinc tests/test_control.cpp -o /tmp/nos-time-control-test
    /tmp/nos-time-control-test

This fixes command safety; it does not synchronize NOS ticks to the separate
Basilisk physics scheduler.

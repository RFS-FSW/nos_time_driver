/* Copyright (C) 2009 - 2021 National Aeronautics and Space Administration. All Foreign Rights are Reserved to the U.S. Government.

This software is provided "as is" without any warranty of any, kind either express, implied, or statutory, including, but not
limited to, any warranty that the software will conform to, specifications any implied warranties of merchantability, fitness
for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the program, or
any warranty that the software will be error free.

In no event shall NASA be liable for any damages, including, but not limited to direct, indirect, special or consequential damages,
arising out of, resulting from, or in any way connected with the software or its documentation.  Whether or not based upon warranty,
contract, tort or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software,
documentation or services provided hereunder

ITC Team
NASA IV&V
ivv-itc@lists.nasa.gov
*/

/* Standard Includes */
#include <thread>
#include <string>
#include <curses.h>
#include <climits>
#include <chrono>
#include <stdexcept>
#include <time_driver_control.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <ItcLogger/Logger.hpp>

#include <Client/Bus.hpp>

#include <sim_hardware_model_factory.hpp>
#include <sim_i_hardware_model.hpp>
#include <sim_config.hpp>
#include <sim_coordinate_transformations.hpp>
#include <time_driver.hpp>

namespace Nos3
{
    REGISTER_HARDWARE_MODEL(TimeDriver,"TimeDriver");

    ItcLogger::Logger *sim_logger;

    /*************************************************************************
     * Constructors / destructors
     *************************************************************************/

    TimeDriver::TimeDriver(const boost::property_tree::ptree& config) : SimIHardwareModel(config),
        _active(config.get("simulator.active", true)),
        _time_counter(0), _display_counter(0), _pause_ticks(UINT_MAX)
    {
        if (_sim_microseconds_per_tick <= 0 || _real_microseconds_per_tick <= 0)
            throw std::invalid_argument("TimeDriver tick periods must be positive");
        std::string default_time_uri = config.get("common.nos-connection-string", "tcp://127.0.0.1:12001");
        std::string default_time_bus_name = "command";
        struct TimeBusInfo tbi;

        if (_active) 
        {
            sim_logger->debug("TimeDriver::TimeDriver: Creating time sender\n");

            // Use config data if it exists
            if (config.get_child_optional("simulator.hardware-model.connections")) 
            {
                BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, config.get_child("simulator.hardware-model.connections")) 
                {
                    std::ostringstream oss;
                    write_xml(oss, v.second);
                    sim_logger->trace("TimeDriver::TimeDriver - simulator.hardware-model.connections.connection subtree:\n%s", oss.str().c_str());

                    // v.first is the name of the child.
                    // v.second is the child tree.
                    if (v.second.get("type", "").compare("time") == 0) {
                        tbi.time_bus_name = v.second.get("bus-name", default_time_bus_name);
                        tbi.time_uri = v.second.get("nos-connection-string-override", default_time_uri);
                        bool found = false;
                        // Prefer slow search of vector in constructor so that the run method has fast indexing/iteration
                        for (unsigned int i = 0; i < _time_bus_info.size(); i++) {
                            if ((tbi.time_bus_name.compare(_time_bus_info[i].time_bus_name) == 0) &&
                                (tbi.time_uri.compare(_time_bus_info[i].time_uri) == 0)              ) {
                                    found = true;
                                    break;
                            }
                        }
                        if (!found) {
                            tbi.time_bus.reset(new NosEngine::Client::Bus(_hub, tbi.time_uri, tbi.time_bus_name));
                            tbi.time_bus->enable_set_time();
                            _time_bus_info.push_back(std::move(tbi));
                        }
                    }
                }
            }

            sim_logger->debug("TimeDriver::TimeDriver: Time sender created!\n");
        }
    }

    /*************************************************************************
     * Mutating public worker methods
     *************************************************************************/

    void TimeDriver::run(void)
    {
        if (!_active) {
            sim_logger->info("Time driver is not active");
            return;
        }
        initscr();
        erase();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        noecho();
        auto previous = std::chrono::steady_clock::now();
        std::string entered;
        while (true) {
            const auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(_control_mutex);
                int key = getch();
                std::string command, reply;
                if (!entered.empty()) {
                    if (key == '\n' || key == '\r') { command = entered; entered.clear(); }
                    else if (key == 27) entered.clear();
                    else if (key == KEY_BACKSPACE || key == 127) entered.pop_back();
                    else if (key >= 32 && key < 127 && entered.size() < 256) entered += char(key);
                } else {
                    switch (key) {
                    case 'p': case 'P': command = _pause_ticks == UINT_MAX ? "PAUSE" : "UNPAUSE"; break;
                    case '+': command = "DECREASE"; break;
                    case '-': command = "INCREASE"; break;
                    case 'r': case 'R': entered = "RUN "; break;
                    case 'u': case 'U': entered = "UNTIL "; break;
                    }
                }
                if (!command.empty() && !apply_time_command(command, _sim_microseconds_per_tick,
                        _absolute_start_time, _time_counter, _real_microseconds_per_tick, _pause_ticks, reply))
                    sim_logger->warning("Invalid time command");
                _last_time_diff = std::chrono::duration<double, std::micro>(now - previous).count();
                if (_last_time_diff >= _real_microseconds_per_tick) {
                    previous = now;
                    const int64_t ticks_per_second = 1000000 / _real_microseconds_per_tick;
                    if (ticks_per_second < 10 || _display_counter % (ticks_per_second/10) == 0)
                        update_display();
                    if (_time_counter < _pause_ticks && _time_counter < UINT_MAX - 1) {
                        update_time_busses();
                        ++_time_counter;
                    }
                    ++_display_counter;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }

    void TimeDriver::update_time_busses(void)
    {
        for (unsigned int i = 0; i < _time_bus_info.size(); i++) {

            if(!_time_bus_info[i].time_bus->is_connected())
            {
                sim_logger->info("time bus disconnected... reconnecting");
                _time_bus_info[i].time_bus.reset(new NosEngine::Client::Bus(_hub, _time_bus_info[i].time_uri, _time_bus_info[i].time_bus_name));
                _time_bus_info[i].time_bus->enable_set_time();
            }
            _time_bus_info[i].time_bus->set_time(_time_counter);
        }
    }

    void TimeDriver::update_display(void)
    {
        int32_t year, month, day, hour, minute;
        double second;

        wmove(stdscr, 0, 0);
        double abs_time = _absolute_start_time + (double(_time_counter) * double(_sim_microseconds_per_tick)) / 1000000.0;
        SimCoordinateTransformations::AbsTime2YMDHMS(abs_time, year, month, day, hour, minute, second);
        double speed_up = ((double)_sim_microseconds_per_tick) / ((double)_real_microseconds_per_tick);
        double actual_speed_up = (double)_sim_microseconds_per_tick / _last_time_diff;

        printw("TimeDriver::send_tick_to_nos_engine:\n");
        printw("  tick = %d, absolute time = %f = %4.4d/%2.2d/%2.2dT%2.2d:%2.2d:%05.2f\n", _time_counter, abs_time, year, month, day, hour, minute, second);
        printw("  real microseconds per tick = %ld, ", _real_microseconds_per_tick);
        printw("attempted speed-up = %5.2f\n", speed_up);
        printw("  actual speed-up = %5.2f, state = %s", actual_speed_up, (_pause_ticks <= _time_counter) ? "paused" : ((_pause_ticks < UINT_MAX) && (_pause_ticks > _time_counter)) ? "pausing" : "playing");
        if ((_pause_ticks < UINT_MAX) && (_pause_ticks > _time_counter)) printw(" at %f", _absolute_start_time + (double(_pause_ticks) * double(_sim_microseconds_per_tick)) / 1000000.0);
        printw("\n\nPress: 'p' to pause/play,\n       '+' to decrease delay by 2x,\n       '-' to increase delay by 2x\n");
        printw(    "       'r <number>' to run <number> more seconds,\n       'u <number>' to run until <number> absolute time\n");
        refresh();
    }

    double TimeDriver::time_diff(void)
    {
        double then, now, diff;

        then = (double)_then.tv_sec*1000000 + (double)_then.tv_usec;
        now = (double)_now.tv_sec*1000000 + (double)_now.tv_usec;

        diff = (now - then);

        return diff;
    }

    void TimeDriver::command_callback(NosEngine::Common::Message msg)
    {
        std::string response = "INVALID COMMAND";
        NosEngine::Common::DataBufferOverlay dbf(const_cast<NosEngine::Utility::Buffer&>(msg.buffer));
        if (dbf.len <= 257) {
            std::string command(dbf.data, dbf.len);
            if (!command.empty() && command.back() == '\0') command.pop_back();
            std::lock_guard<std::mutex> lock(_control_mutex);
            apply_time_command(command, _sim_microseconds_per_tick, _absolute_start_time,
                _time_counter, _real_microseconds_per_tick, _pause_ticks, response);
        }
        _command_node->send_reply_message_async(msg, response.size(), response.c_str());
    }
}

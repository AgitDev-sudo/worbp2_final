#ifndef INCLUDE_SIMULATION_SSC32U_COMMAND_PARSER_HPP_
#define INCLUDE_SIMULATION_SSC32U_COMMAND_PARSER_HPP_
#include <string>
#include <map>
#include <optional>
#include <cstdint>
#include <vector>

namespace ssc32u
{

    enum class SSC32UCommandType {
        SINGLE,
        GROUP,
        STATUS_QUERY,
        EMERGENCY_STOP,
        STOP_SERVO,
        UNKNOWN
    };
    

    struct SingleServoCommand {
        uint8_t pin_;         // pin number
        uint16_t pulse_width_;      // pulse width in µs
        std::optional<uint16_t> speed_us_s_; // speed in µs/sec
        std::optional<uint16_t> time_ms_; //time in ms for single servo
    };
    
    struct SSC32UCommand {
        SSC32UCommandType type_;
        std::vector<SingleServoCommand> servos_;
        std::optional<uint8_t> stop_pin_;    // pin number to stop when we receive stop command
        std::optional<uint16_t> duration_ms_; //total time in ms for group move
    };
    
    class SSC32UCommandParser {
    public:
        static SSC32UCommand parse(const std::string& msg);
    private:
        static void handleUnknownCommand(const std::string& raw_command, const std::string& reason="");
    };
}






#endif // INCLUDE_SIMULATION_SSC32U_PARSER_HPP_
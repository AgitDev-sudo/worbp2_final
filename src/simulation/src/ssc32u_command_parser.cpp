#include "ssc32u_command_parser.hpp"
#include <iostream>
#include <regex>
#include <algorithm>
#include <math.h>

namespace ssc32u {

    SSC32UCommand SSC32UCommandParser::parse(const std::string& msg) {
        SSC32UCommand command;
        command.type_ = SSC32UCommandType::UNKNOWN;

		/*SSC32 protocol states that every message MUST end on a <cr>*/
		std::size_t pos = msg.find('\r');
		if(pos==std::string::npos){
			handleUnknownCommand(msg, "message does not have a carriage return");
			return command; //No <cr> means not a valid message;

		}
		else if(pos != msg.size()-1) {
			handleUnknownCommand(msg, "message does not end on a carriage return");
			return command; //No <cr> at the end, means not a valid message;
		}
		/*  End checking on <cr>*/

		/*From here on we check what kind of command the message means*/
		const std::size_t n_pulse = std::count(msg.begin(),msg.end(),'P');
		const std::size_t time_counted  = std::count(msg.begin(),msg.end(),'T');
		const std::size_t speed_counted = std::count(msg.begin(),msg.end(),'S');
		const bool time_exist  = time_counted > 0;
		const bool speed_exist = speed_counted > 0;
		/*order of arguments*/
		std::string channel = "#([[:digit:]]+)[[:space:]]*";    //required argument
		std::string pulse_width ="P([[:digit:]]+)[[:space:]]*"; //required argument
		std::string speed = "S([[:digit:]]+)[[:space:]]*"; //optional argument
		std::string time = "T([[:digit:]]+)[[:space:]]*"; //optional argument (when using a group command, this must always be the last and only time argument)
		std::string carriage_return = "\r"; //required argument
		std::string stop ="STOP";
		std::string stop_arg = "[[:space:]]*([[:digit:]]+)[[:space:]]*";
		std::string movement_status = "Q";
		std::smatch match;

		if(std::regex_match(msg, match, std::regex(stop+carriage_return))) {
            command.type_ = SSC32UCommandType::EMERGENCY_STOP;
		}
		else if(std::regex_match(msg, match, std::regex(stop+stop_arg+carriage_return))) {
			uint16_t pin = static_cast<uint16_t>(std::stoi(match[1]));
            command.type_ = SSC32UCommandType::STOP_SERVO;
            command.stop_pin_ = pin;            
		}
		else if(std::regex_match(msg, match, std::regex(movement_status+carriage_return))) {
            command.type_ = SSC32UCommandType::STATUS_QUERY;
		}
		/*parse single servo command*/
		else if(n_pulse == 1) {
			SingleServoCommand servo;
            bool valid_command = true;
			if(time_exist && speed_exist) {
				if(std::regex_match(msg, match, std::regex(channel+pulse_width+speed+time+carriage_return))) {
					servo.pin_ = static_cast<uint8_t>(std::stoi(match[1]));
					servo.pulse_width_ = static_cast<uint16_t>(std::stoi(match[2]));
					servo.speed_us_s_ = static_cast<uint16_t>(std::stoi(match[3]));
					servo.time_ms_ = static_cast<uint16_t>(std::stoi(match[4]));
					
				}
			} else if(speed_exist) {
				if(std::regex_match(msg, match, std::regex(channel+pulse_width+speed+carriage_return))) {
					servo.pin_ = static_cast<uint8_t>(std::stoi(match[1]));
					servo.pulse_width_ = static_cast<uint16_t>(std::stoi(match[2]));
					servo.speed_us_s_ = static_cast<uint16_t>(std::stoi(match[3]));
				}

			} else if(time_exist) {
				if(std::regex_match(msg, match, std::regex(channel+pulse_width+time+carriage_return))) {
					servo.pin_ = static_cast<uint8_t>(std::stoi(match[1]));
					servo.pulse_width_ = static_cast<uint16_t>(std::stoi(match[2]));
					servo.time_ms_ = static_cast<uint16_t>(std::stoi(match[3]));
				}
			} else {
				if(std::regex_match(msg, match, std::regex(channel+pulse_width+carriage_return))) {
					servo.pin_ = static_cast<uint8_t>(std::stoi(match[1]));
					servo.pulse_width_ = static_cast<uint16_t>(std::stoi(match[2]));
				} else {
                    valid_command = false;
                }
			}

			if(valid_command) {
				command.type_ = SSC32UCommandType::SINGLE;
                command.servos_.push_back(servo);
            }else {
				handleUnknownCommand(msg,"Command message is not valid");
			}
		}
		/*Parse multiple servo command = Group Move*/
		/*Any move that involves more than one servo and uses either the S or T modifier is considered aGroup Move*/
		else if(n_pulse > 1 && (speed_exist || time_exist) ) {
			std::size_t time_count = 0;
			std::size_t speed_count = 0;
		
			std::string search_command(msg);
			std::vector<SingleServoCommand> servos;
			//first we search, filter and process messages that contains channel,pwm and speed value
			while(std::regex_search(search_command,match,std::regex(channel+pulse_width+speed)))
            {
                SingleServoCommand servo;
                servo.pin_ = static_cast<uint8_t>(std::stoi(match[1]));
                servo.pulse_width_ = static_cast<uint16_t>(std::stoi(match[2]));
                servo.speed_us_s_ = static_cast<uint16_t>(std::stoi(match[3]));
                servos.push_back(servo);
 
				search_command = match.prefix().str() + match.suffix().str();
				++speed_count;
			}
			//second we search, filter and process messages that contains only channel and pwm value.
			while(std::regex_search(search_command,match,std::regex(channel+pulse_width))) 
            {
                SingleServoCommand servo;
                servo.pin_ = static_cast<uint8_t>(std::stoi(match[1]));
                servo.pulse_width_ = static_cast<uint16_t>(std::stoi(match[2]));
                servos.push_back(servo);
                search_command = match.prefix().str() + match.suffix().str();
			}
			//last we search and process the time value.
			//time should be placed at the end of the line.
			if(time_exist && std::regex_search(search_command, match, std::regex(time+carriage_return))) {

                command.duration_ms_ = static_cast<uint16_t>(std::stoi(match[1]));
				++time_count;
			}

			if(servos.size() == n_pulse && time_counted == time_count && speed_counted == speed_count)
			{
                command.type_ = SSC32UCommandType::GROUP;
                command.servos_ = servos;
			} else {
				handleUnknownCommand(msg,"Invalid Group Command");
			}
		} else if (n_pulse > 1 && (!speed_exist || !time_exist)) {
			handleUnknownCommand(msg, "Invalid Group Command, at least one speed or time argument needed");
		}
		else {
			handleUnknownCommand(msg,"Invalid message in function parseMessage");
		}

        return command;
    }

    void SSC32UCommandParser::handleUnknownCommand(const std::string& raw_command, const std::string& reason) {
        // Handle unknown command
        // This could involve logging the error, throwing an exception, etc.
        std::cerr << "Error: " << reason << " - Command: " << raw_command << std::endl;
    }
}

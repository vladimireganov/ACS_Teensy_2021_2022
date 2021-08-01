#ifndef RADIO_COMMANDS
#define RADIO_COMMANDS
#include <string>
#include <map>
#include <vector>

std::vector<std::string> command_list {
    "echo",
    "arm_system",
    "arm_bbb",
    "auto_data",
    "sensor_info",
    "bno_info",
    "uefs",
    "uefe",
    "reset",
    "start",
    "broadcast",
    "acs_test",
};

class Command {
    public:
    char command_id;
    bool state;
    Command(char command_id, bool state){
        command_id = command_id;
        state = state;
    }
    
};

class RadioCommand {
    public:
    std::map<std::string, char> command_id_from_name;
    std::map<char, std::string> command_name_from_id;
    std::map<std::string, bool> command_state;

    RadioCommand() {
        for (size_t k = 0; k < command_list.size(); k++) {
            command_name_from_id.insert(std::pair<char, std::string>(char(k+97), command_list[k]));
            command_id_from_name.insert(std::pair<std::string, char>(command_list[k], char(k+97)));
            command_state.insert(std::pair<std::string, bool>(command_list[k], false));
        }
    }
};
#endif

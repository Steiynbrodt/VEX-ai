#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
//#include <filesystem>
/**#include "torch/torch.h"
#include <filesystem>
#include "vex.h"
#include "eigen"
using namespace std;
namespace fs = std::filesystem;

struct ModelArgs {
    int hidden_size;
    int num_layers;
    int num_heads;
    float dropout;
};

class VexRobotNavigator {
public:
    VexRobotNavigator(const ModelArgs& args) : args(args) {
        vex::brain Brain;
        vex::motor LeftMotor(vex::PORT1);
        vex::motor RightMotor(vex::PORT2);
    }

    torch::Tensor forward(torch::Tensor sensor_data) {
        // Process sensor data for navigation
        return sensor_data;
    }

    void navigate(torch::Tensor sensor_data) {
        torch::Tensor action = forward(sensor_data);
        int decision = action.argmax().item<int>();
        execute_action(decision);
    }

private:
    ModelArgs args;
    vex::brain Brain;
    vex::motor LeftMotor;
    vex::motor RightMotor;

    void execute_action(int action) {
        switch(action) {
            case 0:
                LeftMotor.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
                RightMotor.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
                break;
            case 1:
                LeftMotor.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
                RightMotor.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
                break;
            case 2:
                LeftMotor.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
                RightMotor.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
                break;
            case 3:
                LeftMotor.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
                RightMotor.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
                break;
            default:
                LeftMotor.stop();
                RightMotor.stop();
        }
    }
};

ModelArgs load_config(const string& filename) {
    ifstream file(filename);
    json config;
    file >> config;

    return ModelArgs{
        config["hidden_size"],
        config["num_layers"],
        config["num_heads"],
        config["dropout"]
    };
}

int main() {
    ModelArgs args = load_config("config.json");
    VexRobotNavigator navigator(args);
    
    torch::Tensor sensor_data = torch::rand({1, args.hidden_size});
    navigator.navigate(sensor_data);
    
    return 0;
}

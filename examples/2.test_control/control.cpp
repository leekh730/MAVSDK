#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <future>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>

using namespace mavsdk;
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;

bool offboard_ctrl_body(std::shared_ptr<mavsdk::Offboard>);
// rotate vehicle(1)
// bool offboard_ctrl_body(std::shared_ptr <mavsdk::Offboard> offboard){
//     Offboard::Attitude control_stick{}; //Send once before starting offboard
//     offboard->set_attitude(control_stick);
//     Offboard::Result offboard_result = offboard->start();
    
//     if(offboard_result != Offboard::Result::Success){return 1;}

//     control_stick.roll_deg = 30.0f;
//     control_stick.thrust_value = 0.6f;
//     offboard -> set_attitude(control_stick);
//     this_thread::sleep_for(chrono::seconds(5));
//     offboard_result = offboard->stop(); // Send it once before ending offboard
//     return true;
// }
// rotate vehicle(2)
bool offboard_ctrl_body(std::shared_ptr<mavsdk::Offboard> offboard){
    Offboard::VelocityBodyYawspeed control_stick{}; // before starting offboard
    offboard->set_velocity_body(control_stick);
    Offboard::Result offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::Success) { return 1; } // Failed
    control_stick.down_m_s = 0.0f; 
    control_stick.yawspeed_deg_s = 60.0f;
    offboard->set_velocity_body(control_stick);
    this_thread::sleep_for(chrono::seconds(5));
    offboard_result = offboard->stop(); // Send it once before ending offboard
    return true;
}

int main(int argc, char** argv){ 
    // 1.connect_result ------------------------------------------------------------------  
    Mavsdk mavsdk;
    ConnectionResult connection_result;
    bool discovered_system = false;

    connection_result = mavsdk.add_any_connection(argv[1]);

    if(connection_result != ConnectionResult::Success){return 1;} //Failed

    mavsdk.subscribe_on_new_system([&mavsdk, &discovered_system](){
        const auto system = mavsdk.systems().at(0);
        if(system -> is_connected()){discovered_system = true;} //Discovered system
    }); //Waiting to discover system

    this_thread::sleep_for(chrono::seconds(2)); //hearbeats at 1Hz

    if(!discovered_system){return 1;} //No system found

    const auto system = mavsdk.systems().at(0);

    system -> register_component_discovered_callback(
        [](ComponentType com_type) -> void{cout << unsigned(com_type);
        }); //Register a callback components(camera,gimbal) etc are found
    
    // 2.regist_telemetry -------------------------------------------------------------------
    auto telemetry = make_shared<Telemetry>(system); //We want to listen to the altitude of drone at 1Hz
    const Telemetry::Result set_rate_result = telemetry -> set_rate_position(1.0);

    if(set_rate_result != Telemetry::Result::Success){return 1;} //Set rate failed
    telemetry->subscribe_position([](Telemetry::Position position) {
        cout << " Home latitude : " << position.latitude_deg << endl;
        cout << " Home longitude : " << position.longitude_deg << endl;
    });
    this_thread::sleep_for(chrono::seconds(1)); // Not finished mission

    telemetry -> subscribe_position([](Telemetry::Position position){
        cout << " Altitude : " << position.relative_altitude_m << "m" << endl;
    }); //Set up callback to monitor altitude

    while(telemetry -> health_all_ok()!= true){
        cout << " Vehicle is getting ready to arm " <<endl;
        this_thread::sleep_for(chrono::seconds(1));
    } //Check if vehicle is ready to arm

    // 3. Arm --------------------------------------------------------------------------------
    auto action = make_shared<Action>(system);
    cout << " Arming... " << endl;
    const Action::Result arm_result = action -> arm();

    if(arm_result != Action::Result::Success){
        cout << " Arming failed : " << arm_result << endl;
        return 1;
    }

    // 4. Take off ----------------------------------------------------------------------------------
    cout << " Taking off... " << endl;
    const Action::Result takeoff_result = action -> takeoff();

    if(takeoff_result != Action::Result::Success){
        cout << " Takeoff failed : " << takeoff_result << endl;
        return 1;
    }
    this_thread::sleep_for(chrono::seconds(20)); // controller rotate 전 키 값을 주면 기체가 뒤집어짐으로 timesleep을 일정시간 줌

    //10. rotate vehicle(1) ------------------------------------------------------------------------------------
    // auto offboard = make_shared<Offboard>(system);
    // bool ret = offboard_ctrl_body(offboard);
    // if(ret == false){return -1;}
    
    //rotate vehicle(2)
    auto offboard = std::make_shared<Offboard>(system);
    bool ret = offboard_ctrl_body(offboard); if (ret == false) { return -1; }

    //9. Landing ------------------------------------------------------------------------------------
    this_thread::sleep_for(chrono::seconds(10)); //stuck air
    cout << " Landing... " << endl;
    const Action::Result land_result = action -> land();

    if(land_result != Action::Result::Success){return 1;} //Land failed

    while(telemetry -> in_air()){ //Check if vehicle is still in air
        this_thread::sleep_for(chrono::seconds(1)); //Vehicle is landing...
        }
    
    cout << " Landed! " << endl; //Relying on auto-disarming but let's keep watching the telemetry for a bit longer
    this_thread::sleep_for(chrono::seconds(3));
    cout << " Finished... " << endl;
    
    return 0;
}

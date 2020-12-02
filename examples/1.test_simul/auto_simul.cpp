#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <future>
#include <math.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>

using namespace mavsdk;
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;

#define PI 3.141592

// rapa/Documents/PX4-Autopilot/make px4_sitl gazebo_typhoon_h480__sonoma_raceway, sonoma_raceway map에서 typhoon_h480 기체로 실행

static Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    Mission::MissionItem);

Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    Mission::MissionItem)

{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    return new_item;
}

int main(int argc, char** argv){ 
    // 1.connect_result ------------------------------------------------------------------  
    Mavsdk mavsdk;
    ConnectionResult connection_result;
    bool discovered_system = false;

    connection_result = mavsdk.add_any_connection(argv[1]);

    if(connection_result != ConnectionResult::Success){return 1;}

    mavsdk.subscribe_on_new_system([&mavsdk, &discovered_system](){
        const auto system = mavsdk.systems().at(0);
        if(system -> is_connected()){discovered_system = true;}
    });

    this_thread::sleep_for(chrono::seconds(2));

    if(!discovered_system){return 1;}

    const auto system = mavsdk.systems().at(0);

    system -> register_component_discovered_callback(
        [](ComponentType com_type) -> void{cout << unsigned(com_type);
        });
    
    // 2.regist_telemetry -------------------------------------------------------------------
    auto telemetry = make_shared<Telemetry>(system);
    const Telemetry::Result set_rate_result = telemetry -> set_rate_position(2.0);

    if(set_rate_result != Telemetry::Result::Success){return 1;}
    telemetry->subscribe_position([](Telemetry::Position position) {
        cout << " Home latitude : " << position.latitude_deg << endl; //38.1615
        cout << " Home longitude : " << position.longitude_deg << endl; //-122.455
    });

    this_thread::sleep_for(chrono::seconds(1));

    telemetry -> subscribe_position([](Telemetry::Position position){
        cout << " Altitude : " << position.relative_altitude_m << "m" << endl;
    });

    while(telemetry -> health_all_ok()!= true){
        cout << " Vehicle is getting ready to arm " <<endl;
        this_thread::sleep_for(chrono::seconds(1));
    }

    //3. fly_mission upload(1) -------------------------------------------------------------------------------------
    vector<Mission::MissionItem> mission_items;
    Mission::MissionItem mission_item;

    // mission_items.push_back(make_mission_item(
    //     38.161615,
    //     -122.455200,
    //     10.0f,
    //     30.0f,
    //     true,
    //     mission_item));

    // mission_items.push_back(make_mission_item(
    //     38.161766,
    //     -122.455117,
    //     10.0f,
    //     30.0f,
    //     true,
    //     mission_item));

    // mission_items.push_back(make_mission_item(
    //     38.161871,
    //     -122.454869,
    //     10.0f,
    //     30.0f,
    //     true,
    //     mission_item));

    // mission_items.push_back(make_mission_item(
    //     38.161741,
    //     -122.454798,
    //     10.0f,
    //     30.0f,
    //     true,
    //     mission_item));
    
    //waypoint 1
    for(double i = 0.0001; i < 0.0007; i=i+0.0001){
        mission_items.push_back(make_mission_item(
            38.161615 + i,
            -122.455200 + i,
            10.0f,
            30.0f,
            false,
            mission_item));
        this_thread::sleep_for(chrono::seconds(7)); 
     }

    //4. fly_mission upload(2) ------------------------------------------------------------------------------------
    auto mission = make_shared<Mission>(system);
    {
        auto prom = make_shared<promise<Mission::Result>>();
        auto future_result = prom->get_future();
        Mission::MissionPlan mission_plan;

        mission_plan.mission_items = mission_items;

        mission->upload_mission_async(mission_plan,
            [prom](Mission::Result result) {
                prom->set_value(result);
                });

        const Mission::Result result = future_result.get();
        
        if (result != Mission::Result::Success) { return 1; }
    }

    // 5. Arm --------------------------------------------------------------------------------
    auto action = make_shared<Action>(system);
    cout << " Arming... " << endl;
    const Action::Result arm_result = action -> arm();

    if(arm_result != Action::Result::Success){
        cout << " Arming failed : " << arm_result << endl;
        return 1;
    }

    // 6. Take off ----------------------------------------------------------------------------------
    cout << " Taking off... " << endl;
    const Action::Result takeoff_result = action -> takeoff();

    if(takeoff_result != Action::Result::Success){
        cout << " Takeoff failed : " << takeoff_result << endl;
        return 1;
    }


    //7. Mission Progress ----------------------------------------------------------------------------
    {
        cout << " Starting mission. " <<endl;
        auto start_prom = make_shared < promise < Mission::Result >> ();
        auto future_result = start_prom -> get_future();
        mission -> start_mission_async([start_prom](Mission::Result result){
            start_prom -> set_value(result);
            cout << "Started mission. " <<endl;
        });

        const Mission::Result result = future_result.get();
        if(result != Mission::Result::Success){return 1;}
    }

    while(!mission -> is_mission_finished().second){
        this_thread::sleep_for(chrono::seconds(1));
        }

    //8. Return to Home ----------------------------------------------------------------------------------
    {
        cout << " Mission Success, Return Home Point! " << endl;
        const Action::Result result = action->return_to_launch();
        if (result != Action::Result::Success){
            cout << " Returning Failed " << endl;
        }
    }

    //9. Landing ------------------------------------------------------------------------------------
    while(telemetry -> armed()){
        this_thread::sleep_for(chrono::seconds(1));
        }
    cout << " Landed! " << endl;
    cout << " Finished... " << endl;
    
    return 0;
}


#include "gesture_receiver.hpp"
#include <cstdlib>
#include <iostream>

int main(int argc,char** argv) {
  if(argc!=3 || !std::getenv("SONIC_GESTURE_FD")) return 2;
  try {
    sonic_gesture::GestureReceiver receiver(std::stoi(std::getenv("SONIC_GESTURE_FD")),
                                            argv[1],argv[2],1004);
    std::cout<<"receiver-ready"<<std::endl;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(std::chrono::steady_clock::now()<deadline) {
      auto snapshot=receiver.Snapshot();
      if(snapshot) {
        nlohmann::json frames=nlohmann::json::array();
        for(const auto& arm:snapshot->frames) {
          auto row=nlohmann::json::array();
          for(auto v:arm.q)row.push_back(v);
          for(auto v:arm.dq)row.push_back(v);
          row.push_back(arm.weight);row.push_back(arm.weight_rate);
          frames.push_back(row);
        }
        std::cout<<"result:"<<nlohmann::json({{"origin_sim_tick",snapshot->origin_sim_tick},
                                               {"frames",frames}}).dump()<<std::endl;
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 3;
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 4;}
}

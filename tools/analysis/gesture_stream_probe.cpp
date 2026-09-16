// Transport-only synthetic clock/consumer. No policy, DDS, simulator or motors.
#include "gesture_receiver.hpp"
#include <cstdlib>
#include <iostream>

int main(int argc,char** argv) {
  if(argc!=2 || !std::getenv("SONIC_GESTURE_FD"))return 2;
  try {
    const int executions=std::stoi(argv[1]);
    sonic_gesture::GestureReceiver receiver(std::stoi(std::getenv("SONIC_GESTURE_FD")),"stream-test",1000);
    const auto start=std::chrono::steady_clock::now();
    std::uint64_t consumed=0;
    unsigned samples=0,max_age_ticks=0;
    std::cout<<"receiver-ready"<<std::endl;
    while(std::chrono::steady_clock::now()-start<std::chrono::seconds(120)) {
      auto now=std::chrono::steady_clock::now();
      auto tick=1000+static_cast<std::uint32_t>(std::chrono::duration<double>(now-start).count()/.005);
      receiver.SetTick(tick);
      auto snapshot=receiver.Snapshot();
      if(snapshot) {
        if(!sonic_gesture::SnapshotCovers(*snapshot,tick,now))throw std::runtime_error("snapshot expired");
        max_age_ticks=std::max(max_age_ticks,tick-snapshot->origin_sim_tick);
        ++samples;
        bool zero=true;
        for(const auto& frame:snapshot->frames)zero=zero && frame.weight==0 && frame.weight_rate==0;
        receiver.MarkConsumed(*snapshot); // synthetic consumption only
        if(zero)consumed=snapshot->execution_id;
      } else if(consumed==static_cast<std::uint64_t>(executions)) {
        std::cout<<"result:"<<nlohmann::json({{"executions",consumed},{"samples",samples},
          {"max_snapshot_age_ticks",max_age_ticks}}).dump()<<std::endl;
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("stream timeout");
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 4;}
}

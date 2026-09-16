#include "gesture_composer.hpp"
#include <iostream>
#include <iomanip>

int main() {
  std::cout << std::setprecision(17);
  int count;
  if (!(std::cin >> count) || count < 1 || count > 10000) return 2;
  for (int n=0;n<count;++n) {
    sonic_gesture::JointReference base;
    sonic_gesture::ArmSample arm;
    for (auto& v:base.q) if (!(std::cin>>v)) return 2;
    for (auto& v:base.dq) if (!(std::cin>>v)) return 2;
    for (auto& v:arm.q) if (!(std::cin>>v)) return 2;
    for (auto& v:arm.dq) if (!(std::cin>>v)) return 2;
    if (!(std::cin>>arm.weight>>arm.weight_rate)) return 2;
    auto out=sonic_gesture::Compose(base,arm);
    for (auto v:out.q) std::cout<<v<<' ';
    for (auto v:out.dq) std::cout<<v<<' ';
    std::cout<<'\n';
  }
}

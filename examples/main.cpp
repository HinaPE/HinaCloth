#include "rphys/api_version.h"
#include "rphys/api_world.h"
#include <iostream>

int main() {
    rphys::world_desc desc{};
    auto wid = rphys::create_world(desc);
    if (wid.value == 0) {
        std::cerr << "Failed to create world\n";
        return 1;
    }
    for (int i = 0; i < 5; ++i) {
        rphys::step_world(wid, 0.016); // simulate ~16 ms
    }
    std::cout << "Frames: " << rphys::world_frame_count(wid) << " TotalTime: " << rphys::world_total_time(wid) << "\n";
    rphys::destroy_world(wid);
    return 0;
}

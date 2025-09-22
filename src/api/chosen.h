#ifndef HINACLOTH_CHOSEN_H
#define HINACLOTH_CHOSEN_H
#include "policy_in.h"

namespace sim {
    struct Chosen {
        DataLayout layout;
        Backend backend;
        int threads;
    };
}

#endif // HINACLOTH_CHOSEN_H


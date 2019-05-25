//
// Created by untouch on 19-1-16.
//

#include "judger.h"

bool Judger::Detect(bool state) {
    if(states_.size() == size_)
        states_.pop_front();
    states_.emplace_back(state);

    size_t count = 0;
    for(auto& item : states_){
        if(item) ++count;
    }

    return count >= threshold_;
}

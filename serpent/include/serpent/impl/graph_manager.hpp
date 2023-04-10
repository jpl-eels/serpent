#ifndef SERPENT_IMPL_GRAPH_MANAGER_HPP
#define SERPENT_IMPL_GRAPH_MANAGER_HPP

#include "serpent/graph_manager.hpp"

namespace serpent {

template<gtsam::Key (*KeyFunc)(std::uint64_t), typename ValueType>
void GraphManager::add(const int key_, const ValueType& value_) {
    assert(key_ >= 0);
    const gtsam::Key raw_key = KeyFunc(key_);
    values_.insert(raw_key, value_);
    new_values_.insert(raw_key, value_);
    unmarginalised_keys(key_).push_back(raw_key);
}

template<gtsam::Key (*KeyFunc)(std::uint64_t), typename ValueType>
void GraphManager::set(const int key_, const ValueType& value_) {
    assert(key_ >= 0);
    const gtsam::Key raw_key = KeyFunc(key_);
    if (values_.exists(raw_key)) {
        update(raw_key, value_);
    } else {
        add<KeyFunc>(key_, value_);
    }
}

template<typename ValueType>
void GraphManager::update(const gtsam::Key raw_key, const ValueType& value_) {
    assert(raw_key >= 0);
    values_.update(raw_key, value_);
    if (new_values_.exists(raw_key)) {
        new_values_.update(raw_key, value_);
    }
}

}

#endif

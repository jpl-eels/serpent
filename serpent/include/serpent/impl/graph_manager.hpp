#ifndef SERPENT_IMPL_GRAPH_MANAGER_HPP
#define SERPENT_IMPL_GRAPH_MANAGER_HPP

#include "serpent/graph_manager.hpp"

namespace serpent {

template<typename ValueType>
void GraphManager::add(const gtsam::Key key_, const ValueType& value_) {
    values_.insert(key_, value_);
    new_values_.insert(key_, value_);
}

template<typename ValueType>
void GraphManager::set(const gtsam::Key key_, const ValueType& value_) {
    assert(key_ >= 0);
    if (values_.exists(key_)) {
        update(key_, value_);
    } else {
        add(key_, value_);
    }
}

template<typename ValueType>
void GraphManager::update(const gtsam::Key key_, const ValueType& value_) {
    assert(key_ >= 0);
    values_.update(key_, value_);
    if (new_values_.exists(key_)) {
        new_values_.update(key_, value_);
    }
}

}

#endif

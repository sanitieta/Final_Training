//
// Created by xuhao on 2025/11/11.
//

#include "CanTxManager.h"

CanTxManager& CanTxManager::instance() {
    static CanTxManager instance;
    return instance;
}
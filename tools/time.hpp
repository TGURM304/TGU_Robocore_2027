//
// Created by Interweave on 2026/5/21.
//

/**
 * @file time.hpp
 * @brief 项目统一时间戳工具。
 */

#ifndef TGU_ROBOCORE_2027_TIME_HPP
#define TGU_ROBOCORE_2027_TIME_HPP

#pragma once

#include <chrono>
#include <cstdint>

namespace tools {

/**
 * @brief 获取单调时钟时间戳，单位 ns。
 *
 * 该时间戳来自 std::chrono::steady_clock，保证单调递增，不受系统时间校准影响。
 * 推荐用于帧时间戳、延迟统计、帧间隔 dt、预测等项目内部时间计算。
 *
 * 注意：该时间戳不是 Unix 时间戳，不能直接用于跨设备真实时间对齐。
 */
[[nodiscard]] inline uint64_t steady_time_ns() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

/**
 * @brief 获取系统时钟 Unix 时间戳，单位 ns。
 *
 * 该时间戳来自 std::chrono::system_clock，会受到系统时间调整影响。
 * 推荐用于需要和真实世界时间对应的日志、录包、外部系统对齐等场景。
 */
[[nodiscard]] inline uint64_t system_time_ns() noexcept {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

} // namespace tools

#endif // TGU_ROBOCORE_2027_TIME_HPP

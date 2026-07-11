#ifndef TIMED_SNAPSHOT_H
#define TIMED_SNAPSHOT_H

#include <stdint.h>

namespace foundation
{
    /**
     * @brief 保存带时间戳、序号和有效标志的数据快照
     *
     * @tparam value_type 快照负载类型
     */
    template<typename value_type>
    struct timed_snapshot
    {
        value_type value{};
        uint32_t timestamp_us = 0;
        uint32_t sequence = 0;
        bool valid = false;

        void set(const value_type &new_value, uint32_t new_timestamp_us,
            uint32_t new_sequence)
        {
            value = new_value;
            timestamp_us = new_timestamp_us;
            sequence = new_sequence;
            valid = true;
        }

        void invalidate()
        {
            valid = false;
        }

        bool fresh(uint32_t now_us, uint32_t timeout_us) const
        {
            return valid &&
                   (uint32_t)(now_us - timestamp_us) <= timeout_us;
        }
    };
}

#endif

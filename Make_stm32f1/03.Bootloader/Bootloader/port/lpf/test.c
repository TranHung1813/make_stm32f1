#include "lpf.h"

void test_lpf(void)
{
    lpf_data_t current =
    {
        .gain = 1,
        .estimate_value = 1603
    };
    static uint32_t m_table[6] = { 2683, 1303, 1013, 683, 2185, 2681 };
    for (uint32_t i = 0; i < sizeof(m_table) / sizeof(m_table[0]); i++)
    {
        uint32_t prev = current.estimate_value;
        current.estimate_value *= 1;
        uint32_t tmp = m_table[i]*1;
        lpf_update_estimate(&current, (int32_t*)&tmp);
        current.estimate_value /= 1;
        printf("Current %u, measure %u, estimate %u\r\n", prev, m_table[i], current.estimate_value);
    }
}

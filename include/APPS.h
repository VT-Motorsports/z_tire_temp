#pragma once
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/sys/mem_stats.h>
#include <zephyr/debug/cpu_load.h>
#include "hardware.h"
#include "vehicle_state.h"

class APPS_THREAD
{
  private:
    VehicleState *vehicle;

    static constexpr int APPS_THREAD_STACK_SIZE = 1024;
    static constexpr int APPS_THREAD_PRIORITY = 5;
    static constexpr int APPS_PERIOD_MS = 100;

    struct k_thread apps_thread_;
    k_thread_stack_t *apps_stack_;
    static VehicleState *vehicle_;
    static Hardware *hardware_;

    int64_t agreement_fault_deadline_ = 0;
    bool brake_fault_latched_ = false;

    static void apps_main_loop(void *p1, void *p2, void *p3);

    void update();
    float readPedalPercent(uint16_t raw, uint16_t low, uint16_t range, PEDAL_SLOPE_DIRECTION slope);
    bool checkOpenCircuit(uint16_t raw, uint16_t low_threshold);
    bool checkShortCircuit(uint16_t raw, uint16_t high_threshold);
    bool checkPedalAgreement(float p1_pct, float p2_pct);
    bool checkBrakeOverlap(float avg_pct);

  public:
    APPS_THREAD(VehicleState *vehicle);

    int Initialize(Hardware *hw);
};
#include "can.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>

LOG_MODULE_REGISTER(can);

// ============================================================================
// Construction
// ============================================================================

CanBus::CanBus()
    : dev_(nullptr), bitrate_(0), sample_point_(0), initialized_(false), started_(false),
      frames_rec(0), frames_sent(0)
{
}

// ============================================================================
// Init
// ============================================================================

int CanBus::init(const struct device *dev, uint32_t bitrate, uint32_t sample_point)
{
    if (!dev)
    {
        LOG_ERR("CAN device is NULL");
        return -1;
    }

    if (!device_is_ready(dev))
    {
        LOG_ERR("CAN device not ready");
        return -2;
    }

    dev_ = dev;
    bitrate_ = bitrate;
    sample_point_ = sample_point;

    LOG_INF("CAN device ready - calculating timing...");

    struct can_timing timing;
    int ret = can_calc_timing(dev_, &timing, bitrate_, sample_point_);
    if (ret != 0)
    {
        LOG_ERR("can_calc_timing() failed: %d", ret);
        return ret;
    }

    enum can_state state;
    can_get_state(dev_, &state, nullptr);
    if (state != CAN_STATE_STOPPED)
    {
        LOG_INF("Stopping CAN to configure timing");
        ret = can_stop(dev_);
        if (ret != 0)
        {
            LOG_ERR("can_stop() failed: %d", ret);
            return ret;
        }
        started_ = false;
    }

    LOG_INF("Applying timing configuration...");
    ret = can_set_timing(dev_, &timing);
    if (ret != 0)
    {
        LOG_ERR("can_set_timing() failed: %d", ret);
        return ret;
    }

    initialized_ = true;
    LOG_INF("CAN initialized (%d bps, sample point %d)", bitrate_, sample_point_);
    return 0;
}

// ============================================================================
// Start / Stop / Send
// ============================================================================

int CanBus::start()
{
    if (!initialized_)
    {
        LOG_ERR("CAN not initialized");
        return -1;
    }

    if (started_)
    {
        LOG_WRN("CAN already started");
        return 0;
    }

    LOG_INF("Starting CAN...");
    int ret = can_start(dev_);
    if (ret != 0)
    {
        LOG_ERR("can_start() failed: %d", ret);
        return ret;
    }

    started_ = true;
    LOG_INF("CAN started successfully");
    return 0;
}

int CanBus::stop()
{
    if (!initialized_)
    {
        return -1;
    }

    if (!started_)
    {
        return 0;
    }

    int ret = can_stop(dev_);
    if (ret == 0)
    {
        started_ = false;
        LOG_INF("CAN stopped");
    }
    return ret;
}

int CanBus::send(const struct can_frame *frame, k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
    if (!initialized_ || !started_)
    {
        return -1;
    }

    int ret = can_send(dev_, frame, timeout, callback, user_data);
    if (ret == 0)
    {
        frames_sent++;
    }
    return ret;
}

// ============================================================================
// Filter / State / Mode
// ============================================================================

int CanBus::add_rx_filter_msgq(struct k_msgq *msgq, const struct can_filter *filter)
{
    if (!initialized_)
    {
        return -1;
    }

    return can_add_rx_filter_msgq(dev_, msgq, filter);
}

void CanBus::remove_rx_filter(int filter_id)
{
    if (!initialized_)
    {
        return;
    }

    can_remove_rx_filter(dev_, filter_id);
}

int CanBus::get_state(enum can_state *state) const
{
    if (!initialized_ || !state)
    {
        return -1;
    }

    return can_get_state(dev_, state, nullptr);
}

int CanBus::set_mode(can_mode_t mode)
{
    if (!initialized_)
    {
        return -1;
    }

    bool was_started = started_;
    if (started_)
    {
        int ret = stop();
        if (ret != 0)
        {
            LOG_ERR("Failed to stop CAN before mode change");
            return ret;
        }
    }

    int ret = can_set_mode(dev_, mode);
    if (ret != 0)
    {
        LOG_ERR("can_set_mode failed: %d", ret);
        return ret;
    }

    if (was_started)
    {
        ret = start();
        if (ret != 0)
        {
            LOG_ERR("Failed to restart CAN after mode change");
            return ret;
        }
    }

    LOG_INF("CAN mode set to 0x%x", mode);
    return 0;
}

can_mode_t CanBus::get_mode() const
{
    if (!initialized_)
    {
        return (can_mode_t)0;
    }

    return can_get_mode(dev_);
}

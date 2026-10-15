// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.

#include <linux/module.h>
#include <linux/auxiliary_bus.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/leds.h>
#include <linux/math64.h>
#include <linux/bits.h>
#include "nh_fpga_fan.h"
#include "fbiob-auxdev.h"
#include "platform/nh_platform.h"
#include "nh_irq.h"

#define DRIVER_NAME "nh_fpga_fan"
#define DRIVER_VERSION "1.0"

/* Forward declarations */
static int nh_fpga_fan_probe(struct auxiliary_device *aux_dev,
			     const struct auxiliary_device_id *id);
static void nh_fpga_fan_remove(struct auxiliary_device *aux_dev);

/* ============================================================================
 * PWM Control Functions
 * ============================================================================ */

/* Read PWM value for a fan */
static int nh_fpga_fan_read_pwm(struct nh_fpga_fan_controller *controller,
				int channel)
{
	u32 reg_val;
	void __iomem *reg_addr;
	u32 hw_pwm;

	if (channel >= controller->platform_cfg->fan_cfg->num_fan_controllers)
		return -EINVAL;

	reg_addr = controller->base + controller->fan_pwm_offsets[channel];
	reg_val = ioread32(reg_addr);
	hw_pwm = (reg_val & controller->platform_cfg->fan_cfg->pwm_mask) >>
		 __ffs(controller->platform_cfg->fan_cfg->pwm_mask);

	dev_dbg(&controller->aux_dev->dev,
		"PWM read: channel=%d, addr=%p, reg_val=0x%08x, hw_pwm=%d\n",
		channel, reg_addr, reg_val, hw_pwm);

	return hw_pwm;
}

/* Write PWM value for a fan */
static int nh_fpga_fan_write_pwm(struct nh_fpga_fan_controller *controller,
				 int channel, long value)
{
	const struct fan_card_status_addr *cfg =
		controller->platform_cfg->fan_cfg;
	u32 pwm_shift = __ffs(cfg->pwm_mask);
	u32 max_pwm = cfg->pwm_mask >> pwm_shift;
	u32 reg_val, new_reg_val;
	void __iomem *reg_addr;
	u32 hw_pwm;

	if (channel >= cfg->num_fan_controllers || value < 0 || value > max_pwm)
		return -EINVAL;

	hw_pwm = (u32)value;

	reg_addr = controller->base + controller->fan_pwm_offsets[channel];

	mutex_lock(&controller->lock);
	reg_val = ioread32(reg_addr);
	new_reg_val = (reg_val & ~cfg->pwm_mask) |
		      ((hw_pwm << pwm_shift) & cfg->pwm_mask);
	iowrite32(new_reg_val, reg_addr);
	mutex_unlock(&controller->lock);

	dev_dbg(&controller->aux_dev->dev,
		"PWM write: channel=%d, addr=%p, old=0x%08x, new=0x%08x, hw_pwm=%d\n",
		channel, reg_addr, reg_val, new_reg_val, hw_pwm);

	/* Read back to verify */
	reg_val = ioread32(reg_addr);
	dev_dbg(&controller->aux_dev->dev,
		"PWM readback: channel=%d, reg_val=0x%08x, hw_pwm=%d\n",
		channel, reg_val, (reg_val & cfg->pwm_mask) >> pwm_shift);

	return 0;
}

/* ============================================================================
 * Fan Tachometer Functions
 * ============================================================================ */

/* Read tachometer value for a fan */
static int nh_fpga_fan_read_tach(struct nh_fpga_fan_controller *controller,
				 int channel, bool inner)
{
	u32 reg_val;
	u32 offset;

	if (channel >= controller->platform_cfg->fan_cfg->num_fan_controllers)
		return -EINVAL;

	if (inner) {
		u32 mask = controller->platform_cfg->fan_cfg->inner_tach_mask;

		offset = controller->fan_inner_tach_offsets[channel];
		reg_val = ioread32(controller->base + offset);
		return (reg_val & mask) >> __ffs(mask);
	} else {
		u32 mask = controller->platform_cfg->fan_cfg->outer_tach_mask;

		offset = controller->fan_outer_tach_offsets[channel];
		reg_val = ioread32(controller->base + offset);
		return (reg_val & mask) >> __ffs(mask);
	}
}

/* Convert a tach period reading to RPM. The FPGA counts the time over one
 * fan rotation (two tach pulses) in ticks of tach_time_unit_ns, so
 * RPM = 60e9 / (tach_time_unit_ns * tach_cycles).
 */
u32 nh_fpga_fan_tach_to_rpm(u32 tach_cycles, u32 tach_time_unit_ns)
{
	u64 rpm;

	if (tach_cycles == 0 || tach_cycles == 0xFFFF)
		return 0; /* Fan stopped */

	if (!tach_time_unit_ns)
		tach_time_unit_ns = FAN_TACH_TIME_UNIT_NS;

	rpm = div64_u64(60000000000ULL,
			(u64)tach_cycles * tach_time_unit_ns);

	return (u32)rpm;
}

/* ============================================================================
 * Fan Status Functions
 * ============================================================================ */

/**
 * Check if a fan is present and accessible
 * @fan: Fan controller instance
 * @channel: Fan channel (0-based)
 * @dev: Device for debug logging (can be NULL)
 * @operation: Operation name for debug message (e.g., "PWM read", "LED control")
 *
 * Returns: 0 if fan is present, negative error code otherwise
 */
static int
nh_fpga_fan_get_status_and_presence(struct nh_fpga_fan_controller *controller,
				    int channel, struct device *dev,
				    const char *operation)
{
	struct fan_status status;
	int ret;

	ret = nh_fpga_fan_get_status(controller, channel, &status);
	if (ret)
		return ret;

	if (!status.present) {
		if (dev && operation)
			dev_dbg(dev, "%s failed: Fan %d not present\n",
				operation, channel);
		return -ENODEV;
	}

	return 0;
}

/* Get fan status information */
int nh_fpga_fan_get_status(struct nh_fpga_fan_controller *controller,
			   int channel, struct fan_status *status)
{
	u32 reg_val;
	const struct fan *fan;

	if (channel >= controller->platform_cfg->fan_cfg->num_fan_controllers ||
	    !status)
		return -EINVAL;

	fan = &controller->platform_cfg->fan_cfg->fans[channel];
	reg_val = ioread32(controller->base + fan->status_offset);

	/* Extract status information.
	 * Presence is driven by the fan module ID nibble: only the known
	 * IDs are treated as present and avoids treating
	 * reset/default/unknown values as present.
	 */
	status->module_id = (reg_val & fan->module_id_mask) >>
			    fan->module_id_shift;
	status->present = (status->module_id == FAN_MODULE_ID_DUAL_ROTOR);
	status->power_good = !!(reg_val & fan->pwrgood);
	status->inner_tach_ok = !!(reg_val & fan->inner_tach);
	status->outer_tach_ok = !!(reg_val & fan->outer_tach);

	dev_dbg(&controller->aux_dev->dev,
		"Fan %d status: present=%d, pg=%d, module_id=0x%x, inner_tach=%d, outer_tach=%d\n",
		channel, status->present, status->power_good, status->module_id,
		status->inner_tach_ok, status->outer_tach_ok);

	return 0;
}

/* Get fan card type */
int nh_fpga_fan_get_card_type(struct nh_fpga_fan_controller *controller)
{
	u32 reg_val;

	/* Read status register */
	reg_val = ioread32(
		controller->base +
		controller->platform_cfg->fan_cfg->fans[0].status_offset);

	/* Extract card type */
	return (reg_val & controller->platform_cfg->fan_cfg->card_type_mask);
}

/* ============================================================================
 * hwmon Interface Functions
 * ============================================================================ */

/* hwmon is_visible function */
static umode_t nh_fpga_fan_is_visible(const void *data,
				      enum hwmon_sensor_types type, u32 attr,
				      int channel)
{
	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_label:
			return 0444;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return 0;
}

/* hwmon read function */
static int nh_fpga_fan_read(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long *val)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);
	u32 tach_cycles;
	int fan_channel;
	int ret;

	switch (type) {
	case hwmon_pwm:
		if (attr == hwmon_pwm_input) {
			/* Check if fan is present before reading PWM */
			ret = nh_fpga_fan_get_status_and_presence(
				controller, channel, dev, "PWM read");
			if (ret)
				return ret;
			*val = nh_fpga_fan_read_pwm(controller, channel);
			return 0;
		}
		break;
	case hwmon_fan:
		if (attr == hwmon_fan_input) {
			const struct fan_card_status_addr *fan_cfg =
				controller->platform_cfg->fan_cfg;
			bool inner;

			/* A tray's rotors occupy consecutive channels, inner
			 * first, so a single-tach tray is just one channel.
			 */
			fan_channel = channel / fan_cfg->num_rotors;
			inner = (channel % fan_cfg->num_rotors) == 0;
			/* Check if fan is present before reading tachometer */
			ret = nh_fpga_fan_get_status_and_presence(
				controller, fan_channel, dev, "Fan speed read");
			if (ret)
				return ret;
			tach_cycles = nh_fpga_fan_read_tach(controller,
							    fan_channel, inner);
			*val = nh_fpga_fan_tach_to_rpm(tach_cycles,
						       fan_cfg->tach_time_unit_ns);
			return 0;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

/* hwmon write function */
static int nh_fpga_fan_write(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long val)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_pwm:
		if (attr == hwmon_pwm_input) {
			/* Check if fan is present before writing PWM */
			ret = nh_fpga_fan_get_status_and_presence(
				controller, channel, dev, "PWM write");
			if (ret)
				return ret;
			return nh_fpga_fan_write_pwm(controller, channel, val);
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

/* hwmon read_string function, serving fanN_label.
 *
 * Unlike the tachometer read this deliberately does not check presence: a label
 * is static identity, not a measurement, and lm_sensors reads labels for absent
 * fans too.
 */
static int nh_fpga_fan_read_string(struct device *dev,
				   enum hwmon_sensor_types type, u32 attr,
				   int channel, const char **str)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);

	if (type != hwmon_fan || attr != hwmon_fan_label)
		return -EOPNOTSUPP;

	*str = controller->fan_labels[channel];
	return 0;
}

static const struct hwmon_ops nh_fpga_fan_hwmon_ops = {
	.is_visible = nh_fpga_fan_is_visible,
	.read = nh_fpga_fan_read,
	.read_string = nh_fpga_fan_read_string,
	.write = nh_fpga_fan_write,
};

/* Number of hwmon fan channels this platform exposes: one per tachometer. */
static u32 nh_fpga_fan_num_tach(const struct fan_card_status_addr *fan_cfg)
{
	return fan_cfg->num_fan_controllers * fan_cfg->num_rotors;
}

/* Build a hwmon_chip_info that advertises one pwm channel per fan tray and
 * one fan-input channel per tachometer. Memory is devm-managed against @dev
 * so it lives as long as the hwmon device.
 */
static const struct hwmon_chip_info *
nh_fpga_fan_build_chip_info(struct device *dev,
			    const struct fan_card_status_addr *fan_cfg)
{
	u32 num_fans = fan_cfg->num_fan_controllers;
	u32 num_tach = nh_fpga_fan_num_tach(fan_cfg);
	struct hwmon_chip_info *chip;
	struct hwmon_channel_info *pwm_info, *fan_info;
	const struct hwmon_channel_info **info_arr;
	u32 *pwm_config, *fan_config;
	u32 i;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	pwm_info = devm_kzalloc(dev, sizeof(*pwm_info), GFP_KERNEL);
	fan_info = devm_kzalloc(dev, sizeof(*fan_info), GFP_KERNEL);
	info_arr = devm_kcalloc(dev, 3, sizeof(*info_arr), GFP_KERNEL);
	pwm_config = devm_kcalloc(dev, num_fans + 1, sizeof(*pwm_config),
				  GFP_KERNEL);
	fan_config = devm_kcalloc(dev, num_tach + 1, sizeof(*fan_config),
				  GFP_KERNEL);
	if (!chip || !pwm_info || !fan_info || !info_arr || !pwm_config ||
	    !fan_config)
		return NULL;

	for (i = 0; i < num_fans; i++)
		pwm_config[i] = HWMON_PWM_INPUT;
	for (i = 0; i < num_tach; i++)
		fan_config[i] = HWMON_F_INPUT | HWMON_F_LABEL;

	pwm_info->type = hwmon_pwm;
	pwm_info->config = pwm_config;
	fan_info->type = hwmon_fan;
	fan_info->config = fan_config;

	info_arr[0] = pwm_info;
	info_arr[1] = fan_info;
	info_arr[2] = NULL;

	chip->ops = &nh_fpga_fan_hwmon_ops;
	chip->info = info_arr;
	return chip;
}

/* Build the fanN_label strings, one per hwmon fan channel.
 *
 * Raw channel numbers say nothing about which physical rotor they read: the
 * labels are what let a technician map a failing channel to a fan tray. The
 * channel-to-tray mapping here must match the read path, so both derive it
 * from num_rotors: dual-tach fans get fan<tray>_inner / fan<tray>_outer,
 * single-tach fans just fan<tray>.
 *
 * read_string hands out a borrowed pointer, so the strings are formatted once
 * here rather than per read. Memory is devm-managed against @dev.
 */
static const char **
nh_fpga_fan_build_labels(struct device *dev,
			 const struct fan_card_status_addr *fan_cfg)
{
	u32 num_tach = nh_fpga_fan_num_tach(fan_cfg);
	const char **labels;
	u32 i;

	labels = devm_kcalloc(dev, num_tach, sizeof(*labels), GFP_KERNEL);
	if (!labels)
		return NULL;

	for (i = 0; i < num_tach; i++) {
		u32 tray = i / fan_cfg->num_rotors + 1;

		if (fan_cfg->num_rotors > 1)
			labels[i] = devm_kasprintf(
				dev, GFP_KERNEL, "fan%u_%s", tray,
				(i % fan_cfg->num_rotors) == 0 ? "inner" :
								 "outer");
		else
			labels[i] =
				devm_kasprintf(dev, GFP_KERNEL, "fan%u", tray);
		if (!labels[i])
			return NULL;
	}

	return labels;
}

/* ============================================================================
 * LED Control Functions
 * ============================================================================ */

/* LED brightness set function */
static int nh_fpga_fan_led_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct nh_fpga_fan_led *fan_led =
		container_of(led_cdev, struct nh_fpga_fan_led, led_cdev);
	struct nh_fpga_fan_controller *controller = fan_led->fan_ctrl;
	const struct fan *fan;
	u32 reg_val, new_reg_val;
	u32 bit_pos, sibling_bit;
	bool led_on = (brightness != LED_OFF);
	int ret;

	/* Check if fan is present before controlling LED */
	ret = nh_fpga_fan_get_status_and_presence(controller,
						  fan_led->fan_index,
						  &controller->aux_dev->dev,
						  "LED control");
	if (ret)
		return ret;

	fan = &controller->platform_cfg->fan_cfg->fans[fan_led->fan_index];
	bit_pos = fan_led->is_good_led ? fan->ctrl_good_led_bit :
					 fan->ctrl_fail_led_bit;
	/* The sibling is whichever of the two color bits isn't bit_pos. */
	sibling_bit = (fan->ctrl_good_led_bit | fan->ctrl_fail_led_bit) &
		      ~bit_pos;

	mutex_lock(&controller->lock);

	/* Read current control register */
	reg_val = ioread32(controller->base +
			   controller->platform_cfg->fan_cfg->ctrl_offset);
	new_reg_val = reg_val;

	/* The good/fail bits drive one physical bi-color LED. fan_service only
	 * ever turns a color on and never clears the other, and the FPGA powers
	 * up with the fail bit set, so enforce mutual exclusion here: asserting
	 * one color clears its sibling. brightness_get reads HW, so no cached
	 * brightness sync is needed for the sibling class device.
	 */
	if (led_on) {
		new_reg_val |= bit_pos;
		new_reg_val &= ~sibling_bit;
	} else {
		new_reg_val &= ~bit_pos;
	}

	/* Write updated value */
	iowrite32(new_reg_val,
		  controller->base +
			  controller->platform_cfg->fan_cfg->ctrl_offset);

	mutex_unlock(&controller->lock);

	dev_dbg(&controller->aux_dev->dev,
		"LED %s: fan=%d, %s=%s, reg=0x%08x->0x%08x\n", led_cdev->name,
		fan_led->fan_index + 1, fan_led->is_good_led ? "good" : "fail",
		led_on ? "on" : "off", reg_val, new_reg_val);

	return 0;
}

/* LED brightness get function */
static enum led_brightness nh_fpga_fan_led_get(struct led_classdev *led_cdev)
{
	struct nh_fpga_fan_led *fan_led =
		container_of(led_cdev, struct nh_fpga_fan_led, led_cdev);
	struct nh_fpga_fan_controller *controller = fan_led->fan_ctrl;
	const struct fan *fan;
	u32 reg_val;
	u32 bit_pos;
	bool led_on;
	int ret;

	/* Check if fan is present before reading LED state */
	ret = nh_fpga_fan_get_status_and_presence(controller,
						  fan_led->fan_index,
						  &controller->aux_dev->dev,
						  "LED read");
	if (ret) {
		return LED_OFF; /* Return OFF for missing fans or errors */
	}

	fan = &controller->platform_cfg->fan_cfg->fans[fan_led->fan_index];
	bit_pos = fan_led->is_good_led ? fan->ctrl_good_led_bit :
					 fan->ctrl_fail_led_bit;

	/* Read current control register */
	reg_val = ioread32(controller->base +
			   controller->platform_cfg->fan_cfg->ctrl_offset);
	led_on = !!(reg_val & bit_pos);

	return led_on ? LED_FULL : LED_OFF;
}

/* Clear all fan LED bits in the Fan Card Control register.
 *
 * The FPGA powers up with every fail bit set ("Default On" per the HW spec),
 * so an unprogrammed, healthy chassis shows a spurious fault color. Clear both
 * bits for every tray at probe so the LEDs stay off until fan_service drives
 * them to their good/fail color.
 */
static void nh_fpga_fan_init_leds(struct nh_fpga_fan_controller *controller)
{
	u32 offset = controller->platform_cfg->fan_cfg->ctrl_offset;
	u32 reg_val;
	int i;

	mutex_lock(&controller->lock);
	reg_val = ioread32(controller->base + offset);
	for (i = 0; i < controller->num_fan_trays; i++) {
		const struct fan *fan =
			&controller->platform_cfg->fan_cfg->fans[i];
		reg_val &= ~fan->ctrl_good_led_bit;
		reg_val &= ~fan->ctrl_fail_led_bit;
	}
	iowrite32(reg_val, controller->base + offset);
	mutex_unlock(&controller->lock);
}

/* Register LED class devices for all fans */
static int nh_fpga_fan_register_leds(struct nh_fpga_fan_controller *controller)
{
	int i, ret;
	char led_name[64];

	for (i = 0; i < controller->num_fan_trays; i++) {
		/* Register good (blue) LED */
		controller->good_leds[i].fan_ctrl = controller;
		controller->good_leds[i].fan_index = i;
		controller->good_leds[i].is_good_led = true;

		/* good = blue, fail = amber: fan_service and its ConfigValidator
		 * expect these class-device colors.
		 */
		snprintf(led_name, sizeof(led_name), "fan%d_led:blue:status",
			 i + 1);
		controller->good_leds[i].led_cdev.name = devm_kstrdup(
			&controller->aux_dev->dev, led_name, GFP_KERNEL);
		if (!controller->good_leds[i].led_cdev.name)
			return -ENOMEM;

		controller->good_leds[i].led_cdev.brightness_set_blocking =
			nh_fpga_fan_led_set;
		controller->good_leds[i].led_cdev.brightness_get =
			nh_fpga_fan_led_get;
		controller->good_leds[i].led_cdev.max_brightness = LED_FULL;
		controller->good_leds[i].led_cdev.brightness = LED_OFF;

		ret = devm_led_classdev_register(
			&controller->aux_dev->dev,
			&controller->good_leds[i].led_cdev);
		if (ret) {
			dev_err(&controller->aux_dev->dev,
				"Failed to register good LED for fan %d: %d\n",
				i + 1, ret);
			return ret;
		}

		/* Register fail (amber) LED */
		controller->fail_leds[i].fan_ctrl = controller;
		controller->fail_leds[i].fan_index = i;
		controller->fail_leds[i].is_good_led = false;

		snprintf(led_name, sizeof(led_name), "fan%d_led:amber:status",
			 i + 1);
		controller->fail_leds[i].led_cdev.name = devm_kstrdup(
			&controller->aux_dev->dev, led_name, GFP_KERNEL);
		if (!controller->fail_leds[i].led_cdev.name)
			return -ENOMEM;

		controller->fail_leds[i].led_cdev.brightness_set_blocking =
			nh_fpga_fan_led_set;
		controller->fail_leds[i].led_cdev.brightness_get =
			nh_fpga_fan_led_get;
		controller->fail_leds[i].led_cdev.max_brightness = LED_FULL;
		/* Bits are cleared to OFF at probe (see nh_fpga_fan_init_leds),
		 * overriding the FPGA power-up default, so start the cache OFF
		 * too. brightness_get reads HW, so this is just the cache.
		 */
		controller->fail_leds[i].led_cdev.brightness = LED_OFF;

		ret = devm_led_classdev_register(
			&controller->aux_dev->dev,
			&controller->fail_leds[i].led_cdev);
		if (ret) {
			dev_err(&controller->aux_dev->dev,
				"Failed to register fail LED for fan %d: %d\n",
				i + 1, ret);
			return ret;
		}
	}

	dev_info(&controller->aux_dev->dev, "Registered %d fan LED pairs\n",
		 controller->num_fan_trays);
	return 0;
}

/* ============================================================================
 * Sysfs Interface Functions
 * ============================================================================ */

/* Fan status sysfs show function */
static ssize_t fan_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);
	struct fan_status status;
	int fan_num;
	int ret;

	/* Extract fan number from attribute name */
	if (sscanf(attr->attr.name, "fan%d_status", &fan_num) != 1 ||
	    fan_num < 1)
		return -EINVAL;

	/* Bound by actual fans on this board */
	if (fan_num > controller->num_fan_trays)
		return -EINVAL;

	fan_num--; /* Convert to 0-based index */

	ret = nh_fpga_fan_get_status(controller, fan_num, &status);
	if (ret)
		return ret;

	return sprintf(
		buf,
		"present:%d power_good:%d module_id:0x%x inner_tach:%d outer_tach:%d\n",
		status.present, status.power_good, status.module_id,
		status.inner_tach_ok, status.outer_tach_ok);
}

/* Fan card type sysfs show function */
static ssize_t fan_card_type_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);
	int card_type;
	const char *type_str;

	card_type = nh_fpga_fan_get_card_type(controller);

	switch (card_type) {
	case FAN_CARD_TYPE_INVALID:
		type_str = "invalid";
		break;
	case FAN_CARD_TYPE_RAPTOR_2RU_12V:
		type_str = "raptor_2ru_12v";
		break;
	default:
		type_str = "unknown";
		break;
	}

	return sysfs_emit(buf, "%s (0x%x)\n", type_str, card_type);
}

/* Simple fan presence detection for FBOSS fan service */
static ssize_t fan_present_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);
	struct fan_status status;
	int fan_num;
	int ret;

	/* Extract fan number from attribute name */
	if (sscanf(attr->attr.name, "fan%d_present", &fan_num) != 1 ||
	    fan_num < 1)
		return -EINVAL;

	if (fan_num > controller->num_fan_trays)
		return -EINVAL;

	fan_num--; /* Convert to 0-based index */

	ret = nh_fpga_fan_get_status(controller, fan_num, &status);
	if (ret)
		return ret;

	/* Return 1 for present, 0 for missing (FBOSS convention) */
	return sysfs_emit(buf, "%d\n", status.present ? 1 : 0);
}

/* Simple LED control for FBOSS fan service */
static ssize_t fan_led_simple_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);
	const struct fan *fan;
	u32 reg_val;
	int fan_num;
	bool good_color, fail_color;
	int ret;

	/* Extract fan number from attribute name */
	if (sscanf(attr->attr.name, "fan%d_led", &fan_num) != 1 || fan_num < 1)
		return -EINVAL;

	if (fan_num > controller->num_fan_trays)
		return -EINVAL;

	fan_num--; /* Convert to 0-based index */

	/* Check if fan is present before reading LED state */
	ret = nh_fpga_fan_get_status_and_presence(controller, fan_num, dev,
						  "LED read");
	if (ret)
		return ret;

	fan = &controller->platform_cfg->fan_cfg->fans[fan_num];
	reg_val = ioread32(controller->base +
			   controller->platform_cfg->fan_cfg->ctrl_offset);
	good_color = !!(reg_val & fan->ctrl_good_led_bit);
	fail_color = !!(reg_val & fan->ctrl_fail_led_bit);

	/* FBOSS convention: 1 = good, 0 = fail, 2 = off */
	if (good_color && !fail_color) {
		return sysfs_emit(buf, "1\n"); /* Good */
	} else if (fail_color && !good_color) {
		return sysfs_emit(buf, "0\n"); /* Fail */
	} else {
		return sysfs_emit(buf, "2\n"); /* Off or both */
	}
}

static ssize_t fan_led_simple_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nh_fpga_fan_controller *controller = dev_get_drvdata(dev);
	const struct fan *fan;
	int fan_num, led_val;
	bool good_color = false, fail_color = false;
	u32 reg_val, new_reg_val;
	u32 good_color_bit, fail_color_bit;
	int ret;

	/* Extract fan number from attribute name */
	if (sscanf(attr->attr.name, "fan%d_led", &fan_num) != 1 || fan_num < 1)
		return -EINVAL;

	if (fan_num > controller->num_fan_trays)
		return -EINVAL;

	if (kstrtoint(buf, 10, &led_val))
		return -EINVAL;

	fan_num--; /* Convert to 0-based index */

	/* Check if fan is present before controlling LED */
	ret = nh_fpga_fan_get_status_and_presence(controller, fan_num, dev,
						  "LED write");
	if (ret)
		return ret;

	/* Convert FBOSS values to LED states
	 * FBOSS standard: 1 = good, 0 = fail, 2 = off */
	switch (led_val) {
	case 1: /* Good */
		good_color = true;
		fail_color = false;
		break;
	case 0: /* Fail */
		good_color = false;
		fail_color = true;
		break;
	case 2: /* Off */
	default:
		good_color = false;
		fail_color = false;
		break;
	}

	/* Calculate bit positions */
	fan = &controller->platform_cfg->fan_cfg->fans[fan_num];
	good_color_bit = fan->ctrl_good_led_bit;
	fail_color_bit = fan->ctrl_fail_led_bit;

	mutex_lock(&controller->lock);

	/* Update hardware register */
	reg_val = ioread32(controller->base +
			   controller->platform_cfg->fan_cfg->ctrl_offset);
	new_reg_val = reg_val;

	if (good_color)
		new_reg_val |= good_color_bit;
	else
		new_reg_val &= ~good_color_bit;

	if (fail_color)
		new_reg_val |= fail_color_bit;
	else
		new_reg_val &= ~fail_color_bit;

	iowrite32(new_reg_val,
		  controller->base +
			  controller->platform_cfg->fan_cfg->ctrl_offset);

	mutex_unlock(&controller->lock);

	/* Also update LED class devices to keep them in sync */
	led_set_brightness(&controller->good_leds[fan_num].led_cdev,
			   good_color ? LED_FULL : LED_OFF);
	led_set_brightness(&controller->fail_leds[fan_num].led_cdev,
			   fail_color ? LED_FULL : LED_OFF);

	dev_dbg(&controller->aux_dev->dev,
		"FBOSS LED control: fan=%d, value=%d, good=%d, fail=%d\n",
		fan_num + 1, led_val, good_color, fail_color);

	return count;
}

/* Global (non per-fan) attribute */
static DEVICE_ATTR(fan_card_type, 0444, fan_card_type_show, NULL);

/* Per-fan sysfs attributes are built dynamically in probe so the driver
 * supports any number of fans. Each fan exposes three attrs: <name>_present,
 * <name>_led, <name>_status. The show/store handlers parse the fan number
 * from attr->attr.name via sscanf, so the names just need to match the
 * "fan%d_..." pattern.
 */
struct nh_fpga_fan_attr_template {
	const char *suffix;
	umode_t mode;
	ssize_t (*show)(struct device *, struct device_attribute *, char *);
	ssize_t (*store)(struct device *, struct device_attribute *,
			 const char *, size_t);
};

static const struct nh_fpga_fan_attr_template nh_fpga_fan_per_fan_attrs[] = {
	{ "_present", 0444, fan_present_show, NULL },
	{ "_led", 0644, fan_led_simple_show, fan_led_simple_store },
	{ "_status", 0444, fan_status_show, NULL },
};
#define NH_FPGA_FAN_ATTRS_PER_FAN ARRAY_SIZE(nh_fpga_fan_per_fan_attrs)

/* Build a sysfs attribute_group covering @num_fans fans plus the
 * platform-wide fan_card_type attribute. All memory is devm-managed.
 */
static const struct attribute_group *
nh_fpga_fan_build_attr_group(struct device *dev, u32 num_fans)
{
	struct attribute_group *group;
	struct attribute **attrs;
	struct device_attribute *fan_attrs;
	u32 total_attrs;
	u32 i, j;

	total_attrs = num_fans * NH_FPGA_FAN_ATTRS_PER_FAN +
		      1; /* +1 for fan_card_type */

	group = devm_kzalloc(dev, sizeof(*group), GFP_KERNEL);
	attrs = devm_kcalloc(dev, total_attrs + 1, sizeof(*attrs), GFP_KERNEL);
	fan_attrs = devm_kcalloc(dev, num_fans * NH_FPGA_FAN_ATTRS_PER_FAN,
				 sizeof(*fan_attrs), GFP_KERNEL);
	if (!group || !attrs || !fan_attrs)
		return NULL;

	for (i = 0; i < num_fans; i++) {
		for (j = 0; j < NH_FPGA_FAN_ATTRS_PER_FAN; j++) {
			const struct nh_fpga_fan_attr_template *t =
				&nh_fpga_fan_per_fan_attrs[j];
			struct device_attribute *attr =
				&fan_attrs[i * NH_FPGA_FAN_ATTRS_PER_FAN + j];

			attr->attr.name = devm_kasprintf(
				dev, GFP_KERNEL, "fan%u%s", i + 1, t->suffix);
			if (!attr->attr.name)
				return NULL;
			attr->attr.mode = t->mode;
			attr->show = t->show;
			attr->store = t->store;
			sysfs_attr_init(&attr->attr);
			attrs[i * NH_FPGA_FAN_ATTRS_PER_FAN + j] = &attr->attr;
		}
	}
	attrs[num_fans * NH_FPGA_FAN_ATTRS_PER_FAN] =
		&dev_attr_fan_card_type.attr;

	group->attrs = attrs;
	return group;
}

/* ============================================================================
 * Driver Infrastructure
 * ============================================================================ */

/* Auxiliary device probe function */
static int nh_fpga_fan_probe(struct auxiliary_device *aux_dev,
			     const struct auxiliary_device_id *id)
{
	struct fbiob_aux_adapter *adap =
		container_of(aux_dev, struct fbiob_aux_adapter, auxdev);
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	struct nh_fpga_fan_controller *controller;
	const struct nh_platform_cfg *platform_cfg;
	u16 device_id;
	int i, ret;
	struct device *dev = &aux_dev->dev;

	device_id = pdev->device;

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg || !platform_cfg->fan_cfg) {
		dev_err(dev, "No fan config for device ID: 0x%04x\n",
			device_id);
		return -ENODEV;
	}

	/* The channel-to-tray mapping divides by num_rotors. */
	if (!platform_cfg->fan_cfg->num_rotors) {
		dev_err(dev, "Platform table declares no fan rotors\n");
		return -EINVAL;
	}

	/* Allocate fan controller structure */
	controller =
		devm_kzalloc(&aux_dev->dev, sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;

	controller->platform_cfg = platform_cfg;

	controller->fan_pwm_offsets = devm_kmalloc_array(
		dev, controller->platform_cfg->fan_cfg->num_fan_controllers,
		sizeof(u32), GFP_KERNEL);
	controller->fan_inner_tach_offsets = devm_kmalloc_array(
		dev, controller->platform_cfg->fan_cfg->num_fan_controllers,
		sizeof(u32), GFP_KERNEL);
	controller->fan_outer_tach_offsets = devm_kmalloc_array(
		dev, controller->platform_cfg->fan_cfg->num_fan_controllers,
		sizeof(u32), GFP_KERNEL);

	if (!controller->fan_pwm_offsets ||
	    !controller->fan_inner_tach_offsets ||
	    !controller->fan_outer_tach_offsets)
		return -ENOMEM;

	/* Populate offset arrays */
	for (i = 0; i < controller->platform_cfg->fan_cfg->num_fan_controllers;
	     i++) {
		const struct fan *fan =
			&controller->platform_cfg->fan_cfg->fans[i];
		controller->fan_pwm_offsets[i] = fan->pwm_offset;
		controller->fan_inner_tach_offsets[i] = fan->inner_tach_offset;
		controller->fan_outer_tach_offsets[i] = fan->outer_tach_offset;
	}

	controller->good_leds = devm_kcalloc(
		dev, controller->platform_cfg->fan_cfg->num_fan_controllers,
		sizeof(struct nh_fpga_fan_led), GFP_KERNEL);
	controller->fail_leds = devm_kcalloc(
		dev, controller->platform_cfg->fan_cfg->num_fan_controllers,
		sizeof(struct nh_fpga_fan_led), GFP_KERNEL);

	if (!controller->good_leds || !controller->fail_leds)
		return -ENOMEM;

	controller->aux_dev = aux_dev;
	controller->base = nh_irq_mmio_base(pdev);
	if (!controller->base)
		return -ENODEV;
	controller->num_fan_trays = adap->data.fan_data.num_fans;

	/* num_fans comes from userspace via the FBIOB_IOC_NEW_DEVICE ioctl.
	 * The LED arrays above and the per fan attribute loops are sized by
	 * the platform table, so reject counts the table cannot back. */
	if (controller->num_fan_trays == 0 ||
	    controller->num_fan_trays >
		    platform_cfg->fan_cfg->num_fan_controllers) {
		dev_err(&aux_dev->dev,
			"Invalid fan count %d (platform supports %u)\n",
			controller->num_fan_trays,
			platform_cfg->fan_cfg->num_fan_controllers);
		return -EINVAL;
	}

	/* Initialize mutex */
	mutex_init(&controller->lock);

	dev_info(
		&aux_dev->dev,
		"Initializing Fan Controller with %d fan trays, base=%p, aux_offset=0x%x\n",
		controller->num_fan_trays, controller->base,
		adap->data.csr_offset);

	/* Build the per-fan sysfs attribute group dynamically and hand it to
	 * hwmon as an extra group so the attributes exist before the device
	 * is announced to udev and are removed automatically on unbind. */
	controller->attr_group = nh_fpga_fan_build_attr_group(
		&aux_dev->dev, platform_cfg->fan_cfg->num_fan_controllers);
	if (!controller->attr_group)
		return -ENOMEM;

	controller->fan_labels =
		nh_fpga_fan_build_labels(&aux_dev->dev, platform_cfg->fan_cfg);
	if (!controller->fan_labels)
		return -ENOMEM;

	/* Register hwmon device with a chip_info sized to this platform's
	 * fan count rather than a static MAX-sized array.
	 */
	{
		const struct hwmon_chip_info *chip_info =
			nh_fpga_fan_build_chip_info(&aux_dev->dev,
						    platform_cfg->fan_cfg);
		const struct attribute_group **groups;

		if (!chip_info)
			return -ENOMEM;

		groups = devm_kcalloc(&aux_dev->dev, 2, sizeof(*groups),
				      GFP_KERNEL);
		if (!groups)
			return -ENOMEM;
		groups[0] = controller->attr_group;

		controller->hwmon_dev = devm_hwmon_device_register_with_info(
			&aux_dev->dev, "nh_fpga_fan", controller, chip_info,
			groups);
	}
	if (IS_ERR(controller->hwmon_dev)) {
		ret = PTR_ERR(controller->hwmon_dev);
		dev_err(&aux_dev->dev, "Failed to register hwmon device: %d\n",
			ret);
		return ret;
	}

	/* Clear the power-up default before exposing the LEDs. */
	nh_fpga_fan_init_leds(controller);

	/* Register LED class devices */
	ret = nh_fpga_fan_register_leds(controller);
	if (ret) {
		dev_err(&aux_dev->dev, "Failed to register LED devices: %d\n",
			ret);
		return ret;
	}

	/* Store fan controller in auxiliary device data */
	auxiliary_set_drvdata(aux_dev, controller);

	dev_info(
		&aux_dev->dev,
		"FPGA Fan controller registered as hwmon device with LED class devices\n");

	return 0;
}

/* Auxiliary device remove function */
static void nh_fpga_fan_remove(struct auxiliary_device *aux_dev)
{
	struct nh_fpga_fan_controller *controller =
		auxiliary_get_drvdata(aux_dev);

	if (controller) {
		/* Clear auxiliary device data to prevent further access */
		auxiliary_set_drvdata(aux_dev, NULL);
	}

	/* With devm_hwmon_device_register_with_info(), the device is automatically removed */
	dev_info(&aux_dev->dev, "FPGA Fan controller removed\n");
}

/* Auxiliary device ID table */
static const struct auxiliary_device_id nh_fpga_fan_aux_id_table[] = {
	{
		.name = FBOSS_IOB_PCI_DRIVER".fan_ctrl",
	},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_fpga_fan_aux_id_table);

/* Auxiliary driver structure */
static struct auxiliary_driver nh_fpga_fan_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_fpga_fan_probe,
	.remove = nh_fpga_fan_remove,
	.id_table = nh_fpga_fan_aux_id_table,
};

/* ============================================================================
 * Module Infrastructure
 * ============================================================================ */

/* Module initialization */
static int __init nh_fpga_fan_init_module(void)
{
	int ret;

	pr_info("NH FPGA Fan driver v%s loading\n", DRIVER_VERSION);

	ret = auxiliary_driver_register(&nh_fpga_fan_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		return ret;
	}

	pr_info("NH FPGA Fan driver loaded successfully\n");
	return 0;
}

/* Module cleanup */
static void __exit nh_fpga_fan_exit_module(void)
{
	pr_info("NH FPGA Fan driver unloading\n");
	auxiliary_driver_unregister(&nh_fpga_fan_aux_driver);
	pr_info("NH FPGA Fan driver unloaded\n");
}

module_init(nh_fpga_fan_init_module);
module_exit(nh_fpga_fan_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Arif Mohammad <marif@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA Fan Controller Driver");
MODULE_VERSION(DRIVER_VERSION);

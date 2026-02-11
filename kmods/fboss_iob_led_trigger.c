// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/limits.h>
#include <linux/leds.h>

#include "fboss_iob_led_trigger.h"

#ifndef CONFIG_LEDS_TRIGGERS
static ssize_t trigger_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t size)
{
	struct led_classdev *led_cdev =
			((struct led_classdev *)dev_get_drvdata(dev));
	char *nl;

	nl = strchr(buf, '\n');
	if (nl)
		*nl = '\0';

	if ((strcmp(buf, "timer") == 0) ||
	    (strcmp(buf, "bp4f_timer") == 0)) {
		dev_info(led_cdev->dev, "ledtrigger (%s) enabled", buf);
		led_blink_set(led_cdev, &led_cdev->blink_delay_on,
				&led_cdev->blink_delay_off);
	}
	return size;
}

static ssize_t trigger_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	return 0;
}

static ssize_t delay_on_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t size)
{

	struct led_classdev *led_cdev =
			((struct led_classdev *)dev_get_drvdata(dev));
	int res;
	long value;

	res = kstrtol(buf, 10, &value);
	if (res < 0)
		return res;

	led_cdev->blink_delay_on = value;
	return size;
}

static ssize_t delay_on_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct led_classdev *led_cdev =
			((struct led_classdev *)dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", led_cdev->blink_delay_on);
}

static ssize_t delay_off_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t size)
{
	struct led_classdev *led_cdev =
			((struct led_classdev *)dev_get_drvdata(dev));
	int res;
	long value;

	res = kstrtol(buf, 10, &value);
	if (res < 0)
		return res;

	led_cdev->blink_delay_off = value;
	return size;
}

static ssize_t delay_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = ((struct led_classdev *)dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", led_cdev->blink_delay_off);
}

static DEVICE_ATTR_RW(delay_on);
static DEVICE_ATTR_RW(delay_off);
static DEVICE_ATTR_RW(trigger);
static struct attribute *timer_trig_attrs[] = { &dev_attr_delay_on.attr,
						&dev_attr_delay_off.attr,
						&dev_attr_trigger.attr, NULL };
ATTRIBUTE_GROUPS(timer_trig);

static void
led_timer_trig_deactivate(struct led_classdev *led_cdev)
{
	/* Stop blinking */
	led_set_brightness(led_cdev, LED_OFF);
}

int
led_trigger_init(struct device *dev)
{
	int ret;
	struct led_classdev *led_cdev =
			((struct led_classdev *)dev_get_drvdata(dev));

	ret = device_add_groups(led_cdev->dev, timer_trig_groups);
	if (ret)
		dev_err(led_cdev->dev, "Failed to add trigger attributes\n");
	return ret;
}

int
led_trigger_deinit(struct device *dev)
{
	struct led_classdev *led_cdev =
			((struct led_classdev *)dev_get_drvdata(dev));

	led_timer_trig_deactivate(led_cdev);
	device_remove_groups(led_cdev->dev, timer_trig_groups);
	return 0;
}
#else /* CONFIG_LEDS_TRIGGERS */
/*
 * Do NOT create sysfs files in led_trigger_init() when CONFIG_LEDS_TRIGGERS
 * is enabled, because led_classdev_register() API will create "trigger"
 * sysfs entry in this condition.
 */
int
led_trigger_init(struct device *dev)
{
	return 0;
}

int
led_trigger_deinit(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_LEDS_TRIGGERS */

/*
 * libudev - interface to udev device information
 *
 * Copyright (C) 2008 Kay Sievers <kay.sievers@vrfy.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>

#include "libudev.h"
#include "libudev-private.h"

static size_t devpath_to_db_path(struct udev *udev, const char *devpath, char *filename, size_t len)
{
	char *s;
	size_t l;

	s = filename;
	l = util_strpcpyl(&s, len, udev_get_dev_path(udev), "/.udev/db/", NULL);
	return util_path_encode(devpath, s, l);
}

int udev_device_update_db(struct udev_device *udev_device)
{
	struct udev *udev = udev_device_get_udev(udev_device);
	char filename[UTIL_PATH_SIZE];
	FILE *f;
	char target[232]; /* on 64bit, tmpfs inlines up to 239 bytes */
	size_t devlen = strlen(udev_get_dev_path(udev))+1;
	char *s;
	size_t l;
	struct udev_list_entry *list_entry;
	int ret;

	devpath_to_db_path(udev, udev_device_get_devpath(udev_device), filename, sizeof(filename));
	util_create_path(udev, filename);
	unlink(filename);

	udev_list_entry_foreach(list_entry, udev_device_get_properties_list_entry(udev_device))
		if (udev_list_entry_get_flag(list_entry))
			goto file;
	if (udev_device_get_num_fake_partitions(udev_device) != 0)
		goto file;
	if (udev_device_get_ignore_remove(udev_device))
		goto file;
	if (udev_device_get_devlink_priority(udev_device) != 0)
		goto file;
	if (udev_device_get_event_timeout(udev_device) >= 0)
		goto file;
	if (udev_device_get_watch_handle(udev_device) >= 0)
		goto file;
	if (udev_device_get_devnode(udev_device) == NULL)
		goto out;

	/*
	 * if we have only the node and symlinks to store, try not to waste
	 * tmpfs memory -- store values, if they fit, in a symlink target
	 */
	s = target;
	l = util_strpcpy(&s, sizeof(target), &udev_device_get_devnode(udev_device)[devlen]);
	udev_list_entry_foreach(list_entry, udev_device_get_devlinks_list_entry(udev_device)) {
		l = util_strpcpyl(&s, l, " ", &udev_list_entry_get_name(list_entry)[devlen], NULL);
		if (l == 0) {
			info(udev, "size of links too large, create file\n");
			goto file;
		}
	}
	info(udev, "create db link (%s)\n", target);
	udev_selinux_setfscreatecon(udev, filename, S_IFLNK);
	ret = symlink(target, filename);
	udev_selinux_resetfscreatecon(udev);
	if (ret == 0)
		goto out;
file:
	f = fopen(filename, "w");
	if (f == NULL) {
		err(udev, "unable to create db file '%s': %m\n", filename);
		return -1;
		}
	info(udev, "created db file for '%s' in '%s'\n", udev_device_get_devpath(udev_device), filename);

	if (udev_device_get_devnode(udev_device) != NULL) {
		fprintf(f, "N:%s\n", &udev_device_get_devnode(udev_device)[devlen]);
		udev_list_entry_foreach(list_entry, udev_device_get_devlinks_list_entry(udev_device))
			fprintf(f, "S:%s\n", &udev_list_entry_get_name(list_entry)[devlen]);
	}
	if (udev_device_get_devlink_priority(udev_device) != 0)
		fprintf(f, "L:%u\n", udev_device_get_devlink_priority(udev_device));
	if (udev_device_get_event_timeout(udev_device) >= 0)
		fprintf(f, "T:%u\n", udev_device_get_event_timeout(udev_device));
	if (udev_device_get_num_fake_partitions(udev_device) != 0)
		fprintf(f, "A:%u\n", udev_device_get_num_fake_partitions(udev_device));
	if (udev_device_get_ignore_remove(udev_device))
		fprintf(f, "R:%u\n", udev_device_get_ignore_remove(udev_device));
	if (udev_device_get_watch_handle(udev_device) >= 0)
		fprintf(f, "W:%u\n", udev_device_get_watch_handle(udev_device));
	udev_list_entry_foreach(list_entry, udev_device_get_properties_list_entry(udev_device)) {
		if (!udev_list_entry_get_flag(list_entry))
			continue;
		fprintf(f, "E:%s=%s\n",
			udev_list_entry_get_name(list_entry),
			udev_list_entry_get_value(list_entry));
	}
	fclose(f);
out:
	return 0;
}

int udev_device_delete_db(struct udev_device *udev_device)
{
	struct udev *udev = udev_device_get_udev(udev_device);
	char filename[UTIL_PATH_SIZE];

	devpath_to_db_path(udev, udev_device_get_devpath(udev_device), filename, sizeof(filename));
	unlink(filename);
	return 0;
}

int udev_device_rename_db(struct udev_device *udev_device, const char *devpath_old)
{
	struct udev *udev = udev_device_get_udev(udev_device);
	char filename_old[UTIL_PATH_SIZE];
	char filename[UTIL_PATH_SIZE];

	devpath_to_db_path(udev, devpath_old, filename_old, sizeof(filename_old));
	devpath_to_db_path(udev, udev_device_get_devpath(udev_device), filename, sizeof(filename));
	return rename(filename_old, filename);
}

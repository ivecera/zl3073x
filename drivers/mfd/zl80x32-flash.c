// SPDX-License-Identifier: GPL-2.0-or-later

#include "zl80x32.h"
#include "zl80x32-flash.h"

/**
 * zl80x32_flash_update - Devlink flash update callback
 * @devlink: devlink structure pointer
 * @params: flashing parameters pointer
 * @extack: netlink ext ack pointer to report errors
 *
 * Returns 0 in case of success or negative value otherwise
 */
int zl80x32_flash_update(struct devlink *devlink,
			 struct devlink_flash_update_params *params,
			 struct netlink_ext_ack *extack)
{
	devlink_flash_update_status_notify(devlink, "Preparing to flash",
					   params->component, 0, 0);

	devlink_flash_update_status_notify(devlink, "Flashing done",
					   params->component, 0, 0);

	return 0;
}

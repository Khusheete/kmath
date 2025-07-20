#pragma once

void *quat_dquat_init();
void quat_dquat_run(void *p_data);
void quat_dquat_cleanup(void *p_data);


void *camera_init();
void camera_run(void *p_data);
void camera_cleanup(void *p_data);

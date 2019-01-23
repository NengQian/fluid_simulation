#pragma once

bool playback_flag;
bool pausing_flag;
bool render_velocity_flag;
bool render_density_flag;
bool render_mesh_flag;
bool render_particle_flag;
bool render_bounding_box_flag;
bool render_discarded_particle_flag;

float boundary_particle_size;
float particle_size;

float render_max_velocity;
float render_max_density;

float speed_ratio;
int sim_count;
int global_total_frame = INT_MAX;
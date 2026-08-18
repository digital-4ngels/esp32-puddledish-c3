#pragma once

#include <stdint.h>

typedef struct {
    float x, y, z;
    float speed;
} sim_particle_view_t;

typedef struct {
    float gravity[3];
    float omega[3];
    float alpha[3];
} sim_forces_t;

typedef struct {
    float mean_density;
    float rest_density;
    float mean_speed;
    float max_speed;
    int us_grid;
    int us_density;
    int us_relax;
    float front_density, back_density;
    float front_speed, back_speed;
    float front_push, back_push;
    int front_count, back_count;
    int front_hits, back_hits;
    int clamped;
    int pairs;
} sim_stats_t;

void sim_init(void);
void sim_reset(void);
void sim_step(float dt_real, const sim_forces_t *forces);
int sim_snapshot(sim_particle_view_t *out, int max);
void sim_stats(sim_stats_t *out);

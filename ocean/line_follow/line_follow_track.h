#ifndef LINE_FOLLOW_TRACK_H
#define LINE_FOLLOW_TRACK_H

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#ifndef LINE_FOLLOW_PI
#define LINE_FOLLOW_PI 3.14159265358979323846f
#endif

#define LINE_FOLLOW_MAX_TRACK_SAMPLES 384
#define LINE_FOLLOW_TRACK_SAMPLE_SPACING_M 0.02f

#define LINE_FOLLOW_TRACK_RANDOM -1
#define LINE_FOLLOW_TRACK_STRAIGHT 0
#define LINE_FOLLOW_TRACK_ARC 1
#define LINE_FOLLOW_TRACK_S_CURVE 2
#define LINE_FOLLOW_TRACK_OVAL 3

typedef struct {
    float x;
    float y;
    float tangent_x;
    float tangent_y;
    float normal_x;
    float normal_y;
    float progress_s;
    float line_width_m;
    float black_reflectance;
} LineFollowTrackSample;

typedef struct {
    LineFollowTrackSample samples[LINE_FOLLOW_MAX_TRACK_SAMPLES];
    int sample_count;
    int family;
    float length_m;
    float bounds_m;
} LineFollowTrack;

typedef struct {
    float distance_m;
    float signed_lateral_m;
    float progress_s;
    float heading_rad;
    float line_width_m;
    float black_reflectance;
    int segment_idx;
} LineFollowNearest;

static inline float track_clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static inline float rand01(unsigned int* rng) {
    return (float)rand_r(rng) / (float)RAND_MAX;
}

static inline void track_begin(LineFollowTrack* track, int family, float bounds_m) {
    memset(track, 0, sizeof(*track));
    track->family = family;
    track->bounds_m = bounds_m;
}

static inline bool track_push(LineFollowTrack* track, float x, float y,
        float line_width_m, float black_reflectance) {
    if (track->sample_count >= LINE_FOLLOW_MAX_TRACK_SAMPLES) {
        return false;
    }

    LineFollowTrackSample* sample = &track->samples[track->sample_count++];
    sample->x = x;
    sample->y = y;
    sample->line_width_m = line_width_m;
    sample->black_reflectance = black_reflectance;
    return true;
}

static inline bool track_finalize(LineFollowTrack* track) {
    if (track->sample_count < 2) {
        return false;
    }

    track->samples[0].progress_s = 0.0f;
    for (int i = 1; i < track->sample_count; i++) {
        float dx = track->samples[i].x - track->samples[i - 1].x;
        float dy = track->samples[i].y - track->samples[i - 1].y;
        float ds = sqrtf(dx * dx + dy * dy);
        track->samples[i].progress_s = track->samples[i - 1].progress_s + ds;
    }
    track->length_m = track->samples[track->sample_count - 1].progress_s;

    for (int i = 0; i < track->sample_count; i++) {
        int a = i == 0 ? 0 : i - 1;
        int b = i == track->sample_count - 1 ? i : i + 1;
        float dx = track->samples[b].x - track->samples[a].x;
        float dy = track->samples[b].y - track->samples[a].y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-6f) {
            dx = 1.0f;
            dy = 0.0f;
            len = 1.0f;
        }
        float tx = dx / len;
        float ty = dy / len;
        track->samples[i].tangent_x = tx;
        track->samples[i].tangent_y = ty;
        track->samples[i].normal_x = -ty;
        track->samples[i].normal_y = tx;
    }

    return track->length_m > 1e-5f;
}

static inline bool track_in_bounds(const LineFollowTrack* track, float bounds_m) {
    if (track->sample_count < 2 || track->sample_count > LINE_FOLLOW_MAX_TRACK_SAMPLES) {
        return false;
    }

    for (int i = 0; i < track->sample_count; i++) {
        if (fabsf(track->samples[i].x) > bounds_m || fabsf(track->samples[i].y) > bounds_m) {
            return false;
        }
    }
    return true;
}

static inline bool generate_straight(LineFollowTrack* track, float length_m,
        float line_width_m, float bounds_m) {
    length_m = track_clampf(length_m, 0.2f, 2.0f * bounds_m);
    track_begin(track, LINE_FOLLOW_TRACK_STRAIGHT, bounds_m);

    int samples = (int)ceilf(length_m / LINE_FOLLOW_TRACK_SAMPLE_SPACING_M) + 1;
    if (samples > LINE_FOLLOW_MAX_TRACK_SAMPLES) {
        samples = LINE_FOLLOW_MAX_TRACK_SAMPLES;
    }
    if (samples < 2) {
        samples = 2;
    }

    for (int i = 0; i < samples; i++) {
        float t = samples == 1 ? 0.0f : (float)i / (float)(samples - 1);
        float x = (t - 0.5f) * length_m;
        if (!track_push(track, x, 0.0f, line_width_m, 1.0f)) {
            return false;
        }
    }

    return track_finalize(track) && track_in_bounds(track, bounds_m);
}

static inline bool generate_arc(LineFollowTrack* track, float radius_m,
        float angle_rad, float line_width_m, float bounds_m) {
    radius_m = track_clampf(radius_m, 0.08f, bounds_m * 0.85f);
    angle_rad = track_clampf(angle_rad, 0.45f, 1.7f);
    track_begin(track, LINE_FOLLOW_TRACK_ARC, bounds_m);

    float length_m = radius_m * angle_rad;
    int samples = (int)ceilf(length_m / LINE_FOLLOW_TRACK_SAMPLE_SPACING_M) + 1;
    if (samples > LINE_FOLLOW_MAX_TRACK_SAMPLES) {
        samples = LINE_FOLLOW_MAX_TRACK_SAMPLES;
    }

    for (int i = 0; i < samples; i++) {
        float t = (float)i / (float)(samples - 1);
        float phi = (t - 0.5f) * angle_rad;
        float x = radius_m * sinf(phi);
        float y = radius_m * (1.0f - cosf(phi));
        if (!track_push(track, x, y, line_width_m, 1.0f)) {
            return false;
        }
    }

    return track_finalize(track) && track_in_bounds(track, bounds_m);
}

static inline bool generate_s_curve(LineFollowTrack* track, float length_m,
        float amplitude_m, float line_width_m, float bounds_m) {
    length_m = track_clampf(length_m, 0.2f, 1.7f * bounds_m);
    amplitude_m = track_clampf(amplitude_m, 0.015f, 0.22f * bounds_m);
    track_begin(track, LINE_FOLLOW_TRACK_S_CURVE, bounds_m);

    int samples = (int)ceilf(length_m / LINE_FOLLOW_TRACK_SAMPLE_SPACING_M) + 1;
    if (samples > LINE_FOLLOW_MAX_TRACK_SAMPLES) {
        samples = LINE_FOLLOW_MAX_TRACK_SAMPLES;
    }

    for (int i = 0; i < samples; i++) {
        float t = (float)i / (float)(samples - 1);
        float x = (t - 0.5f) * length_m;
        float y = amplitude_m * sinf(2.0f * LINE_FOLLOW_PI * t);
        if (!track_push(track, x, y, line_width_m, 1.0f)) {
            return false;
        }
    }

    return track_finalize(track) && track_in_bounds(track, bounds_m);
}

static inline bool generate_oval(LineFollowTrack* track, float radius_x_m,
        float radius_y_m, float line_width_m, float bounds_m) {
    radius_x_m = track_clampf(radius_x_m, 0.06f, bounds_m * 0.75f);
    radius_y_m = track_clampf(radius_y_m, 0.035f, bounds_m * 0.55f);
    track_begin(track, LINE_FOLLOW_TRACK_OVAL, bounds_m);

    float approx_len = 2.0f * LINE_FOLLOW_PI * sqrtf((radius_x_m * radius_x_m
        + radius_y_m * radius_y_m) * 0.5f);
    int samples = (int)ceilf(approx_len / LINE_FOLLOW_TRACK_SAMPLE_SPACING_M) + 1;
    if (samples > LINE_FOLLOW_MAX_TRACK_SAMPLES) {
        samples = LINE_FOLLOW_MAX_TRACK_SAMPLES;
    }
    if (samples < 16) {
        samples = 16;
    }

    for (int i = 0; i < samples; i++) {
        float t = (float)i / (float)(samples - 1);
        float theta = 2.0f * LINE_FOLLOW_PI * t;
        float x = radius_x_m * cosf(theta);
        float y = radius_y_m * sinf(theta);
        if (!track_push(track, x, y, line_width_m, 1.0f)) {
            return false;
        }
    }

    return track_finalize(track) && track_in_bounds(track, bounds_m);
}

static inline bool generate_track(LineFollowTrack* track, unsigned int* rng,
        int family, float line_width_m, float bounds_m) {
    int chosen = family;
    if (chosen == LINE_FOLLOW_TRACK_RANDOM) {
        chosen = (int)(rand_r(rng) % 4);
    }

    bool ok = false;
    if (chosen == LINE_FOLLOW_TRACK_STRAIGHT) {
        float length = 0.25f + 0.25f * rand01(rng);
        ok = generate_straight(track, length, line_width_m, bounds_m);
    } else if (chosen == LINE_FOLLOW_TRACK_ARC) {
        float radius = 0.10f + 0.12f * rand01(rng);
        float angle = 0.7f + 0.8f * rand01(rng);
        ok = generate_arc(track, radius, angle, line_width_m, bounds_m);
    } else if (chosen == LINE_FOLLOW_TRACK_S_CURVE) {
        float length = 0.30f + 0.25f * rand01(rng);
        float amp = 0.025f + 0.045f * rand01(rng);
        ok = generate_s_curve(track, length, amp, line_width_m, bounds_m);
    } else if (chosen == LINE_FOLLOW_TRACK_OVAL) {
        float rx = 0.075f + 0.045f * rand01(rng);
        float ry = 0.045f + 0.035f * rand01(rng);
        ok = generate_oval(track, rx, ry, line_width_m, bounds_m);
    }

    if (!ok) {
        ok = generate_straight(track, bounds_m * 1.2f, line_width_m, bounds_m);
    }
    return ok;
}

static inline LineFollowNearest nearest_track(const LineFollowTrack* track,
        float x, float y) {
    LineFollowNearest best;
    memset(&best, 0, sizeof(best));
    best.distance_m = 1e9f;
    best.line_width_m = 0.006f;
    best.black_reflectance = 1.0f;

    if (track->sample_count < 2) {
        return best;
    }

    for (int i = 0; i < track->sample_count - 1; i++) {
        const LineFollowTrackSample* a = &track->samples[i];
        const LineFollowTrackSample* b = &track->samples[i + 1];
        float vx = b->x - a->x;
        float vy = b->y - a->y;
        float len2 = vx * vx + vy * vy;
        if (len2 < 1e-12f) {
            continue;
        }

        float wx = x - a->x;
        float wy = y - a->y;
        float t = track_clampf((wx * vx + wy * vy) / len2, 0.0f, 1.0f);
        float px = a->x + t * vx;
        float py = a->y + t * vy;
        float dx = x - px;
        float dy = y - py;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist < best.distance_m) {
            float len = sqrtf(len2);
            float tx = vx / len;
            float ty = vy / len;
            float nx = -ty;
            float ny = tx;
            best.distance_m = dist;
            best.signed_lateral_m = dx * nx + dy * ny;
            best.progress_s = a->progress_s + t * len;
            best.heading_rad = atan2f(ty, tx);
            best.line_width_m = a->line_width_m + t * (b->line_width_m - a->line_width_m);
            best.black_reflectance = a->black_reflectance
                + t * (b->black_reflectance - a->black_reflectance);
            best.segment_idx = i;
        }
    }

    return best;
}

static inline float line_coverage(float signed_lateral_m,
        float line_width_m, float edge_softness_m) {
    float dist = fabsf(signed_lateral_m);
    float half_width = 0.5f * line_width_m;
    if (edge_softness_m <= 1e-6f) {
        return dist <= half_width ? 1.0f : 0.0f;
    }

    float inner = fmaxf(0.0f, half_width - edge_softness_m);
    float outer = half_width + edge_softness_m;
    if (dist <= inner) {
        return 1.0f;
    }
    if (dist >= outer) {
        return 0.0f;
    }

    float t = (dist - inner) / (outer - inner);
    float smooth = t * t * (3.0f - 2.0f * t);
    return 1.0f - smooth;
}

static inline float angle_diff(float a, float b) {
    float d = a - b;
    while (d > LINE_FOLLOW_PI) d -= 2.0f * LINE_FOLLOW_PI;
    while (d < -LINE_FOLLOW_PI) d += 2.0f * LINE_FOLLOW_PI;
    return d;
}

#endif

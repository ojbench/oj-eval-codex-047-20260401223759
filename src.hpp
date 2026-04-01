#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include <cmath>
#include <vector>

// Include judge-provided headers by logical names as OJ compiles with them
#include "math.h"
#include "monitor.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) { pos_cur = _pos_cur; }
    void set_v_cur(const Vec &_v_cur) { v_cur = _v_cur; }

private:
    int id{};
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max{0}, r{0};
    Monitor *monitor{nullptr};

    // Helper: clamp vector length to max_len
    Vec clamp_speed(const Vec &v, double max_len) const {
        double n = v.norm();
        if (n <= max_len) return v;
        if (n < 1e-12) return Vec();
        return v * (max_len / n);
    }

    // Predict if choosing velocity v_self will collide with any other robot in next interval
    bool will_collide(const Vec &v_self) const {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            Vec vj = monitor->get_v_cur(j);
            double rj = monitor->get_r(j);
            Vec delta_pos = pos_cur - pj;
            Vec delta_v = v_self - vj;
            double project = delta_pos.dot(delta_v);
            if (project >= 0) {
                continue; // moving apart or orthogonal
            }
            double dvn = delta_v.norm();
            if (dvn < 1e-12) {
                // No relative motion; if already overlapping (shouldn’t), treat as unsafe
                double delta_r = r + rj;
                if (delta_pos.norm_sqr() <= delta_r * delta_r - 1e-9) return true;
                continue;
            }
            project /= -dvn; // distance along relative motion to closest approach
            double min_dis_sqr;
            double delta_r = r + rj;
            if (project < dvn * TIME_INTERVAL) {
                min_dis_sqr = delta_pos.norm_sqr() - project * project;
            } else {
                Vec end_delta = delta_pos + delta_v * TIME_INTERVAL;
                min_dis_sqr = end_delta.norm_sqr();
            }
            if (min_dis_sqr <= delta_r * delta_r - EPSILON) {
                return true;
            }
        }
        return false;
    }

public:
    Vec get_v_next() {
        // If already at target (within EPS), stop.
        if ((pos_cur - pos_tar).norm_sqr() <= EPSILON * EPSILON) {
            return Vec();
        }

        // Greedy move towards target at max speed, but keep conservative speed when close.
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();

        // Desired speed: min(v_max, dist / TIME_INTERVAL) to not overshoot in one tick.
        double desired_speed = std::min(v_max, dist / TIME_INTERVAL);
        Vec v_desired = (dist < 1e-12) ? Vec() : to_tar * (desired_speed / dist);

        // If current desired leads to no collision, take it.
        if (!will_collide(v_desired)) {
            return v_desired;
        }

        // Otherwise, try slower speeds down to zero.
        const int STEPS = 8;
        for (int k = STEPS - 1; k >= 1; --k) {
            double sp = desired_speed * (double)k / (double)STEPS;
            Vec v_try = (dist < 1e-12) ? Vec() : to_tar * (sp / dist);
            if (!will_collide(v_try)) return v_try;
        }

        // As last resort, try sidestep directions: perpendiculars and small rotations.
        Vec dir = (dist < 1e-12) ? Vec(1, 0) : to_tar * (1.0 / dist);
        std::vector<double> angles = {PI/2, -PI/2, PI/4, -PI/4, PI*3/4, -PI*3/4, PI};
        for (double ang : angles) {
            Vec side = dir.rotate(ang);
            Vec v_try = side * (desired_speed * 0.6);
            if (!will_collide(v_try)) return v_try;
            v_try = side * (desired_speed * 0.3);
            if (!will_collide(v_try)) return v_try;
        }

        // If all else fails, stop to avoid collision.
        return Vec();
    }
};

#endif // PPCA_SRC_HPP

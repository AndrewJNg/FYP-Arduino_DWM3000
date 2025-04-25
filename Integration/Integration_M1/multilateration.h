#include <stdio.h>
#include <math.h>

#define MAX_ROBOTS 10

typedef struct {
    int bot_id;
    double x;
    double y;
    double theta;
    unsigned long last_sent_time;
    double distances;  // Distance from this robot to others

} Robot;

typedef struct {
    double x;
    double y;
} Position;

int Trilateration_2D(Robot robots[], int n, int target_id, double* out_x, double* out_y) {
    int target_index = -1;

    for (int i = 0; i < n; ++i) {
        if (robots[i].bot_id == target_id) {
            target_index = i;
            break;
        }
    }

    if (target_index == -1) return -1;  // Target not found

    Position neighbors[MAX_ROBOTS];
    double neighbor_distances[MAX_ROBOTS];
    int ref_count = 0;

    // Loop through all robots to find ones that have distances to the target
    for (int i = 0; i < n; ++i) {
        if (i == target_index) continue;

        double d = robots[i].distances;  // Distance from robot[i] to target robot
        if (d <= 0) continue;  // Skip if no valid distance

        neighbors[ref_count].x = robots[i].x;
        neighbors[ref_count].y = robots[i].y;
        neighbor_distances[ref_count] = d;
        ref_count++;
    }

    if (ref_count < 2) return -2;  // Not enough references

    double x_prev = robots[target_index].x;
    double y_prev = robots[target_index].y;

    if (ref_count == 2) {
        // Two-circle intersection
        double x1 = neighbors[0].x, y1 = neighbors[0].y, d1 = neighbor_distances[0];
        double x2 = neighbors[1].x, y2 = neighbors[1].y, d2 = neighbor_distances[1];

        double D = hypot(x2 - x1, y2 - y1);
        if (D > d1 + d2 || D < fabs(d1 - d2) || D == 0) return -3;

        double a = (d1*d1 - d2*d2 + D*D) / (2 * D);
        double px = x1 + a * (x2 - x1) / D;
        double py = y1 + a * (y2 - y1) / D;
        double h = sqrt(d1*d1 - a*a);
        double rx = -(y2 - y1) * (h / D);
        double ry =  (x2 - x1) * (h / D);

        double xA = px + rx, yA = py + ry;
        double xB = px - rx, yB = py - ry;

        // Pick solution closer to previous estimate
        double distA = hypot(xA - x_prev, yA - y_prev);
        double distB = hypot(xB - x_prev, yB - y_prev);
        if (distA < distB) {
            *out_x = xA;
            *out_y = yA;
        } else {
            *out_x = xB;
            *out_y = yB;
        }
    } else {
        // Least-squares trilateration
        double A[MAX_ROBOTS][2];
        double b[MAX_ROBOTS];
        double x1 = neighbors[0].x, y1 = neighbors[0].y, d1 = neighbor_distances[0];

        for (int i = 1; i < ref_count; ++i) {
            double xi = neighbors[i].x;
            double yi = neighbors[i].y;
            double di = neighbor_distances[i];

            A[i-1][0] = 2 * (xi - x1);
            A[i-1][1] = 2 * (yi - y1);
            b[i-1] = d1*d1 - di*di + xi*xi - x1*x1 + yi*yi - y1*y1;
        }

        double AtA[2][2] = {0}, Atb[2] = {0};
        for (int i = 0; i < ref_count - 1; ++i) {
            AtA[0][0] += A[i][0] * A[i][0];
            AtA[0][1] += A[i][0] * A[i][1];
            AtA[1][0] += A[i][1] * A[i][0];
            AtA[1][1] += A[i][1] * A[i][1];

            Atb[0] += A[i][0] * b[i];
            Atb[1] += A[i][1] * b[i];
        }

        double det = AtA[0][0]*AtA[1][1] - AtA[0][1]*AtA[1][0];
        if (fabs(det) < 1e-6) return -4;

        *out_x = (Atb[0]*AtA[1][1] - Atb[1]*AtA[0][1]) / det;
        *out_y = (Atb[1]*AtA[0][0] - Atb[0]*AtA[1][0]) / det;
    }

    return 0;
}

// int main() {
//     Robot robots[4] = {
//         {.bot_id = 0xAA, .x = 2, .y = -2, .theta = 0, .last_sent_time = 0, .distances = 0},
//         {.bot_id = 0xBB, .x = 2, .y = -2, .theta = 0, .last_sent_time = 0, .distances = 7.0000},
//         {.bot_id = 0xCC, .x = -2, .y = 2, .theta = 0, .last_sent_time = 0, .distances = 5.0000},
//         {.bot_id = 0xDD, .x = 2, .y =  2, .theta = 0, .last_sent_time = 0, .distances = 8.0623}
//     };

    

//     double x, y;
//     int status = Trilateration_2D(robots, 4, 0xAA, &x, &y);
//     if (status == 0) {
//         printf("Estimated position: (%.2f, %.2f)\n", x, y);
//     } else {
//         printf("Trilateration failed. Error code: %d\n", status);
//     }

//     return 0;
// }


/*
r = wheel radius
L = half of robot length
W = half of robot wdith
delta_phi[1-4] = wheel displacement (radians or distance)
dt = time stamp 
dx,dy,dtheta = pose chane in robot frame
*/

void updateMecanumOdometry(double delta_phi[4], double L, double W,
    double* x_mm, double* y_mm, double* theta_rad) {

    // Robot-centric velocities (already in mm/s)
    double vx_r = (1.0 / 4.0) * (delta_phi[0] + delta_phi[1] + delta_phi[2] + delta_phi[3]);
    double vy_r = (1.0 / 4.0) * (-delta_phi[0] + delta_phi[1] + delta_phi[2] - delta_phi[3]);
    double omega = (1.0 / (4.0 * (L + W))) * (-delta_phi[0] + delta_phi[1] - delta_phi[2] + delta_phi[3]);  // rad/s

    double vx = vx_r * cos(*theta_rad) - vy_r * sin(*theta_rad);
    double vy = vx_r * sin(*theta_rad) + vy_r * cos(*theta_rad);

    *x_mm += vx;
    *y_mm += vy;
    *theta_rad += omega;
}

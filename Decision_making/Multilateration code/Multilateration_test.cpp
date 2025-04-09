#include <stdio.h>
#include <math.h>

#define MAX_ROBOTS 10

typedef struct {
    double x;
    double y;
} Position;

int Trilateration_2D(Position actual_positions[], double distances[][MAX_ROBOTS],
                     int n, int robot_id, double* out_x, double* out_y) {

    if (robot_id < 0 || robot_id >= n) {
        return -1;  // Invalid robot_id
    }

    Position neighbors[MAX_ROBOTS];
    double neighbor_distances[MAX_ROBOTS];
    int ref_count = 0;

    // Collect neighbor positions and distances
    for (int i = 0; i < n; ++i) {
        if (i == robot_id) continue;
        neighbors[ref_count] = actual_positions[i];
        neighbor_distances[ref_count] = distances[robot_id][i];
        ref_count++;
    }

    if (ref_count < 2) {
        return -2;  // Not enough reference points
    }

    double x_prev = actual_positions[robot_id].x;
    double y_prev = actual_positions[robot_id].y;

    if (ref_count == 2) {
        // Two-circle intersection
        double x1 = neighbors[0].x, y1 = neighbors[0].y, d1 = neighbor_distances[0];
        double x2 = neighbors[1].x, y2 = neighbors[1].y, d2 = neighbor_distances[1];

        double D = hypot(x2 - x1, y2 - y1);

        if (D > d1 + d2 || D < fabs(d1 - d2) || D == 0) {
            return -3;  // No intersection
        }

        double a = (d1*d1 - d2*d2 + D*D) / (2 * D);
        double px = x1 + a * (x2 - x1) / D;
        double py = y1 + a * (y2 - y1) / D;
        double h = sqrt(d1*d1 - a*a);
        double rx = -(y2 - y1) * (h / D);
        double ry =  (x2 - x1) * (h / D);

        double xA = px + rx, yA = py + ry;
        double xB = px - rx, yB = py - ry;

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

        // Solve least squares: x = (AᵗA)^-1 Aᵗb for 2D
        double AtA[2][2] = {0}, Atb[2] = {0};

        for (int i = 0; i < ref_count - 1; ++i) {
            AtA[0][0] += A[i][0] * A[i][0];
            AtA[0][1] += A[i][0] * A[i][1];
            AtA[1][0] += A[i][1] * A[i][0];
            AtA[1][1] += A[i][1] * A[i][1];

            Atb[0] += A[i][0] * b[i];
            Atb[1] += A[i][1] * b[i];
        }

        // Solve 2x2 system [AtA]{x} = {Atb}
        double det = AtA[0][0]*AtA[1][1] - AtA[0][1]*AtA[1][0];
        if (fabs(det) < 1e-6) {
            return -4; // Singular matrix
        }

        *out_x = (Atb[0]*AtA[1][1] - Atb[1]*AtA[0][1]) / det;
        *out_y = (Atb[1]*AtA[0][0] - Atb[0]*AtA[1][0]) / det;
    }

    return 0; // Success
}

int main() {
    Position positions[4] = {{-5, -2}, {2, -2}, {-2, 2}, {2, 2}};
    double distances[4][MAX_ROBOTS] = {
        {0 ,   7.0000 ,   5.0000 ,   8.0623},
        {7.0000,         0,    5.6569 ,   4.0000},
        {5.0000 ,   5.6569,         0,    4.0000},
        {8.0623,    4.0000,    4.0000 ,        0}
    };

    double x, y;
    int status = Trilateration_2D(positions, distances, 4, 0, &x, &y);
    if (status == 0) {
        printf("Estimated position: (%.2f, %.2f)\n", x, y);
    } else {
        printf("Trilateration failed. Error code: %d\n", status);
    }

    return 0;
}


// int Trilateration_2D(Position actual_positions[], double distances[][MAX_ROBOTS],
//     int n, int robot_id, double* out_x, double* out_y) {
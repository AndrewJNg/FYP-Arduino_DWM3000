#include <stdio.h>
#include <math.h>

#define MAX_ROBOTS 10

typedef struct {
    double x;
    double y;
} Position;



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


int main() {
    double delta_phi[4] = {-377, 377, -377, 377};  // distance in mm moved by each wheel
    double L = 60.0;  // in mm
    double W = 60.0;  // in mm

    double x = 0.0, y = 0.0, theta = 0.0;  // Pose in mm and radians

    updateMecanumOdometry(delta_phi, L, W, &x, &y, &theta);

    printf("x = %.6f mm\ny = %.6f mm\ntheta = %.6f degree\n", x, y, theta*180/3.142);
    return 0;
}
#include <iostream>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <iomanip>
#include <cmath>

#include "kalmanFilter.h"

class RobotSwarm {
private:
  struct Robot {
    int16_t address;
    double position[3];  // x, y, z coordinates
    double velocity[3];  // vx, vy, vz components
    unsigned long last_update_time;
    double distance;  // Distance measurement from this robot to the base

    double filtered_distance;  // Kalman-filtered distance
    Kalman distance_filter;    // Individual Kalman filter for this robot
    


    Robot(int16_t addr)
      : address(addr), distance(0.0), filtered_distance(0.0),
        distance_filter(0.2, 0.2, 0.1) {
      position[0] = position[1] = position[2] = 0.0;
      velocity[0] = velocity[1] = velocity[2] = 0.0;
      update_timestamp();
    }

    void update_timestamp() {
      last_update_time = current_millis();
    }

    static unsigned long current_millis() {
      // return std::chrono::system_clock::now().time_since_epoch() /
      // std::chrono::milliseconds(1);
      return millis();
    }
    void update_distance(double new_distance) {
      distance = new_distance;
      filtered_distance = distance_filter.updateFilter(new_distance);
      update_timestamp();
    }
  };
  std::unordered_map<int16_t, Robot> robots;

  // Helper structure for trilateration
  struct Position {
    double x, y;
  };

public:
  int16_t base_bot_id;                        // Store the base robot ID
  const unsigned long STALE_TIMEOUT = 10000;  //60000;  // 1 minute in milliseconds
  RobotSwarm(int16_t base_id)
    : base_bot_id(base_id) {}


  // Add a new robot address to the swarm
  bool add_address(int16_t address) {
    if (robots.find(address) != robots.end()) {
      return false;  // Address already exists
    }
    robots.emplace(address, Robot(address));
    return true;
  }

  // Access a robot by address
  Robot* get_robot(int16_t address) {
    auto it = robots.find(address);
    if (it != robots.end()) {
      return &(it->second);
    }
    return nullptr;
  }

  // Get all robot addresses in the swarm
  std::vector<int16_t> get_all_addresses() const {
    std::vector<int16_t> addresses;
    for (const auto& pair : robots) {
      addresses.push_back(pair.first);
    }
    return addresses;
  }

  // Remove a robot from the swarm
  bool remove_robot(int16_t address) {
    return robots.erase(address) > 0;
  }

  // Get the number of robots in the swarm
  size_t size() const {
    return robots.size();
  }

  // Print all robots in the swarm
  void print_Robot_Swarm() const {
    Serial.println("\n=== Robot Swarm Status ===");
    Serial.print("Total robots: ");
    Serial.println(robots.size());
    Serial.println();

    for (const auto& pair : robots) {
      const Robot& robot = pair.second;

      Serial.print("Robot Address: 0x");
      Serial.println(robot.address, HEX);

      Serial.print("Position: [");
      Serial.print(String(robot.position[0], 2));
      Serial.print(", ");
      Serial.print(String(robot.position[1], 2));
      Serial.print(", ");
      Serial.print(String(robot.position[2], 2));
      Serial.println("]");

      Serial.print("Velocity: [");
      Serial.print(String(robot.velocity[0], 2));
      Serial.print(", ");
      Serial.print(String(robot.velocity[1], 2));
      Serial.print(", ");
      Serial.print(String(robot.velocity[2], 2));
      Serial.println("]");

      Serial.print("Time since last msg: ");
      Serial.print(millis()-robot.last_update_time);
      Serial.println(" ms");

      Serial.print("Distance: ");
      Serial.print(String(robot.distance, 2));
      Serial.println(" m");

      Serial.println("------------------------");
    }
  }
  // Enhanced Trilateration function
  int trilaterate_2D(int16_t target_address, double* out_x, double* out_y) {
    auto target_it = robots.find(target_address);
    if (target_it == robots.end()) return -1;  // Target not found

    std::vector<Position> neighbors;
    std::vector<double> neighbor_distances;

    // Find all robots that have distance measurements to the target
    for (const auto& pair : robots) {
      if (pair.first == target_address) continue;  // Skip the target itself

      const Robot& robot = pair.second;
      if (robot.distance <= 0) continue;  // Skip invalid distances

      neighbors.push_back({ robot.position[0], robot.position[1] });
      neighbor_distances.push_back(robot.distance);
    }

    const size_t ref_count = neighbors.size();
    if (ref_count < 2) return -2;  // Not enough reference points

    double x_prev = target_it->second.position[0];
    double y_prev = target_it->second.position[1];

    if (ref_count == 2) {
      // Two-circle intersection
      const auto& p1 = neighbors[0];
      const auto& p2 = neighbors[1];
      double d1 = neighbor_distances[0];
      double d2 = neighbor_distances[1];

      double D = hypot(p2.x - p1.x, p2.y - p1.y);
      if (D > d1 + d2 || D < fabs(d1 - d2) || D == 0) return -3;

      double a = (d1 * d1 - d2 * d2 + D * D) / (2 * D);
      double px = p1.x + a * (p2.x - p1.x) / D;
      double py = p1.y + a * (p2.y - p1.y) / D;
      double h = sqrt(d1 * d1 - a * a);
      double rx = -(p2.y - p1.y) * (h / D);
      double ry = (p2.x - p1.x) * (h / D);

      double xA = px + rx, yA = py + ry;
      double xB = px - rx, yB = py - ry;

      // Pick solution closer to previous estimate
      if (hypot(xA - x_prev, yA - y_prev) < hypot(xB - x_prev, yB - y_prev)) {
        *out_x = xA;
        *out_y = yA;
      } else {
        *out_x = xB;
        *out_y = yB;
      }
    } else {
      // Least-squares trilateration (3+ reference points)
      const auto& p1 = neighbors[0];
      double d1 = neighbor_distances[0];

      // Prepare matrices for least squares
      std::vector<std::vector<double>> A(ref_count - 1, std::vector<double>(2));
      std::vector<double> b(ref_count - 1);

      for (size_t i = 1; i < ref_count; ++i) {
        const auto& pi = neighbors[i];
        double di = neighbor_distances[i];

        A[i - 1][0] = 2 * (pi.x - p1.x);
        A[i - 1][1] = 2 * (pi.y - p1.y);
        b[i - 1] = d1 * d1 - di * di + pi.x * pi.x - p1.x * p1.x + pi.y * pi.y - p1.y * p1.y;
      }

      // Compute A^T*A and A^T*b
      double AtA[2][2] = { 0 }, Atb[2] = { 0 };
      for (size_t i = 0; i < ref_count - 1; ++i) {
        AtA[0][0] += A[i][0] * A[i][0];
        AtA[0][1] += A[i][0] * A[i][1];
        AtA[1][0] += A[i][1] * A[i][0];
        AtA[1][1] += A[i][1] * A[i][1];

        Atb[0] += A[i][0] * b[i];
        Atb[1] += A[i][1] * b[i];
      }

      // Solve the system
      double det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
      if (fabs(det) < 1e-6) return -4;

      *out_x = (Atb[0] * AtA[1][1] - Atb[1] * AtA[0][1]) / det;
      *out_y = (Atb[1] * AtA[0][0] - Atb[0] * AtA[1][0]) / det;
    }

    return 0;
  }

  // Update target robot's position using trilateration and automatically update timestamp
  bool update_base_position(int16_t target_address, float pos[3]) {
    // First check if the target exists
    auto target_it = robots.find(target_address);
    if (target_it == robots.end()) {
      return false;  // Target robot not found
    }

    // Perform trilateration
    double x, y;
    int result = trilaterate_2D(target_address, &x, &y);

    if (result == 0) {
      // Update position and timestamp
      Robot& target = target_it->second;
      target.position[0] = x;
      target.position[1] = y;
      target.last_update_time = current_millis();
      pos[0] =x;
      pos[1] =y;

// Optionally print debug info
#ifdef SWARM_DEBUG
      std::cout << "Updated base position for robot 0x" << std::hex << target_address << std::dec
                << " to (" << x << ", " << y << ")\n";
#endif

      return true;
    } else {
// Handle different error cases if needed
#ifdef SWARM_DEBUG
      std::cerr << "Trilateration failed for robot 0x" << std::hex << target_address << std::dec
                << " with error code: " << result << "\n";
#endif
      return false;
    }
  }

  // Helper function to get current time in milliseconds
  static unsigned long current_millis() {
    // return std::chrono::system_clock::now().time_since_epoch() /
    //  std::chrono::milliseconds(1);
    return millis();
  }


  void update_robot(int16_t address, double x, double y, float new_distance) {
    // First remove any stale robots
    remove_stale_robots();

    auto it = robots.find(address);
    if (it == robots.end()) {
      // Robot doesn't exist, add it first
      add_address(address);
      it = robots.find(address);
      if (it == robots.end()) return;  // Failed to add
    }

    // Update the robot's information
    Robot& robot = it->second;
    robot.position[0] = x;
    robot.position[1] = y;
    // robot.distance = robot.filter_distance.updateFilter(new_distance);
    // robot.distance = new_distance;
    if (new_distance != -1) {
      robot.update_distance(new_distance);
    }
    robot.update_timestamp();
  }

  // Function to remove robots not seen for more than 1 minute
  void remove_stale_robots() {
    unsigned long current_time = Robot::current_millis();
    std::vector<int16_t> to_remove;

    for (const auto& pair : robots) {
      if (pair.first == base_bot_id) continue;  // ignore base address

      if (current_time - pair.second.last_update_time > STALE_TIMEOUT) {
        to_remove.push_back(pair.first);
      }
    }

    for (int16_t addr : to_remove) {
      robots.erase(addr);
#ifdef DEBUG_OUTPUT
      Serial.print("Removed stale robot: 0x");
      Serial.println(addr, HEX);
#endif
    }
  }
};


// int main() {
//     RobotSwarm swarm;

//     // Add base robot, with starting positions and distance must always be 0
//     swarm.update_robot(0xAA, 1, 0, 0); // x, y, distance

//     // This will automatically create a new robot if it doesn't exist, or update it if it has existed
//     swarm.update_robot(0xBB, 5, 0, 5);
//     swarm.update_robot(0xCC, 0, 5, 5);
//     swarm.update_robot(0xDD, 5, 5, 7.0);

//     swarm.print_Robot_Swarm();

//     // Update base position based on all info
//     swarm.update_base_position(0xAA);

//     // Print all robot information
//     swarm.print_Robot_Swarm();

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

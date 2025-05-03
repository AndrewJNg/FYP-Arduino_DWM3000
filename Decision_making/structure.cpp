#include <iostream>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <iomanip>
#include <cmath>


class Kalman {
public:
    Kalman(float mea_e, float est_e, float q) {
        setParameters(mea_e, est_e, q);
    }
    
    Kalman(float mea_e, float q) {
        setParameters(mea_e, mea_e, q);
    }
    
    void setParameters(float mea_e, float est_e, float q) {
        _err_measure = mea_e;
        _err_estimate = est_e;
        _q = q;
    }
    
    void setParameters(float mea_e, float q) {
        setParameters(mea_e, mea_e, q);
    }
    
    float updateFilter(float value) {		
        float _kalman_gain, _current_estimate;
        _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
        _current_estimate = _last_estimate + _kalman_gain * (value - _last_estimate);
        _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
        _last_estimate=_current_estimate;
        return _current_estimate;
    }

private:
    float _err_measure = 0.0;
    float _err_estimate = 0.0;
    float _q = 0.0;
    float _last_estimate = 0.0;
};


class RobotSwarm {
private:
    struct Robot {
        int16_t address;
        double position[3];       // x, y, z coordinates
        double velocity[3];       // vx, vy, vz components
        unsigned long last_sent_time;
        double distance;          // Distance measurement from this robot to the base

        
        // Kalman filters for each position dimension
        Kalman filter_distance;

        Robot(int16_t addr) : address(addr), distance(0.0), 
        filter_distance(0.02, 0.02, 0.1) {
            position[0] = position[1] = position[2] = 0.0;
            velocity[0] = velocity[1] = velocity[2] = 0.0;
            update_timestamp();
        }

        void update_timestamp() {
            last_sent_time = current_millis();
        }

        static unsigned long current_millis() {
            return std::chrono::system_clock::now().time_since_epoch() / 
                std::chrono::milliseconds(1);
        }
    };
    std::unordered_map<int16_t, Robot> robots;

    // Helper structure for trilateration
    struct Position {
        double x, y;
    };
    
public:
    // Add a new robot address to the swarm
    bool add_address(int16_t address) {
        if (robots.find(address) != robots.end()) {
            return false; // Address already exists
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
        std::cout << "\n=== Robot Swarm Status ===\n";
        std::cout << "Total robots: " << robots.size() << "\n\n";
        
        for (const auto& pair : robots) {
            const Robot& robot = pair.second;
            
            std::cout << "Robot Address: 0x" << std::hex << robot.address << std::dec << "\n";
            std::cout << "Position: [" << std::fixed << std::setprecision(2) 
                      << robot.position[0] << ", " 
                      << robot.position[1] << ", " 
                      << robot.position[2] << "]\n";
            std::cout << "Velocity: [" << std::fixed << std::setprecision(2) 
                      << robot.velocity[0] << ", " 
                      << robot.velocity[1] << ", " 
                      << robot.velocity[2] << "]\n";
            std::cout << "Last Sent Time: " << robot.last_sent_time << " ms\n";
            std::cout << "Distance: " << std::fixed << std::setprecision(2) 
                      << robot.distance << " m\n";
            std::cout << "------------------------\n";
        }
    }
    // Enhanced Trilateration function
    int trilaterate_2D(int16_t target_address, double* out_x, double* out_y) {
        auto target_it = robots.find(target_address);
        if (target_it == robots.end()) return -1; // Target not found

        std::vector<Position> neighbors;
        std::vector<double> neighbor_distances;

        // Find all robots that have distance measurements to the target
        for (const auto& pair : robots) {
            if (pair.first == target_address) continue; // Skip the target itself

            const Robot& robot = pair.second;
            if (robot.distance <= 0) continue; // Skip invalid distances

            neighbors.push_back({robot.position[0], robot.position[1]});
            neighbor_distances.push_back(robot.distance);
        }

        const size_t ref_count = neighbors.size();
        if (ref_count < 2) return -2; // Not enough reference points

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

            double a = (d1*d1 - d2*d2 + D*D) / (2 * D);
            double px = p1.x + a * (p2.x - p1.x) / D;
            double py = p1.y + a * (p2.y - p1.y) / D;
            double h = sqrt(d1*d1 - a*a);
            double rx = -(p2.y - p1.y) * (h / D);
            double ry =  (p2.x - p1.x) * (h / D);

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
            std::vector<std::vector<double>> A(ref_count-1, std::vector<double>(2));
            std::vector<double> b(ref_count-1);

            for (size_t i = 1; i < ref_count; ++i) {
                const auto& pi = neighbors[i];
                double di = neighbor_distances[i];

                A[i-1][0] = 2 * (pi.x - p1.x);
                A[i-1][1] = 2 * (pi.y - p1.y);
                b[i-1] = d1*d1 - di*di + pi.x*pi.x - p1.x*p1.x + pi.y*pi.y - p1.y*p1.y;
            }

            // Compute A^T*A and A^T*b
            double AtA[2][2] = {0}, Atb[2] = {0};
            for (size_t i = 0; i < ref_count-1; ++i) {
                AtA[0][0] += A[i][0] * A[i][0];
                AtA[0][1] += A[i][0] * A[i][1];
                AtA[1][0] += A[i][1] * A[i][0];
                AtA[1][1] += A[i][1] * A[i][1];

                Atb[0] += A[i][0] * b[i];
                Atb[1] += A[i][1] * b[i];
            }

            // Solve the system
            double det = AtA[0][0]*AtA[1][1] - AtA[0][1]*AtA[1][0];
            if (fabs(det) < 1e-6) return -4;

            *out_x = (Atb[0]*AtA[1][1] - Atb[1]*AtA[0][1]) / det;
            *out_y = (Atb[1]*AtA[0][0] - Atb[0]*AtA[1][0]) / det;
        }

        return 0;
    }

    // Update target robot's position using trilateration and automatically update timestamp
bool update_base_position(int16_t target_address) {
    // First check if the target exists
    auto target_it = robots.find(target_address);
    if (target_it == robots.end()) {
        return false; // Target robot not found
    }

    // Perform trilateration
    double x, y;
    int result = trilaterate_2D(target_address, &x, &y);
    
    if (result == 0) {
        // Update position and timestamp
        Robot& target = target_it->second;
        target.position[0] = x;
        target.position[1] = y;
        target.last_sent_time = current_millis();
        
        // Optionally print debug info
        #ifdef SWARM_DEBUG
        std::cout << "Updated base position for robot 0x" << std::hex << target_address << std::dec
                  << " to (" << x << ", " << y << ")\n";
        #endif
        
        return true;
    }
    else {
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
    return std::chrono::system_clock::now().time_since_epoch() / 
           std::chrono::milliseconds(1);
}


    void update_robot(int16_t address, double x, double y, double new_distance) {
        auto it = robots.find(address);
        if (it == robots.end()) {
            // Robot doesn't exist, add it first
            add_address(address);
            it = robots.find(address);
            if (it == robots.end()) return; // Failed to add
        }

        // Update the robot's information
        Robot& robot = it->second;
        robot.position[0] = x;
        robot.position[1] = y;
        robot.distance = robot.filter_distance.updateFilter(new_distance);
        robot.update_timestamp();
    }

};

int main() {
    RobotSwarm swarm;
    
    // Add base robot, with starting positions and distance must always be 0 
    swarm.update_robot(0xAA, 1, 0, 0); // x, y, distance

    // This will automatically create a new robot if it doesn't exist, or update it if it has existed
    swarm.update_robot(0xBB, 5, 0, 5);
    swarm.update_robot(0xCC, 0, 5, 5); 
    swarm.update_robot(0xDD, 5, 5, 7.0); 
    
    swarm.print_Robot_Swarm();

    // Update base position based on all info
    swarm.update_base_position(0xAA);

    // Print all robot information
    swarm.print_Robot_Swarm();
    
    return 0;
}
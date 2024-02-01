#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <chrono>

class RobotExplorer {
public:
    const int UNKNOWN = -1;
    const int FREE = 0;
    const int OBSTACLE = 1;
    const int FRONTIER = 2;

private:
    std::vector<std::vector<int>> grid;
    std::vector<double> position;
    int lidar_range;
    int num_rays;
    double HEADING_WEIGHT;

public:
    RobotExplorer(const std::vector<int>& map_size, const std::vector<double>& start_position, int lidar_range = 1, int num_rays = 15)
        : grid(map_size[0], std::vector<int>(map_size[1], UNKNOWN)), position(start_position), lidar_range(lidar_range), num_rays(num_rays), HEADING_WEIGHT(0.5) {
        grid[static_cast<int>(position[0])][static_cast<int>(position[1])] = FREE;
        std::cout << "Map size: " << grid.size() << ", " << grid[0].size() << std::endl;
        }

    std::vector<std::pair<int, int>> bresenham_line(int x0, int y0, int x1, int y1) {
        std::vector<std::pair<int, int>> points;
        int dx = std::abs(x1 - x0);
        int dy = -std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            points.push_back({x0, y0});
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
        return points;
    }

    std::vector<int> ray_casting(double angle) {
        int x = static_cast<int>(position[0]);
        int y = static_cast<int>(position[1]);
        int x_end = static_cast<int>(x + lidar_range * std::cos(angle));
        int y_end = static_cast<int>(y + lidar_range * std::sin(angle));
        std::vector<int> last_point = {x, y, 0};

        for (const auto& point : bresenham_line(x, y, x_end, y_end)) {
            int x_curr = point.first;
            int y_curr = point.second;
            
            if (x_curr >= 0 && x_curr < grid.size() && y_curr >= 0 && y_curr < grid[0].size()) {
                if (grid[x_curr][y_curr] == OBSTACLE) {
                    return {x_curr, y_curr, 1};
                }
                grid[x_curr][y_curr] = FREE;
                last_point = {x_curr, y_curr, 0};
            }
        }
        // std::cout << "Made it! x_end: " << x_end << ", y_end: " << y_end << std::endl;
        return last_point;
    }

    void update_map() {
        for (int i = 0; i < num_rays; ++i) {
            double angle = position[2] + i * 2 * M_PI / num_rays;
            std::vector<int> end_point = ray_casting(angle);
            if (end_point[2] == 1)
                grid[end_point[0]][end_point[1]] = OBSTACLE;
        }
    }

    void get_frontier_points(std::vector<std::pair<int, int>>& frontier_points) {
        for (int i = 0; i < grid.size(); ++i) {
            for (int j = 0; j < grid[0].size(); ++j) {
                if (grid[i][j] == FREE) {
                    for (int k = 0; k < 2; k ++) {
                        for (int l = 0; l < 2; l ++) {
                            int x = i + k;
                            int y = j + l;
                            if (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == FREE) {
                                frontier_points.push_back({i, j});
                            }
                        }
                    }
                }
            }
        }
    }

    void get_frontier_centroids(std::vector<std::pair<int, int>>& frontier_points, std::vector<std::pair<int, int>>& frontier_centroids) {
        int num_frontier_points = 0;
        for (const auto& point : frontier_points) {
            int x = point.first;
            int y = point.second;
            int x_sum = 0;
            int y_sum = 0;
            for (int i = std::max(1, x - 1); i <= std::min(static_cast<int>(grid.size()) - 2, x + 1); ++i) {
                for (int j = std::max(1, y - 1); j <= std::min(static_cast<int>(grid[0].size()) - 2, y + 1); ++j) {
                    if (grid[i][j] == FREE && (grid[i-1][j] == UNKNOWN || grid[i+1][j] == UNKNOWN || grid[i][j-1] == UNKNOWN || grid[i][j+1] == UNKNOWN)) {
                        ++num_frontier_points;
                        x_sum += i;
                        y_sum += j;
                    }
                }
            }
            if (num_frontier_points > 0) {
                frontier_centroids.push_back({x_sum / num_frontier_points, y_sum / num_frontier_points});
            }
        }
        std::cout << "Num frontier points: " << num_frontier_points << std::endl;
    }

    void select_frontier(std::vector<std::pair<int, int>>& frontier_points, std::pair<int, int>& selected_frontier) {
        double max_distance = 0.0;
        for (const auto& centroid : frontier_points) {
            double distance = std::sqrt(std::pow(position[0] - centroid.first, 2) + std::pow(position[1] - centroid.second, 2));
            if (distance > max_distance) {
                max_distance = distance;
                selected_frontier = centroid;
            }
        }
        std::cout << "Selected frontier: " << selected_frontier.first << ", " << selected_frontier.second << std::endl;
    }

    void compute_potential_field(std::pair<int, int>& selected_frontier, std::vector<double>& potential_field) {
        std::pair<double, double> dist_diff = std::pair<double, double>(position[0] - selected_frontier.first, position[1] - selected_frontier.second);
        double norm = std::sqrt(dist_diff.first * dist_diff.first + dist_diff.second * dist_diff.second) + 1e-6;
        std::pair<double, double> U_attractive = std::pair<double,double>(dist_diff.first / norm, dist_diff.second / norm);
        // find coordinates where grid is an obstacle within a certain radius
        std::vector<std::pair<int, int>> obstacle_coords;
        for (int i = std::max(0, static_cast<int>(position[0]) - lidar_range); i <= std::min(static_cast<int>(grid.size()) - 1, static_cast<int>(position[0]) + lidar_range); ++i) {
            for (int j = std::max(0, static_cast<int>(position[1]) - lidar_range); j <= std::min(static_cast<int>(grid[0].size()) - 1, static_cast<int>(position[1]) + lidar_range); ++j) {
                if (grid[i][j] == OBSTACLE) {
                    obstacle_coords.push_back({i, j});
                }
            }
        }
        // compute repulsive potential field
        std::pair<double, double> U_repulsive = std::pair<double, double>(0.0, 0.0);
        for (const auto& coord : obstacle_coords) {
            std::pair<double, double> dist_diff = std::pair<double, double>(position[0] - coord.first, position[1] - coord.second);
            double norm = std::sqrt(dist_diff.first * dist_diff.first + dist_diff.second * dist_diff.second) + 1e-6;
            if (norm < lidar_range) {
                U_repulsive.first += dist_diff.first / norm;
                U_repulsive.second += dist_diff.second / norm;
            }
        }
        // combine attractive and repulsive potential fields

        potential_field = {U_attractive.first + U_repulsive.first, U_attractive.second + U_repulsive.second};
    }

    void move_towards_goal(std::vector<double>& potential_field) {
        //move position towards goal using differential drive 
        std::cout << "Old Position: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
        double heading = std::atan2(potential_field[1], potential_field[0]);
        double heading_change = std::fmod((heading - position[2] + M_PI), (2 * M_PI - M_PI));

        double vel = std::cos(heading_change);
        if (vel>2){
            vel = 2;
        }
        double x = position[0] + std::cos(position[2]) * vel;
        double y = position[1] + std::sin(position[2]) * vel;
        position = {x, y, heading};
        std::cout << "New Position: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
    }

    void explore() {
        int count = 0;
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            std::cout << "Exploring..." << std::endl;
            update_map();
            std::cout << "Map updated!" << std::endl;
            std::vector<std::pair<int, int>> frontier_points;
            get_frontier_points(frontier_points);
            if (frontier_points.empty()) {
                std::cout << "No more frontiers to explore!" << std::endl;
                break;
            }
            std::vector<std::pair<int, int>> frontier_centroids;
            get_frontier_centroids(frontier_points, frontier_centroids);
            std::pair<int, int> selected_frontier;
            select_frontier(frontier_points, selected_frontier);
            std::vector<double> potential_field;
            compute_potential_field(selected_frontier, potential_field);
            move_towards_goal(potential_field);
            count += 1;
            auto stop = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(stop-start).count();
            std::cout << "----------------------------------------" << std::endl;
            std::cout << "Position: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
            std::cout << "Selected frontier: " << selected_frontier.first << ", " << selected_frontier.second << std::endl;
            std::cout << "Time taken by function: " << elapsed_time_ms << " milliseconds" << std::endl;
            std::cout << "Number of iterations: " << count << std::endl;
            std::cout << "----------------------------------------" << std::endl;
            // print grid on each iteration
            for (int i = 0; i < grid.size(); ++i) {
                for (int j = 0; j < grid[0].size(); ++j) {
                    std::cout << grid[i][j] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "----------------------------------------" << std::endl;
        }
    }
};

// Main function for demonstration purposes
int main() {
    // Set up a map size and starting position for the robot
    std::vector<int> map_size = {15, 15};
    std::vector<double> start_position = {5.0, 5.0, 0.0}; // x, y, and heading
    RobotExplorer robot(map_size, start_position);
    robot.explore();
    // Simulate the robot's behavior
    // This would include calling the other member functions and simulating behavior over time

    return 0;
}


#include <vector>
#include <array>

// Define array for goal position
std::array<float, 2> goal_pos = {10, 10};

// Define arrays for obstacles and their velocities
std::vector<std::array<float, 2>> obstacles = {{obst1.x, obst1.y},
                                                {obst2.x, obst2.y},
                                                {obst3.x, obst3.y},
                                                {obst4.x, obst4.y},
                                                {obst5.x, obst5.y},
                                                {obst6.x, obst6.y}};

// Define obstacle radii
std::vector<float> obstacles_radius = {0.4, 0.4, 0.4, 0.25, 0.25, 0.25};

// Define obstacle velocities
std::vector<std::array<float, 2>> obstacle_velocities = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};

// Define robot position
std::array<float, 2> robot_pos = {x, y};

// Define other variables
float epsilon = 60000;
float Nd = 20000;
float Ns = 3000000;
float Ne = 40000;
float tau = 0.3;
float Ros = 0.5;
float Dsafe = 1;
float phou0 = 5;

// Define velocity of some object (vos)
std::array<float, 2> vos_velocity = {v_husky.x, v_husky.y};

// ... code to get x, y, v_husky, etc. from msg
float x = msg.pose.pose.position.x;
float y = msg.pose.pose.position.y;
// ... (conversion of quaternion to Euler angles would be different in C++)

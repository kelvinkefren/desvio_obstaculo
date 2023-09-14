#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using namespace std;

struct Entity {
    geometry_msgs::msg::Pose pose;  // Posição
    geometry_msgs::msg::Twist velocity;  // Velocidade
};

// Função para calcular a distância entre o robô e um ponto
float distance(geometry_msgs::msg::Pose robot, float goalX, float goalY) {
    return sqrt(pow(robot.position.x - goalX, 2) + pow(robot.position.y - goalY, 2));
}

// Função para calcular a força resultante usando o método de campo potencial
geometry_msgs::msg::Wrench calculateResultantForce(
    Entity robot, geometry_msgs::msg::Pose goal,
    vector<Entity> obstacles,
    float epsilon, float Nd, float Ns, float Ne, float tau,
    float Ros, float Dsafe, float phou0
) {
    geometry_msgs::msg::Wrench resultant_force;

    float goalX = goal.position.x;
    float goalY = goal.position.y;

    // Inicializar força resultante
    resultant_force.force.x = 0.0;
    resultant_force.force.y = 0.0;

    // Cálculo de força de atração para o objetivo
    // Implemente aqui seu próprio modelo baseado no algoritmo de campo potencial

    // Cálculo de forças de repulsão dos obstáculos
    for (const auto& obs : obstacles) {
        float d = distance(robot.pose, obs.pose.position.x, obs.pose.position.y);

        // Implemente aqui seu próprio modelo de força de repulsão,
        // usando os parâmetros de comportamento passados como argumento
    }

    return resultant_force;
}

int main() {
    // Criar instâncias do robô e obstáculos
    Entity robot;
    robot.pose.position.x = 0.0;
    robot.pose.position.y = 0.0;
    robot.velocity.linear.x = 0.0;
    robot.velocity.linear.y = 0.0;

    geometry_msgs::msg::Pose goal;
    goal.position.x = 10.0;
    goal.position.y = 10.0;

    vector<Entity> obstacles;
    Entity obs1;
    obs1.pose.position.x = 2.0;
    obs1.pose.position.y = 2.0;
    obs1.velocity.linear.x = 0.0;
    obs1.velocity.linear.y = 0.0;
    obstacles.push_back(obs1);

    Entity obs2;
    obs2.pose.position.x = 4.0;
    obs2.pose.position.y = 4.0;
    obs2.velocity.linear.x = 0.0;
    obs2.velocity.linear.y = 0.0;
    obstacles.push_back(obs2);

    // Parâmetros que podem variar para cada situação
    float epsilon = 60000;
    float Nd = 20000;
    float Ns = 3000000;
    float Ne = 40000;
    float tau = 0.3;
    float Ros = 0.5;
    float Dsafe = 1;
    float phou0 = 5;

    // Calcular força resultante
    geometry_msgs::msg::Wrench force = calculateResultantForce(
        robot, goal, obstacles, epsilon, Nd, Ns, Ne, tau, Ros, Dsafe, phou0
    );

    // Mostrar a força resultante
    cout << "Força resultante: (" << force.force.x << ", " << force.force.y << ")" << endl;

    return 0;
}

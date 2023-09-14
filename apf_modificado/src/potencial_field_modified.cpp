#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "Eigen/Dense"

using namespace std;
using Eigen::Vector2f;

struct Entity {
    geometry_msgs::msg::Pose pose;  // Posição
    geometry_msgs::msg::Twist velocity;  // Velocidade
};

// Função para calcular a distância entre o robô e um ponto
float distance(geometry_msgs::msg::Pose robot, float goalX, float goalY) {
    return sqrt(pow(robot.position.x - goalX, 2) + pow(robot.position.y - goalY, 2));
}

geometry_msgs::msg::Wrench calculateResultantForce(
    Entity robot, geometry_msgs::msg::Pose goal,
    vector<Entity> obstacles,
    float epsilon, float Nd, float Ns, float Ne, float tau,
    float Ros, float Dsafe, float phou0
) {
    // Inicializando a força resultante
    geometry_msgs::msg::Wrench resultant_force;
    resultant_force.force.x = 0.0;
    resultant_force.force.y = 0.0;

    // Coordenadas do objetivo e do robô
    float goalX = goal.position.x;
    float goalY = goal.position.y;

    float robotX = robot.pose.position.x;
    float robotY = robot.pose.position.y;

    // Vetor para o objetivo e sua magnitude
    Vector2f pg(goalX - robotX, goalY - robotY);
    float dg = pg.norm();

    // Vetor unitário na direção do objetivo
    Vector2f Nog = pg / dg;

    // Força atrativa (usando seu próprio modelo)
    Vector2f Fatt(0, 0);
    Fatt = modified_attractive_force(...);

    // Força repulsiva (inicializando)
    Vector2f Frep(0, 0);

    // Loop pelos obstáculos para calcular as forças repulsivas    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        Entity obstacle = obstacles[i];
    
        Vector2f Frd(0, 0);
        Vector2f Frs(0, 0);
        Vector2f Fre(0, 0);
    
        float d, Dm, CR, tetha;
        Vector2f Pot, Vto, Not, Notperp;
        float theta_m_radian, AngleDiff, Rts;
    
        std::tie(d, Dm, CR, Pot, theta_m_radian, Vto, tetha, Not, AngleDiff, Notperp, Rts) = iniciation(
            obstacle, robot, vos_velocity, obstacle_velocity, obstacle_radius, Dsafe, phou0, Ros, Nog
        );
    
        if (d <= CR && tetha < theta_m_radian && Dm <= d) {
            if (obstacle.velocity.linear.x != 0.0 || obstacle.velocity.linear.y != 0.0) {  // Dynamic obstacle
                Frd = calculate_Frd(d, Dm, phou0, tetha, Vto, theta_m_radian, AngleDiff, Pot, Nd, Rts, dg, Not, Notperp, Nog);
                std::cout << "d = " << d << " < CR = " << CR << " dynamic --------------  Frd = " << Frd.transpose() << std::endl;
            } else {  // Static obstacle
                Frs = calculate_Frs(d, tau, dg, phou0, Ns, Rts, Not, Nog);
                std::cout << "d = " << d << " < CR = " << CR << " static --------------  Frs = " << Frs.transpose() << std::endl;
            }
        } else {
            if (d < Dm) {
                Fre = calculate_Fre(d, tau, Dm, dg, Ne, Rts, Not, Vto, tetha, Nog, Notperp);
                std::cout << "d = " << d << " < Dm = " << Dm << " --------------  Fre = " << Fre.transpose() << std::endl;
            } else {
                std::cout << " " << std::endl;
            }
        }
    
        Frep += Frd + Frs + Fre;
        std::cout << "Obstacle number: " << i << " Frep = " << Frep.transpose() << " tetha = " << tetha << " theta_m = " << theta_m_radian << std::endl;
    }    

    // Força total
    Vector2f F_total = Fatt + Frep;

    // Atribuindo à força resultante
    resultant_force.force.x = F_total[0];
    resultant_force.force.y = F_total[1];

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

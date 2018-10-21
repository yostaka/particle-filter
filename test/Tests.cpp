//
// Created by Yosuke Takahashi on 10/14/18.
//

#include <gtest/gtest.h>
#include "../src/particle_filter.h"
#include <Eigen/Dense>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

TEST(ParticleFilter, SimpleTest) {
    ASSERT_EQ(2, 2);
}

TEST(ParticleFilter, prediction) {
    std::vector<Particle> particles;

    Particle p;

    p.x = 102;
    p.y = 65;
    p.theta = 5*M_PI/8;

    double velocity = 110.0;
    double yaw_rate = M_PI/8;
    double delta_t = 0.1;

    double sigma_pos[3] = {0.0, 0.0, 0.0}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]

    particles.push_back(p);

    ParticleFilter pf;
    pf.setParticles(particles);

    pf.prediction(delta_t, sigma_pos, velocity, yaw_rate);

    EXPECT_NEAR(pf.particles[0].x, 97.59, 0.01);
    EXPECT_NEAR(pf.particles[0].y, 75.08, 0.01);
    EXPECT_NEAR(pf.particles[0].theta, 51*M_PI/80, 0.01);
}

TEST(ParticleFilter, transformObservationCoordinateVehicleToMap_1) {
    ParticleFilter pf;

    Particle p;
    LandmarkObs obs;

    p.x = 4;
    p.y = 5;
    p.theta = -M_PI/2;
    obs.x = 2;
    obs.y = 2;

    LandmarkObs map;

    map = pf.transformObservationCoordinateVehicleToMap(p, obs);

    EXPECT_NEAR(map.x, 6.0, 0.1);
    EXPECT_NEAR(map.y, 3.0, 0.1);
}

TEST(ParticleFilter, transformObservationCoordinateVehicleToMap_2) {
    ParticleFilter pf;

    Particle p;
    LandmarkObs obs;

    p.x = 4;
    p.y = 5;
    p.theta = -M_PI/2;
    obs.x = 3;
    obs.y = -2;

    LandmarkObs map;

    map = pf.transformObservationCoordinateVehicleToMap(p, obs);

    EXPECT_NEAR(map.x, 2.0, 0.1);
    EXPECT_NEAR(map.y, 2.0, 0.1);
}

TEST(ParticleFilter, getNearestLandmark) {
    ParticleFilter pf;

    Particle p;
    p.x = 4;
    p.y = 5;
    p.theta = -M_PI/2;

    Map map;
    Map::single_landmark_s l1, l2, l3, l4, l5;

    l1.id_i = 1;
    l1.x_f  = 5;
    l1.y_f  = 3;
    map.landmark_list.push_back(l1);

    l2.id_i = 2;
    l2.x_f  = 2;
    l2.y_f  = 1;
    map.landmark_list.push_back(l2);

    l3.id_i = 3;
    l3.x_f  = 6;
    l3.y_f  = 1;
    map.landmark_list.push_back(l3);

    l4.id_i = 4;
    l4.x_f  = 7;
    l4.y_f  = 4;
    map.landmark_list.push_back(l4);

    l5.id_i = 5;
    l5.x_f  = 4;
    l5.y_f  = 7;
    map.landmark_list.push_back(l5);

    LandmarkObs obs1, obs2, obs3;
    obs1.x = 2;
    obs1.y = 2;

    obs2.x = 3;
    obs2.y = -2;

    obs3.x = 0;
    obs3.y = -4;

    LandmarkObs obs_map1, obs_map2, obs_map3;
    obs_map1 = pf.transformObservationCoordinateVehicleToMap(p, obs1);
    obs_map2 = pf.transformObservationCoordinateVehicleToMap(p, obs2);
    obs_map3 = pf.transformObservationCoordinateVehicleToMap(p, obs3);

    LandmarkObs nearest_obs1, nearest_obs2, nearest_obs3;
    nearest_obs1 = pf.getNearestLandmark(obs_map1, map);
    nearest_obs2 = pf.getNearestLandmark(obs_map2, map);
    nearest_obs3 = pf.getNearestLandmark(obs_map3, map);

    EXPECT_EQ(nearest_obs1.id, 1);
    EXPECT_EQ(nearest_obs2.id, 2);
    EXPECT_EQ(nearest_obs3.id, 2);

    double weight1, weight2, weight3;
    double std_landmark[3] = {0.3, 0.3, 0.0};
    weight1 = pf.getObservationWeight(obs_map1, nearest_obs1, std_landmark);
    weight2 = pf.getObservationWeight(obs_map2, nearest_obs2, std_landmark);
    weight3 = pf.getObservationWeight(obs_map3, nearest_obs3, std_landmark);

    EXPECT_NEAR(weight1, 6.84e-3, 0.01e-3);
    EXPECT_NEAR(weight2, 6.84e-3, 0.01e-3);
    EXPECT_NEAR(weight3, 9.83e-49, 0.01e-49);
}

TEST(ParticleFilter, getParticleWeight) {
    ParticleFilter pf;

    Particle p;
    p.x = 4;
    p.y = 5;
    p.theta = -M_PI/2;

    Map map;
    Map::single_landmark_s l1, l2, l3, l4, l5;

    l1.id_i = 1;
    l1.x_f  = 5;
    l1.y_f  = 3;
    map.landmark_list.push_back(l1);

    l2.id_i = 2;
    l2.x_f  = 2;
    l2.y_f  = 1;
    map.landmark_list.push_back(l2);

    l3.id_i = 3;
    l3.x_f  = 6;
    l3.y_f  = 1;
    map.landmark_list.push_back(l3);

    l4.id_i = 4;
    l4.x_f  = 7;
    l4.y_f  = 4;
    map.landmark_list.push_back(l4);

    l5.id_i = 5;
    l5.x_f  = 4;
    l5.y_f  = 7;
    map.landmark_list.push_back(l5);

    LandmarkObs obs1, obs2, obs3;
    obs1.x = 2;
    obs1.y = 2;

    obs2.x = 3;
    obs2.y = -2;

    obs3.x = 0;
    obs3.y = -4;

    std::vector<LandmarkObs> observations;
    observations.push_back(obs1);
    observations.push_back(obs2);
    observations.push_back(obs3);

    double std_landmark[3] = {0.3, 0.3, 0.0};
    double weight = pf.getParticleWeight(p, std_landmark, observations, map);

    EXPECT_NEAR(weight, 4.60e-53, 0.01e-53);
}


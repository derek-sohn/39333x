#include "main.h"
#include "robot.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

const int N_PARTICLES = 300;
const double FIELD_WIDTH = 3650.0;
const double FIELD_LENGTH = 3650.0;
const double SENSOR_NOISE_STD = 50.0;
const double ODOMETRY_NOISE_STD = 5.0;
const double ROTATION_NOISE_STD = 1.0;

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

std::vector<Particle> particles;
std:: default_random_engine rng;

double prev_left_enc = 0.0;
double prev_right_enc = 0.0;
double prev_imu_heading = 0.0;

void initilizePArticles(double x0=0, double y0=0, double theta0=0, bool useGuess=false){
    std::uniform_real_distribution<double> distX(0, FIELD_WIDTH);
    std::uniform_real_distribution<double> distY(0, FIELD_LENGTH);
    std::uniform_real_distribution<double> distTheta(0, 360);
    
    std::normal_distribution<double> gaussX(x0, FIELD_WIDTH/10.0);
    std::normal_distribution<double> gaussY(y0, FIELD_LENGTH/10.0);
    std::normal_distribution<double> gaussTheta(theta0, 15.0);
    particles.resize(N_PARTICLES);
    for (int i = 0; i < N_PARTICLES; ++i){
        if (useGuess) {
            particles[i].x = gaussX(rng);
            particles[i].y = gaussY(rng);
            particles[i].theta = gaussTheta(rng);
        } else {
            particles[i].x = distX(rng);
            particles[i].y = distY(rng);
            particles[i].theta = distTheta(rng);
        }
        if (particles[i].x < 0) particles[i].x = 0;
        if (particles[i].x > FIELD_WIDTH) particles[i].x = FIELD_WIDTH;
        if (particles[i].y < 0) particles[i].y = 0;
        if (particles[i].y > FIELD_WIDTH) particles[i].y = FIELD_LENGTH;
        particles[i].weight = 1.0 / N_PARTICLES;
    }
}

double left_enc_deg = 0;
double right_enc_deg = 0;
double left_enc_mm = 0;
double right_enc_mm = 0;
double imu_heading = 0;
void updateParticlesWithMotion() {
    prev_left_enc = left_enc_mm;
    prev_right_enc = right_enc_mm;
    prev_imu_heading = imu_heading;

    left_enc_deg = 0; // put left motor position later
    right_enc_deg = 0; //put right motor position later
    left_enc_mm = left_enc_deg / 360.0 * (/* wheel circumference mm  */ 320.0);
    right_enc_mm = right_enc_deg / 360.0 * (/* wheel circumference mm  */ 320.0);
    imu_heading = imu.get_rotation();

    double delta_left = left_enc_mm - prev_left_enc;
    double delta_right = right_enc_mm - prev_right_enc;
    double delta_heading = imu_heading - prev_imu_heading;
    if (delta_heading > 180) delta_heading -= 360;
    if (delta_heading < 180) delta_heading += 360;
    double delta_d = (delta_left + delta_right) / 2.0;

    std::normal_distribution<double> transNoise(0.0, ODOMETRY_NOISE_STD);
    std::normal_distribution<double> rotNoise(0.0, ROTATION_NOISE_STD);

    for (Particle& p : particles) {
        double noisy_d = delta_d + transNoise(rng);
        double noisy_theta = delta_heading + rotNoise(rng);
        p.theta += noisy_theta;
        if (p.theta < 0) p.theta += 360;
        if (p.theta >= 360) p.theta -= 360;
        double theta_rad = p.theta * M_PI / 180.0;
        p.x += noisy_d * cos(theta_rad);
        p.x += noisy_d * sin(theta_rad);
        if (p.x < 0) p.x = 0;
        if (p.x > FIELD_WIDTH) p.x = FIELD_WIDTH;
        if (p.y < 0) p.y = 0;
        if (p.y > FIELD_LENGTH) p.y = FIELD_LENGTH;
    }

}

void updateParticlesWithSensor() {
    int32_t z_left = DistanceLeft.get();
    int32_t z_right = DistanceRight.get();
    if (z_left < 0) z_left = FIELD_WIDTH;
    if (z_right < 0) z_right = FIELD_WIDTH;

    double weight_sum = 0.0;
    double var = SENSOR_NOISE_STD * SENSOR_NOISE_STD;
    for (Particle& p : particles) {
        //expected distances to walls based on particle position
        double pred_left = p.x;
        double pred_right = FIELD_WIDTH - p.x;
        //if using orientation, adjust predicted distances
        double err_left = z_left - pred_left;
        double err_right = z_left - pred_right;
        //likelihood assuming Gaussian noise
        double w_left = exp(-(err_left * err_left) / (2 * var));
        double w_right = exp(-(err_right * err_right) / (2 * var));

        p.weight = w_left * w_right;
        weight_sum += p.weight;
    }
    //normalize weights
    if (weight_sum > 1e-9) {
        for (Particle& p : particles) {
            p.weight /= weight_sum;
        }
    } else {
        //if total weight is 0 (all particles have 0 likelihood) reinitialize or assign equal weight
        for (Particle& p : particles) {
            p.weight = 1.0 / N_PARTICLES;
        }
    }

}

// resample particles based on their weights (low varience resampling)
void resampleParticles() {
    std::vector<Particle> newParticles;
    newParticles.resize(N_PARTICLES);
    // calculate cumulative distribution of weights
    std::vector<double> cumw(N_PARTICLES);;
    double cum = 0;
    for (int i = 0; i < N_PARTICLES; ++i) {
        cum += particles[i].weight;
        cumw[i] = cum;
    }
    //ensure last cumw = 1 (normalized)
    if (cumw.back() == 0) {
        for (int i = 0; i < N_PARTICLES; ++i) {
            cumw[i] = (i+1) / (double)N_PARTICLES;
        }
    }

    std::uniform_real_distribution<double> dist(0.0, 1.0 / N_PARTICLES);
    double start = dist(rng);
    double step = 1.0 / N_PARTICLES;
    double ptr = start;
    int j = 0;
    for (int i = 0; i < N_PARTICLES; ++i) {
        while (j < N_PARTICLES && cumw[j] < ptr) {
            j++;
        }
        if (j == N_PARTICLES) j = N_PARTICLES - 1;
        newParticles[i] = particles[j];
        newParticles[i].weight = 1.0 / N_PARTICLES;
        ptr += step;
    }
    particles = newParticles;


    //(optional) add random particles or noise to particles for robustness
    //e.g., add one random particle each time for kidnapped robot recovery (if brain can handle it)
    std::uniform_real_distribution<double> distX(0, FIELD_WIDTH);
    std::uniform_real_distribution<double> distY(0, FIELD_LENGTH);
    std::uniform_real_distribution<double> distTheta(0, 360);
    particles[0].x = distX(rng);
    particles[0].y = distY(rng);
    particles[0].theta = distTheta(rng);
    particles[0].weight = 1.0 / N_PARTICLES;
}

Particle getEstimatedPose() {
    Particle estimate;
    double mean_x = 0;
    double mean_y = 0;
    double mean_sin = 0;
    double mean_cos = 0;
    for (const Particle& p : particles) {
        mean_x += p.x * p.weight;
        mean_y += p.y * p.weight;
        mean_cos += cos(p.theta * M_PI / 180.0) * p.weight;
        mean_sin += sin(p.theta * M_PI / 180.0) * p.weight;
    }
    estimate.x = mean_x;
    estimate.y = mean_y;
    estimate.theta = atan2(mean_sin, mean_cos) * 180.0 / M_PI;
    if (estimate. theta < 0) estimate.theta += 360;
    estimate.weight = 1.0;
    return estimate;
}

extern void updateParticlesWithMotion();
extern void updateParticlesWithSensor();
extern void resampleParticles();
extern void initializeParticles(double x0=0, double y0=0, double theta0=0, bool useGuess = false);
struct Particle{double x; double y; double theta; double weight;};
extern Particle getEstimatedPose();

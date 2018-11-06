#include "Particles.hpp"

class ParticleFunc {
public:
	update_acceleration(Particle p, Eigen::Ref<RealVector3> force);
	std::vector<Real> update_density(std::vector<std::vector<Particle>>& neighbors_of_set, std::vector<Particle>& particles );
	update_position();
	update_velocity();
private:
	pressure_force(Particle p);
};

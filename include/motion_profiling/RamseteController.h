#include "ARMS/api.h"
#include "ARMS/config.h"

class RamseteController {
    public:
        typedef struct {
            double linVel, angVel;
        } output;

        /**
         * b and ζ are tuning parameters where b > 0 and ζ ∈ (0, 1). Larger values of
         * b make convergence more aggressive (like a proportional term), and larger
         * values of ζ provide more damping
         */
        void setGains(double ibeta, double izeta);
        void setTarget(double x, double y, double itheta, double ivel, double iomega); // in inch, in inch, in rad, in inch/s, in rad/s
	    output step(arms::Point point, double itheta); // in inch, in inch, in degree

    private:
	    double beta, zeta, desX, desY, desT, velDes, omegaDes;
};
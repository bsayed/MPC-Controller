#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


typedef struct Result_ {
  vector<double> xs;
  vector<double> ys;
  vector<double> deltas;
  vector<double> accels;
} Result;

class MPC {
 public:
  MPC();
  int timestepMSec;

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */

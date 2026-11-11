// Thin C ABI around the unmodified ROS-Industrial STOMP optimizer.
// Task-specific noise, costs, limits and update filtering are supplied here.
#include <stomp/stomp.h>
#include <console_bridge/console.h>
#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <exception>
#include <algorithm>

using Clock = std::chrono::steady_clock;
using Callback = int (*)(const double*, int, double*, int*);

class DrakeTask : public stomp::Task {
 public:
  Callback callback;
  Eigen::MatrixXd chol, smooth;
  Eigen::VectorXd lower, upper, sigma;
  std::mt19937 rng;
  std::normal_distribution<double> gaussian{0., 1.};
  Clock::time_point start;
  double budget;
  int iterations = 0;
  bool timed_out = false;
  DrakeTask(int n, int dimensions, unsigned seed, double timeout, const double* noise, Callback cb,
            const double* lo, const double* hi)
      : callback(cb), lower(Eigen::Map<const Eigen::VectorXd>(lo,dimensions)),
        upper(Eigen::Map<const Eigen::VectorXd>(hi,dimensions)),
        sigma(Eigen::Map<const Eigen::VectorXd>(noise,dimensions)), rng(seed),
        start(Clock::now()), budget(timeout) {
    int m = n-2;
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(n,m);
    // Fixed endpoints: covariance is inverse squared-acceleration precision.
    for(int i=0;i<n;i++) for(int j=0;j<m;j++) {
      if(i==j+1) a(i,j)=-2.;
      if(i==j || i==j+2) a(i,j)=1.;
    }
    Eigen::MatrixXd cov = (a.transpose()*a).inverse();
    cov /= cov.diagonal().maxCoeff();
    chol = cov.llt().matrixL();
    smooth = cov;
    for(int j=0;j<m;j++) smooth.col(j) /= m*cov.col(j).maxCoeff();
  }
  bool expired() {
    timed_out = std::chrono::duration<double>(Clock::now()-start).count() >= budget;
    return timed_out;
  }
  bool generateNoisyParameters(const Eigen::MatrixXd& p, std::size_t,
       std::size_t, int, int, Eigen::MatrixXd& noisy, Eigen::MatrixXd& noise) override {
    if(expired()) return false;
    noise.setZero(p.rows(),p.cols());
    for(int d=0;d<p.rows();d++) {
      Eigen::VectorXd z(p.cols()-2);
      for(int t=0;t<z.size();t++) z(t)=gaussian(rng);
      noise.row(d).segment(1,z.size()) = (sigma(d)*chol*z).transpose();
    }
    noisy = p+noise;
    for(int d=0;d<p.rows();d++) noisy.row(d)=noisy.row(d).cwiseMax(lower(d)).cwiseMin(upper(d));
    noise=noisy-p;
    return true;
  }
  bool costs(const Eigen::MatrixXd& p, Eigen::VectorXd& c, bool& valid) {
    if(expired()) return false;
    c.resize(p.cols());
    int v=0;
    bool ok=callback(p.data(),p.cols(),c.data(),&v)==1;
    valid=v;
    return ok;
  }
  bool computeNoisyCosts(const Eigen::MatrixXd& p,std::size_t,std::size_t,
       int,int,Eigen::VectorXd& c,bool& v) override { return costs(p,c,v); }
  bool computeCosts(const Eigen::MatrixXd& p,std::size_t,std::size_t,
       int,Eigen::VectorXd& c,bool& v) override { return costs(p,c,v); }
  bool filterParameterUpdates(std::size_t,std::size_t,int,
       const Eigen::MatrixXd& p,Eigen::MatrixXd& u) override {
    for(int d=0;d<p.rows();d++) {
      u.row(d).segment(1,p.cols()-2) =
        (smooth*u.row(d).segment(1,p.cols()-2).transpose()).transpose().eval();
      for(int t=1;t<p.cols()-1;t++)
        u(d,t)=std::clamp(p(d,t)+u(d,t),lower(d),upper(d))-p(d,t);
    }
    u.col(0).setZero(); u.col(p.cols()-1).setZero();
    return true;
  }
  void postIteration(std::size_t,std::size_t,int i,double,const Eigen::MatrixXd&) override {
    iterations=i;
  }
};

extern "C" int run_stomp_nd(const double* initial, int dimensions, int n, int max_iterations,
    int rollouts, unsigned seed, double timeout, const double* sigma,
    const double* lower, const double* upper, Callback callback,
    double* output, int* iterations, int* timed_out) {
  try {
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_NONE);
    auto task=std::make_shared<DrakeTask>(n,dimensions,seed,timeout,sigma,callback,lower,upper);
    stomp::StompConfiguration c{};
    c.num_timesteps=n; c.num_dimensions=dimensions; c.num_iterations=max_iterations;
    c.num_iterations_after_valid=0; c.delta_t=1.0/(n-1);
    c.initialization_method=stomp::TrajectoryInitializations::LINEAR_INTERPOLATION;
    c.exponentiated_cost_sensitivity=10.; c.num_rollouts=rollouts;
    c.max_rollouts=rollouts; c.control_cost_weight=0.;
    stomp::Stomp optimizer(c,task);
    Eigen::MatrixXd result=Eigen::Map<const Eigen::MatrixXd>(initial,dimensions,n);
    bool success=optimizer.solve(result.eval(),result);
    Eigen::Map<Eigen::MatrixXd>(output,dimensions,n)=result;
    *iterations=task->iterations; *timed_out=task->timed_out;
    return success ? 1 : 0;
  } catch(const std::exception&) { return -1; }
}

// Preserve the original six-joint benchmark ABI and its scalar noise setting.
extern "C" int run_stomp(const double* initial, int n, int max_iterations,
    int rollouts, unsigned seed, double timeout, double sigma,
    const double* lower, const double* upper, Callback callback,
    double* output, int* iterations, int* timed_out) {
  double scales[6] = {sigma,sigma,sigma,sigma,sigma,sigma};
  return run_stomp_nd(initial,6,n,max_iterations,rollouts,seed,timeout,scales,
                      lower,upper,callback,output,iterations,timed_out);
}

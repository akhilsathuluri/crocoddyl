///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2023, LAAS-CNRS, New York University, Max Planck
// Gesellschaft
//                          University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include "unittest_common.hpp"
#include "crocoddyl/core/activations/quadratic-barrier.hpp"

using namespace boost::unit_test;
using namespace crocoddyl::unittest;

//----------------------------------------------------------------------------//
void test_activation_bounds_with_infinity() {
  Eigen::VectorXd lb(1);
  Eigen::VectorXd ub(1);
  double beta;
  beta = 0.1;
  lb[0] = 0;
  ub[0] = std::numeric_limits<double>::infinity();

  // Eigen::VectorXd m = 0.5 * (lb + std::numeric_limits<Eigen::VectorXd>::max());
  Eigen::VectorXd m = 0.5 * (lb + Eigen::VectorXd::Constant(lb.size(), std::numeric_limits<double>::max()));
  Eigen::VectorXd d = 0.5 * (Eigen::VectorXd::Constant(lb.size(), std::numeric_limits<double>::max()) - lb);
  // Eigen::VectorXd d = 0.5 * (std::numeric_limits<Eigen::VectorXd>::max() - lb);

  crocoddyl::ActivationBounds bounds(lb, ub, beta);

  // std::cerr << "ub: \n" << ub << std::endl;
  // std::cerr << "lb: \n" << lb << std::endl;
  // std::cerr << "b.ub: \n" << bounds.ub << std::endl;
  // std::cerr << "b.lb: \n" << bounds.lb << std::endl;
  // std::cerr << "limit: \n" << std::numeric_limits<double>::max() << std::endl;
  std::cerr << "m: \n" << m << std::endl;
  std::cerr << "d: \n" << d << std::endl;
  std::cerr << "comp: \n" << m[0] - beta * d[0] << std::endl;
  
  BOOST_CHECK(bounds.lb !=  m - beta * d);
}

bool register_test()
{
  boost::test_tools::output_test_stream test_name;
  test_name << "test_activation_bounds_with_infinity";
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(
      BOOST_TEST_CASE(boost::bind(&test_activation_bounds_with_infinity)));
  framework::master_test_suite().add(ts);
  return true;
}


int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&register_test, argc, argv);
}

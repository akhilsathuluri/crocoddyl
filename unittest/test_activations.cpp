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

#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include "factory/activation.hpp"
#include "unittest_common.hpp"

using namespace boost::unit_test;
using namespace crocoddyl::unittest;

//----------------------------------------------------------------------------//

void test_construct_data(ActivationModelTypes::Type activation_type) {
  // create the model
  ActivationModelFactory factory;
  const boost::shared_ptr<crocoddyl::ActivationModelAbstract>& model =
      factory.create(activation_type);

  // Run the print function
  std::ostringstream tmp;
  tmp << *model;

  // create the corresponding data object
  boost::shared_ptr<crocoddyl::ActivationDataAbstract> data =
      model->createData();
}

void test_calc_returns_a_value(ActivationModelTypes::Type activation_type) {
  // create the model
  ActivationModelFactory factory;
  const boost::shared_ptr<crocoddyl::ActivationModelAbstract>& model =
      factory.create(activation_type);

  // create the corresponding data object
  boost::shared_ptr<crocoddyl::ActivationDataAbstract> data =
      model->createData();

  // Generating random input vector
  const Eigen::VectorXd r = Eigen::VectorXd::Random(model->get_nr());
  data->a_value = nan("");

  // Getting the state dimension from calc() call
  model->calc(data, r);

  // Checking that calc returns a value
  BOOST_CHECK(!std::isnan(data->a_value));
}

void test_partial_derivatives_against_numdiff(
    ActivationModelTypes::Type activation_type) {
  // create the model
  ActivationModelFactory factory;
  const boost::shared_ptr<crocoddyl::ActivationModelAbstract>& model =
      factory.create(activation_type);

  // create the corresponding data object and set the cost to nan
  boost::shared_ptr<crocoddyl::ActivationDataAbstract> data =
      model->createData();

  crocoddyl::ActivationModelNumDiff model_num_diff(model);
  boost::shared_ptr<crocoddyl::ActivationDataAbstract> data_num_diff =
      model_num_diff.createData();

  // Generating random values for the state and control
  const Eigen::VectorXd r = Eigen::VectorXd::Random(model->get_nr());

  // Computing the activation derivatives
  model->calc(data, r);
  model->calcDiff(data, r);
  model_num_diff.calc(data_num_diff, r);
  model_num_diff.calcDiff(data_num_diff, r);
  // Tolerance defined as in
  // http://www.it.uom.gr/teaching/linearalgebra/NumericalRecipiesInC/c5-7.pdf
  double tol = std::pow(model_num_diff.get_disturbance(), 1. / 3.);
  BOOST_CHECK(std::abs(data->a_value - data_num_diff->a_value) < tol);
  BOOST_CHECK((data->Ar - data_num_diff->Ar).isZero(tol));

  // numerical differentiation of the Hessian is not good enough to be tested.
  // BOOST_CHECK((data->Arr - data_num_diff->Arr).isMuchSmallerThan(1.0, tol));
}

void test_activation_bounds_with_infinity() {
  Eigen::VectorXd lb(1);
  Eigen::VectorXd ub(1);
  double beta;
  beta = 0.1;
  lb[0] = 0;
  ub[0] = std::numeric_limits<double>::infinity();

  Eigen::VectorXd m =
      0.5 * (lb + Eigen::VectorXd::Constant(
                      lb.size(), std::numeric_limits<double>::max()));
  Eigen::VectorXd d =
      0.5 * (Eigen::VectorXd::Constant(lb.size(),
                                       std::numeric_limits<double>::max()) -
             lb);
  crocoddyl::ActivationBounds bounds(lb, ub, beta);
  BOOST_CHECK(bounds.lb != m - beta * d);
}

//----------------------------------------------------------------------------//

void register_unit_tests(ActivationModelTypes::Type activation_type) {
  boost::test_tools::output_test_stream test_name;
  test_name << "test_" << activation_type;
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_construct_data, activation_type)));
  ts->add(BOOST_TEST_CASE(
      boost::bind(&test_calc_returns_a_value, activation_type)));
  ts->add(BOOST_TEST_CASE(
      boost::bind(&test_partial_derivatives_against_numdiff, activation_type)));
  framework::master_test_suite().add(ts);
}

bool register_bounds_unit_test() {
  boost::test_tools::output_test_stream test_name;
  test_name << "test_"
            << "ActivationBoundsInfinity";
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_activation_bounds_with_infinity)));
  framework::master_test_suite().add(ts);
  return true;
}

bool init_function() {
  for (size_t i = 0; i < ActivationModelTypes::all.size(); ++i) {
    register_unit_tests(ActivationModelTypes::all[i]);
  }
  register_bounds_unit_test();
  return true;
}

int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}

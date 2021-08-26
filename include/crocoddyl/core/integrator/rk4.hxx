///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, IRI: CSIC-UPC, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/core/integrator/rk4.hpp"

namespace crocoddyl {

template <typename Scalar>
IntegratedActionModelRK4Tpl<Scalar>::IntegratedActionModelRK4Tpl(
    boost::shared_ptr<DifferentialActionModelAbstract> model,
    boost::shared_ptr<ControlParametrizationModelAbstract> control, const Scalar time_step,
    const bool with_cost_residual)
    : Base(model, control, time_step, with_cost_residual), nthreads_(1) {
  rk4_c_ = {Scalar(0.), Scalar(0.5), Scalar(0.5), Scalar(1.)};

#ifdef CROCODDYL_WITH_MULTITHREADING
  nthreads_ = CROCODDYL_WITH_NTHREADS;
#endif
}

template <typename Scalar>
IntegratedActionModelRK4Tpl<Scalar>::IntegratedActionModelRK4Tpl(
    boost::shared_ptr<DifferentialActionModelAbstract> model, const Scalar time_step, const bool with_cost_residual)
    : Base(model, time_step, with_cost_residual), nthreads_(1) {
  rk4_c_ = {Scalar(0.), Scalar(0.5), Scalar(0.5), Scalar(1.)};

#ifdef CROCODDYL_WITH_MULTITHREADING
  nthreads_ = CROCODDYL_WITH_NTHREADS;
#endif
}

template <typename Scalar>
IntegratedActionModelRK4Tpl<Scalar>::~IntegratedActionModelRK4Tpl() {}

template <typename Scalar>
void IntegratedActionModelRK4Tpl<Scalar>::calc(const boost::shared_ptr<ActionDataAbstract>& data,
                                               const Eigen::Ref<const VectorXs>& x,
                                               const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }

  const std::size_t nv = differential_->get_state()->get_nv();
  const boost::shared_ptr<Data>& d = boost::static_pointer_cast<Data>(data);

  // Computing the next state (discrete time)
  if (enable_integration_) {
    control_->calc(d->control[0], rk4_c_[0], u);
    d->ws[0] = d->control[0]->w;
    differential_->calc(d->differential[0], x, d->ws[0]);
    d->y[0] = x;
    d->ki[0].head(nv) = d->y[0].tail(nv);
    d->ki[0].tail(nv) = d->differential[0]->xout;
    d->integral[0] = d->differential[0]->cost;
    for (std::size_t i = 1; i < 4; ++i) {
      d->dx_rk4[i].noalias() = time_step_ * rk4_c_[i] * d->ki[i - 1];
      differential_->get_state()->integrate(x, d->dx_rk4[i], d->y[i]);
      control_->calc(d->control[i], rk4_c_[i], u);
      d->ws[i] = d->control[i]->w;
      differential_->calc(d->differential[i], d->y[i], d->ws[i]);
      d->ki[i].head(nv) = d->y[i].tail(nv);
      d->ki[i].tail(nv) = d->differential[i]->xout;
      d->integral[i] = d->differential[i]->cost;
    }
    d->dx = (d->ki[0] + Scalar(2.) * d->ki[1] + Scalar(2.) * d->ki[2] + d->ki[3]) * time_step_ / Scalar(6.);
    differential_->get_state()->integrate(x, d->dx, d->xnext);
    d->cost = (d->integral[0] + Scalar(2.) * d->integral[1] + Scalar(2.) * d->integral[2] + d->integral[3]) *
              time_step_ / Scalar(6.);
  } else {
    control_->calc(d->control[0], rk4_c_[0], u);
    d->ws[0] = d->control[0]->w;
    differential_->calc(d->differential[0], x, d->ws[0]);
    d->dx.setZero();
    d->xnext = x;
    d->cost = d->differential[0]->cost;
  }

  // Updating the cost value
  if (with_cost_residual_) {
    d->r = d->differential[0]->r;
  }
}

template <typename Scalar>
void IntegratedActionModelRK4Tpl<Scalar>::calcDiff(const boost::shared_ptr<ActionDataAbstract>& data,
                                                   const Eigen::Ref<const VectorXs>& x,
                                                   const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }

  const std::size_t nv = state_->get_nv();
  const std::size_t nu = control_->get_nu();
  const boost::shared_ptr<Data>& d = boost::static_pointer_cast<Data>(data);
  assert_pretty(d->dyi_dx[0] == MatrixXs::Identity(state_->get_ndx(), state_->get_ndx()),
                "you have changed dyi_dx[0] values that supposed to be constant.");
  assert_pretty(d->dki_dx[0].topRightCorner(nv, nv) == MatrixXs::Identity(nv, nv),
                "you have changed dki_dx[0] values that supposed to be constant.");

  if (enable_integration_) {
#ifdef CROCODDYL_WITH_MULTITHREADING
#pragma omp parallel for num_threads(nthreads_)
#endif
    for (std::size_t i = 0; i < 4; ++i) {
      differential_->calcDiff(d->differential[i], d->y[i], d->ws[i]);
    }

    const boost::shared_ptr<DifferentialActionDataAbstract>& k0_data = d->differential[0];
    d->dki_dx[0].bottomRows(nv) = k0_data->Fx;
    control_->multiplyByJacobian(d->control[0], k0_data->Fu,
                                 d->dki_du[0].bottomRows(nv));  // dki_du = dki_dw * dw_du

    d->dli_dx[0] = k0_data->Lx;
    control_->multiplyJacobianTransposeBy(d->control[0], k0_data->Lu,
                                          d->dli_du[0]);  // dli_du = dli_dw * dw_du

    d->ddli_ddx[0] = k0_data->Lxx;
    d->ddli_ddw[0] = k0_data->Luu;
    control_->multiplyByJacobian(d->control[0], d->ddli_ddw[0], d->ddli_dwdu[0]);  // ddli_dwdu = ddli_ddw * dw_du
    control_->multiplyJacobianTransposeBy(d->control[0], d->ddli_dwdu[0],
                                          d->ddli_ddu[0]);  // ddli_ddu = dw_du.T * ddli_dwdu
    d->ddli_dxdw[0] = k0_data->Lxu;
    control_->multiplyByJacobian(d->control[0], d->ddli_dxdw[0], d->ddli_dxdu[0]);  // ddli_dxdu = ddli_dxdw * dw_du

    for (std::size_t i = 1; i < 4; ++i) {
      const boost::shared_ptr<DifferentialActionDataAbstract>& ki_data = d->differential[i];
      Eigen::Block<MatrixXs> dkvi_dq = d->dki_dx[i].bottomLeftCorner(nv, nv);
      Eigen::Block<MatrixXs> dkvi_dv = d->dki_dx[i].bottomRightCorner(nv, nv);
      Eigen::Block<MatrixXs> dkqi_du = d->dki_du[i].topLeftCorner(nv, nu);
      Eigen::Block<MatrixXs> dkvi_du = d->dki_du[i].bottomLeftCorner(nv, nu);
      const Eigen::Block<MatrixXs> dki_dqi = ki_data->Fx.bottomLeftCorner(nv, nv);
      const Eigen::Block<MatrixXs> dki_dvi = ki_data->Fx.bottomRightCorner(nv, nv);
      const Eigen::Block<MatrixXs> dqi_dq = d->dyi_dx[i].topLeftCorner(nv, nv);
      const Eigen::Block<MatrixXs> dqi_dv = d->dyi_dx[i].topRightCorner(nv, nv);
      const Eigen::Block<MatrixXs> dvi_dq = d->dyi_dx[i].bottomLeftCorner(nv, nv);
      const Eigen::Block<MatrixXs> dvi_dv = d->dyi_dx[i].bottomRightCorner(nv, nv);
      const Eigen::Block<MatrixXs> dqi_du = d->dyi_du[i].topLeftCorner(nv, nu);
      const Eigen::Block<MatrixXs> dvi_du = d->dyi_du[i].bottomLeftCorner(nv, nu);

      d->dyi_dx[i].noalias() = d->dki_dx[i - 1] * rk4_c_[i] * time_step_;
      d->dyi_du[i].noalias() = d->dki_du[i - 1] * rk4_c_[i] * time_step_;
      state_->JintegrateTransport(x, d->dx_rk4[i], d->dyi_dx[i], second);
      state_->Jintegrate(x, d->dx_rk4[i], d->dyi_dx[i], d->dyi_dx[i], first, addto);
      state_->JintegrateTransport(x, d->dx_rk4[i], d->dyi_du[i], second);  // dyi_du = Jintegrate * dyi_du
      // Sparse matrix-matrix multiplication for computing:
      //   i. d->dki_dx[i].noalias() = d->dki_dy[i] * d->dyi_dx[i], where dki_dy is ki_data.Fx
      d->dki_dx[i].topRows(nv) = d->dyi_dx[i].bottomRows(nv);
      if (i == 1) {
        dkvi_dq = dki_dqi;
        dkvi_dq.noalias() += dki_dvi * dvi_dq;
        dkvi_dv = time_step_ / Scalar(2.) * dki_dqi;
      } else {
        dkvi_dq.noalias() = dki_dqi * dqi_dq;
        dkvi_dq.noalias() += dki_dvi * dvi_dq;
        dkvi_dv.noalias() = dki_dqi * dqi_dv;
      }
      dkvi_dv.noalias() += dki_dvi * dvi_dv;
      //  ii. d->dki_du[i].noalias() = d->dki_dy[i] * d->dyi_du[i], where dki_dy is ki_data.Fx
      dkqi_du = dvi_du;
      dkvi_du.noalias() = dki_dqi * dqi_du;
      dkvi_du.noalias() += dki_dvi * dvi_du;

      control_->multiplyByJacobian(d->control[i], ki_data->Fu,
                                   d->dfi_du[i].bottomRows(nv));  // dfi_du = dki_dw * dw_du
      d->dki_du[i] += d->dfi_du[i];

      d->dli_dx[i].noalias() = ki_data->Lx.transpose() * d->dyi_dx[i];
      control_->multiplyJacobianTransposeBy(d->control[i], ki_data->Lu,
                                            d->dli_du[i]);  // dli_du = Lu * dw_du
      d->dli_du[i].noalias() += ki_data->Lx.transpose() * d->dyi_du[i];

      d->Lxx_partialx[i].noalias() = ki_data->Lxx * d->dyi_dx[i];
      d->ddli_ddx[i].noalias() = d->dyi_dx[i].transpose() * d->Lxx_partialx[i];

      control_->multiplyByJacobian(d->control[i], ki_data->Lxu, d->Lxu_i[i]);  // Lxu = Lxw * dw_du
      d->Luu_partialx[i].noalias() = d->Lxu_i[i].transpose() * d->dyi_du[i];
      d->Lxx_partialu[i].noalias() = ki_data->Lxx * d->dyi_du[i];
      control_->multiplyByJacobian(d->control[i], ki_data->Luu,
                                   d->ddli_dwdu[i]);  // ddli_dwdu = ddli_ddw * dw_du
      control_->multiplyJacobianTransposeBy(d->control[i], d->ddli_dwdu[i],
                                            d->ddli_ddu[i]);  // ddli_ddu = dw_du.T * ddli_dwdu
      d->ddli_ddu[i].noalias() +=
          d->Luu_partialx[i].transpose() + d->Luu_partialx[i] + d->dyi_du[i].transpose() * d->Lxx_partialu[i];

      d->ddli_dxdw[i].noalias() = d->dyi_dx[i].transpose() * ki_data->Lxu;
      control_->multiplyByJacobian(d->control[i], d->ddli_dxdw[i],
                                   d->ddli_dxdu[i]);  // ddli_dxdu = ddli_dxdw * dw_du
      d->ddli_dxdu[i].noalias() += d->dyi_dx[i].transpose() * d->Lxx_partialu[i];
    }

    d->Fx.noalias() = time_step_ / Scalar(6.) *
                      (d->dki_dx[0] + Scalar(2.) * d->dki_dx[1] + Scalar(2.) * d->dki_dx[2] + d->dki_dx[3]);
    state_->JintegrateTransport(x, d->dx, d->Fx, second);
    state_->Jintegrate(x, d->dx, d->Fx, d->Fx, first, addto);

    d->Fu.noalias() = time_step_ / Scalar(6.) *
                      (d->dki_du[0] + Scalar(2.) * d->dki_du[1] + Scalar(2.) * d->dki_du[2] + d->dki_du[3]);
    state_->JintegrateTransport(x, d->dx, d->Fu, second);

    d->Lx.noalias() = time_step_ / Scalar(6.) *
                      (d->dli_dx[0] + Scalar(2.) * d->dli_dx[1] + Scalar(2.) * d->dli_dx[2] + d->dli_dx[3]);
    d->Lu.noalias() = time_step_ / Scalar(6.) *
                      (d->dli_du[0] + Scalar(2.) * d->dli_du[1] + Scalar(2.) * d->dli_du[2] + d->dli_du[3]);

    d->Lxx.noalias() = time_step_ / Scalar(6.) *
                       (d->ddli_ddx[0] + Scalar(2.) * d->ddli_ddx[1] + Scalar(2.) * d->ddli_ddx[2] + d->ddli_ddx[3]);
    d->Luu.noalias() = time_step_ / Scalar(6.) *
                       (d->ddli_ddu[0] + Scalar(2.) * d->ddli_ddu[1] + Scalar(2.) * d->ddli_ddu[2] + d->ddli_ddu[3]);
    d->Lxu.noalias() =
        time_step_ / Scalar(6.) *
        (d->ddli_dxdu[0] + Scalar(2.) * d->ddli_dxdu[1] + Scalar(2.) * d->ddli_dxdu[2] + d->ddli_dxdu[3]);
  } else {
    const boost::shared_ptr<DifferentialActionDataAbstract>& k0_data = d->differential[0];
    differential_->calcDiff(k0_data, x, d->ws[0]);
    differential_->get_state()->Jintegrate(x, d->dx, d->Fx, d->Fx);
    d->Fu.setZero();
    d->Lx = k0_data->Lx;
    d->Lu = k0_data->Lu;
    d->Lxx = k0_data->Lxx;
    d->Lxu = k0_data->Lxu;
    d->Luu = k0_data->Luu;
  }
}

template <typename Scalar>
boost::shared_ptr<ActionDataAbstractTpl<Scalar> > IntegratedActionModelRK4Tpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
bool IntegratedActionModelRK4Tpl<Scalar>::checkData(const boost::shared_ptr<ActionDataAbstract>& data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  if (data != NULL) {
    return differential_->checkData(d->differential[0]) && differential_->checkData(d->differential[2]) &&
           differential_->checkData(d->differential[1]) && differential_->checkData(d->differential[3]);
  } else {
    return false;
  }
}

template <typename Scalar>
void IntegratedActionModelRK4Tpl<Scalar>::quasiStatic(const boost::shared_ptr<ActionDataAbstract>& data,
                                                      Eigen::Ref<VectorXs> u, const Eigen::Ref<const VectorXs>& x,
                                                      const std::size_t maxiter, const Scalar tol) {
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }

  // Static casting the data
  const boost::shared_ptr<Data>& d = boost::static_pointer_cast<Data>(data);
  d->control[0]->w *= 0.;
  differential_->quasiStatic(d->differential[0], d->control[0]->w, x, maxiter, tol);
  control_->params(d->control[0], 0., d->control[0]->w);
  u = d->control[0]->u;
}

template <typename Scalar>
void IntegratedActionModelRK4Tpl<Scalar>::print(std::ostream& os) const {
  os << "IntegratedActionModelRK4 {dt=" << time_step_ << ", " << *differential_ << "}";
}

template <typename Scalar>
void IntegratedActionModelRK4Tpl<Scalar>::set_nthreads(const int nthreads) {
#ifndef CROCODDYL_WITH_MULTITHREADING
  (void)nthreads;
  std::cerr << "Warning: the number of threads won't affect the computational performance as multithreading "
               "support is not enabled."
            << std::endl;
#else
  if (nthreads < 1) {
    nthreads_ = CROCODDYL_WITH_NTHREADS;
  } else {
    nthreads_ = static_cast<std::size_t>(nthreads);
  }
#endif
}

template <typename Scalar>
std::size_t IntegratedActionModelRK4Tpl<Scalar>::get_nthreads() const {
#ifndef CROCODDYL_WITH_MULTITHREADING
  std::cerr << "Warning: the number of threads won't affect the computational performance as multithreading "
               "support is not enabled."
            << std::endl;
#endif
  return nthreads_;
}

}  // namespace crocoddyl

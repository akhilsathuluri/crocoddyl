///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2023, LAAS-CNRS, University of Edinburgh,
//                          University of Oxford, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/callbacks.hpp"

#include "python/crocoddyl/core/core.hpp"
#include "python/crocoddyl/utils/copyable.hpp"

namespace crocoddyl {
namespace python {

void exposeCallbacks() {
  bp::register_ptr_to_python<boost::shared_ptr<CallbackAbstract> >();

  bp::enum_<VerboseLevel>("VerboseLevel")
      .value("_0", _0)
      .value("_1", _1)
      .value("_2", _2)
      .value("_3", _3)
      .value("_4", _4);

  bp::class_<CallbackVerbose, bp::bases<CallbackAbstract> >(
      "CallbackVerbose", "Callback function for printing the solver values.",
      bp::init<bp::optional<VerboseLevel, int> >(
          bp::args("self", "level", "precision"),
          "Initialize the differential verbose callback.\n\n"
          ":param level: verbose level (default _4)\n"
          ":param precision: precision of floating point output (default 5)"))
      .def("__call__", &CallbackVerbose::operator(), bp::args("self", "solver"),
           "Run the callback function given a solver.\n\n"
           ":param solver: solver to be diagnostic")
      .add_property("level", &CallbackVerbose::get_level,
                    &CallbackVerbose::set_level, "verbose level")
      .add_property("precision", &CallbackVerbose::get_precision,
                    &CallbackVerbose::set_precision, "precision")
      .def(CopyableVisitor<CallbackVerbose>());
}

}  // namespace python
}  // namespace crocoddyl

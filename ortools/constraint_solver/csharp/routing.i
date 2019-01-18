// Copyright 2010-2018 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// TODO(user): Refactor this file to adhere to the SWIG style guide.
%include "ortools/constraint_solver/csharp/constraint_solver.i"
%include "ortools/constraint_solver/csharp/routing_types.i"
%include "ortools/constraint_solver/csharp/routing_index_manager.i"
%include "ortools/util/csharp/functions.i"

// We need to forward-declare the proto here, so that PROTO_INPUT involving it
// works correctly. The order matters very much: this declaration needs to be
// before the %{ #include ".../routing.h" %}.
namespace operations_research {
class RoutingModelParameters;
class RoutingSearchParameters;
}  // namespace operations_research

// Include the file we want to wrap a first time.
%{
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"
#include "ortools/constraint_solver/routing_types.h"
%}

%module(directors="1") operations_research;

%ignore operations_research::RoutingModel::AddVectorDimension(
    const int64* values,
    int64 capacity,
    const std::string& name);

%ignore operations_research::RoutingModel::AddMatrixDimension(
    const int64* const* values,
    int64 capacity,
    const std::string& name);

%ignore operations_research::RoutingModel::RegisterStateDependentTransitCallback;
%ignore operations_research::RoutingModel::StateDependentTransitCallback;
%ignore operations_research::RoutingModel::MakeStateDependentTransit;
%ignore operations_research::RoutingModel::AddDimensionDependentDimensionWithVehicleCapacity;

%ignore operations_research::RoutingModel::RegisterTransitCallback(
    operations_research::TransitCallback2);
%ignore operations_research::RoutingModel::RegisterUnaryTransitCallback(
    operations_research::TransitCallback1);

%extend operations_research::RoutingModel {
  int RegisterTransitCallback(swig_util::LongLongToLong* callback) {
    return $self->RegisterTransitCallback([callback](int64 i, int64 j) {
        return callback->Run(i, j);
      });
  }
  int RegisterUnaryTransitCallback(swig_util::LongToLong* callback) {
    return $self->RegisterUnaryTransitCallback([callback](int64 i) {
        return callback->Run(i);
      });
  }
  void AddVectorDimension(const std::vector<int64>& values,
                          int64 capacity,
                          bool fix_start_cumul_to_zero,
                          const std::string& name) {
    DCHECK_EQ(values.size(), self->nodes());
    self->AddVectorDimension(values.data(), capacity,
                             fix_start_cumul_to_zero, name);
  }
}

// Add Support for delegate TransitCallback
// 1) Define C function pointer and add them to the wrapper.
%{
namespace operations_research {
 typedef int64 (*TransitCallback)(int64, int64);
 typedef int64 (*UnaryTransitCallback)(int64);
}  // namespace operations_research
%}

// 2) Define Delegate in C#
// 4) Modify RoutingModel to keep track of all delegates...
%typemap(cscode) operations_research::RoutingModel %{
  // Store list of delegate to avoid the GC to reclaim them.
  private List<TransitCallback> transitCallbacks;
  private List<UnaryTransitCallback> unaryTransitCallbacks;

  // Ensure that the GC does not collect any TransitCallback set from C#
  // as the underlying C++ class stores a shallow copy
  private TransitCallback StoreTransitCallback(TransitCallback c) {
    if (transitCallbacks == null) transitCallbacks = new List<TransitCallback>();
    transitCallbacks.Add(c);
    return c;
  }
  private UnaryTransitCallback StoreUnaryTransitCallback(UnaryTransitCallback c) {
    if (unaryTransitCallbacks == null) unaryTransitCallbacks = new List<UnaryTransitCallback>();
    unaryTransitCallbacks.Add(c);
    return c;
  }
%}

// 3) Link function pointer to delegate
%define %DEFINE_CALLBACK(TYPE, CSTYPE, STORE)
  %typemap(ctype) TYPE, TYPE& "void*"
  %typemap(in) TYPE  %{ $1 = (TYPE)$input; %}
  %typemap(in) TYPE& %{ $1 = (TYPE*)&$input; %}
  %typemap(imtype, out="IntPtr") TYPE, TYPE& "CSTYPE"
  %typemap(cstype, out="IntPtr") TYPE, TYPE& "CSTYPE"
  %typemap(csin) TYPE, TYPE& "STORE($csinput)"
%enddef
%DEFINE_CALLBACK(operations_research::TransitCallback, TransitCallback, StoreTransitCallback)
%DEFINE_CALLBACK(operations_research::UnaryTransitCallback, UnaryTransitCallback, StoreUnaryTransitCallback)

// 5) Add methods to RoutingModel to support this function pointers.
%extend operations_research::RoutingModel {
 int RegisterTransitCallback(operations_research::TransitCallback c) {
   return $self->RegisterTransitCallback([c](int64 i, int64 j) {
       return (*c)(i, j);
       });
 }
 int RegisterUnaryTransitCallback(operations_research::UnaryTransitCallback c) {
   return $self->RegisterUnaryTransitCallback([c](int64 i) {
       return (*c)(i);
       });
 }
}

// Add PickupAndDeliveryPolicy enum value to RoutingModel (like RoutingModel::Status)
// For C++11 strongly typed enum SWIG support see https://github.com/swig/swig/issues/316
%extend operations_research::RoutingModel {
  static const operations_research::RoutingModel::PickupAndDeliveryPolicy ANY =
  operations_research::RoutingModel::PickupAndDeliveryPolicy::ANY;
  static const operations_research::RoutingModel::PickupAndDeliveryPolicy LIFO =
  operations_research::RoutingModel::PickupAndDeliveryPolicy::LIFO;
  static const operations_research::RoutingModel::PickupAndDeliveryPolicy FIFO =
  operations_research::RoutingModel::PickupAndDeliveryPolicy::FIFO;
}

%rename("GetStatus") operations_research::RoutingModel::status;
%rename("%(camelcase)s", %$isfunction) "";

// Protobuf support
PROTO_INPUT(operations_research::RoutingSearchParameters,
            Google.OrTools.ConstraintSolver.RoutingSearchParameters,
            search_parameters)
PROTO_INPUT(operations_research::RoutingModelParameters,
            Google.OrTools.ConstraintSolver.RoutingModelParameters,
            parameters)
PROTO2_RETURN(operations_research::RoutingSearchParameters,
              Google.OrTools.ConstraintSolver.RoutingSearchParameters)
PROTO2_RETURN(operations_research::RoutingModelParameters,
              Google.OrTools.ConstraintSolver.RoutingModelParameters)


%include "ortools/constraint_solver/routing_parameters.h"
%include "ortools/constraint_solver/routing.h"

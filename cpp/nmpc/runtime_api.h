#pragma once
extern "C" {
void* nmpc_create_mode(int hardware);
void nmpc_physical_limits(void* handle, double* lower, double* upper, const double* margins);
void nmpc_destroy(void* handle);
// Arrays are contiguous row-major doubles. q is 14, targets are 24 pose outputs.
int nmpc_query(void* handle, const double* q, double* distances, double* gradients,
               double* flange_matrices);
int nmpc_solve(void* handle, const double* nominal_states, const double* nominal_controls,
               const double* targets, double* controls);
}
extern "C" int nmpc_certify(void* handle, const double* knots, const double* jerks,
                            int segments, double dt, const double* initial_gaps,
                            const double* motion_radii, const double* margins);

# Acados problems

## Overview
Acados tends to raise this exception (`Exception: acados acados_ocp_solver returned status 2. Exiting.`) when the target coordinates are far distances away from the original position. From the docs, status 2 means that the nlp solver's maximum number of iterations has been reached. I have tried setting the `ocp.solver_options.nlp_solver_max_iter` option to 400 rather than the default 100 to no avail.

## Some things that have been tried
- Change `ocp.solver_options.nlp_solver_max_iter` from default 100 to 400
- Switching integrator type from `ERK` to `IRK`
- Reducing size of problem by making the time horizon `T` and number of control intervals `N` smaller (`T` from 10 to 1, `N` from 40 to `5`)
- - Make the ratio between `T` and `N` larger (`T=1, N=20`)
- Multiply `y_ref` by array of zeroes
- Adjusting weight values such that they are more uniform in magnitude
- Set incremental `y_ref` values for each shooting node (`[xf*i/N, yf*i/N, 0, 0, 0, 0, 0]`)

## Some things noticed
- Simulation ran to completion w/ far target in the following cases:
  - Uniform `y_ref` w/ `T=1, N=20`, `0 < v < 1`. Exception raised when `T=2` no matter how small the upper constraint on `v` was changed nor how large the `N` was made
  - Incremental `y_ref` w/ `T=1, N=20`, `0 < v < 2`. Exception raised when `T=2` no matter how large `N` was made.
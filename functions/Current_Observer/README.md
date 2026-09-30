# Current Reconstruction Variant interface

Place the Variant Subsystem at:

`FCSMPC_double_vector/FCS_MPC/SDA_Solve/SDA Solve`, immediately before `SDA`.

Both branches have the identical ordered interface:

```matlab
[idiqEstimated, innovation, idiqFilteredEstimated, estimateValid] = branch( ...
    idiqFiltered, Ad, Bd, fd, Bs_TI, Sin_Cos_Theta, theta_el, wel, Ts, ...
    U_applied, controllerDelay, filterTimeConstant, sensorTimeConstant, ...
    phaseShift_rad, observerGain, reset)
```

Dimensions: idiqFiltered 2x1; Ad/Bd 2x2; fd 2x1; Bs_TI 2x3;
Sin_Cos_Theta 2x1 ordered [sin;cos]; U_applied 6x1 ordered [U1;U2];
observerGain 6x2. Other inputs are scalars. U_applied must be the delayed
actual six-state output of SDA covering the interval ending at the current
measurement. Ts is one vector duration, so the observer update period is 2*Ts.

Both branches output dq current at the command actuation instant after the
same external controllerDelay convention. `LegacyRippleCompensatorDQ` uses
phaseShift_rad and ignores observerGain/reset. `CurrentObserverDQ` uses the
filter time constants and observerGain and ignores phaseShift_rad. The latter
returns the pre-correction innovation and posterior predicted filtered current.
estimateValid is false on observer initialization/reset; it is always true for
the memoryless legacy branch.

`ConfigureCurrentObserverGainDQ` creates a fixed gain for one speed point from
Ad, wel, Ts and filter constants. Build a speed-indexed gain LUT during model
initialization; do not call DARE in generated real-time code. Q and R entries
are tuning parameters, not physical constants unless derived from identified
noise statistics. Interpolate gains only after checking closed-loop observer
stability across the complete speed grid.

The observer filter model uses exact first-order decay over each 20 us vector
with trapezoidal input approximation. The plant currently realizes both
filters as 1 us Tustin blocks. These are close models, but not algebraically
identical. A later refinement can pass the actual discrete filter coefficients
or run a 1 us multirate observer if benchmark residuals show a material error.

`Assembly_QP.m` has become a pure QP assembly stage. Replace the embedded
MATLAB Function code and ports in both inner SDA variants after the outer
Variant Subsystem is connected. Its new ordered interface is:

```matlab
Assembly_QP(Ad,Bd,fd,Gamma,Phi,Apow,Lambdafsw_Ssquare,Lambdafsw_SE, ...
    Bs_TI,idiqEstimated,idiq_ref,Sin_Cos_Theta,wel,Ts,U_applied, ...
    regularization,iabcPredictedPrevious,predictionValid)
```

Existing output names are retained. `iabcCompensated` now means the inverse
Park/Clarke transform of the selected upstream estimate and remains compatible
with SDA_Solve's CMV/dead-time current prediction.

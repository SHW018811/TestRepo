#ifndef EKF_SOC_H
#define EKF_SOC_H
void Init_Battery();
void Update_Temperature();
void Update_Resistance();
void SimulateTerminalVoltage();
/*================================================================
battery[
=================================================================*/
// Todo -> HiHee
// SocFromOcv Error change
void EKFpredict(int k);
void ComputeJacobianH(int k, double *local_H);
void SOCEKF();
void ChargeCurrentLimits();
#endif
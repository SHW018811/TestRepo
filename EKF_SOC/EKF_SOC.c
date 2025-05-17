#include "all_headers.h"

void Init_Battery(){
    double local_Capacity1c = 0;
    for(int i=0; i<BATTERY_CELLS; i++){
        battery[i].SOC = SocFromOcv(charge_ocv[0]);
        battery[i].V1 = 0;
        battery[i].Charge_Current = -0.41;
        battery[i].Capacity = 4.07611;
        local_Capacity1c = (battery[i].Capacity * 3600) / 100;
        battery[i].R0 = 0.00005884314;
        battery[i].R1 = 0.01145801322;
        battery[i].C1 = 4846.080679;
        battery[i].Voltage_terminal = battery[i].Charge_Current * battery[i].R1 * (1 - exp(-DELTA_TIME / (battery[i].R1 * battery[i].C1)));
        battery[i].Temperature = 25;
        estimate[i].SOC = battery[i].SOC;
        estimate[i].V1 = battery[i].V1;
        estimate[i].Voltage_terminal = 0;
    }
}

void Update_Temperature(){
    double local_airtemp = 0.0; //Air temperature
    double internal_heat[BATTERY_CELLS];
    double heater_on, cooler_on, total_heat[BATTERY_CELLS];
    for(int i=0; i<BATTERY_CELLS; i++){
        internal_heat[i] = battery[i].R0 * pow(battery[i].Charge_Current, 2);
        heater_on = (battery[i].Temperature < HEATER_ON_TEMP)? HEAT_COOL_POWER : 0;
        cooler_on = (battery[i].Temperature > COOLER_ON_TEMP)? HEAT_COOL_POWER : 0;
        total_heat[i] = internal_heat[i] + heater_on - cooler_on;
        battery[i].Temperature += DELTA_TIME / 200 * (total_heat[i] - (battery[i].Temperature - local_airtemp) / 3);
    }
}

void Update_Resistance(){
    const double local_R0_reference = 0.00005884314;
    const double local_R1_reference = 0.01145801322;
    const double local_coeff = 0.003;
    for(int i=0; i<BATTERY_CELLS; i++){
        battery[i].R0 = local_R0_reference * (1 + local_coeff * (battery[i].Temperature - 25));
        battery[i].R1 = local_R1_reference * (1 + local_coeff * (battery[i].Temperature - 25));
    }
}

void SimulateTerminalVoltage(){
    for(int i=0; i<BATTERY_CELLS; i++){
        battery[i].SOC -= COULOMIC_EFFICIENCY * DELTA_TIME / ((battery[i].Capacity * 3600) / 100) * battery[i].Charge_Current;
        if(battery[i].SOC < 0) battery[i].SOC = 0;
        if(battery[i].SOC > 100) battery[i].SOC = 100;
        battery[i].V1 = battery[i].V1 * exp(-DELTA_TIME / (battery[i].R1 * battery[i].C1)) + battery[i].R1 * (1 - exp(-DELTA_TIME / (battery[i].R1 * battery[i].C1))) * battery[i].Charge_Current;
        battery[i].Voltage_terminal = OcvFromSoc(battery[i].SOC) - battery[i].V1 - battery[i].R0 * battery[i].Charge_Current;
    }
}

/*================================================================
battery[
=================================================================*/
// Todo -> HiHee
// SocFromOcv Error change
void EKFpredict(int k){//I think error HiHEE Tot
    double local_nconstant = exp(-DELTA_TIME / (cell_data[k].R1 * cell_data[k].C1));
    //estimate[k].SOC = SocFromOcv(cell_data[k].voltage) - COULOMBIC_EFFICIENCY * DELTA_TIME / ((cell_data[k].capacity * 3600) / 100) * cell_data[k].charge_current;
    estimate[k].SOC = battery[k].SOC - COULOMBIC_EFFICIENCY * DELTA_TIME / ((cell_data[k].capacity * 3600) / 100) * cell_data[k].charge_current;
    estimate[k].V1 = local_nconstant * battery[k].voltage_delay + cell_data[k].R1 * (1 - local_nconstant) * cell_data[k].charge_current;
}

void ComputeJacobianH(int k, double *local_H){
    double local_soc_high  = estimate[k].SOC + 0.05;
    double local_soc_low   = estimate[k].SOC - 0.05;
    if(local_soc_high > 100) local_soc_high = 100;
    if(local_soc_low < 0)    local_soc_low = 0;
    local_H[0] = (OcvFromSoc(local_soc_high) - OcvFromSoc(local_soc_low)) / (local_soc_high - local_soc_low);
    local_H[1] = -1;
    if(fabs(local_H[0]) < 1e-4f) local_H[0] = (local_H[0] >= 0)? 1e-4f : -1e-4f;
}

//측정 터미널 전압 기반 -> SOC, V1 추정
void SOCEKF(){
    double local_FP[2][2], local_Pp[2][2], local_H[2], local_HP[2], local_y, local_I_KH[2][2],local_error[2][2];                                    // ToDoLiSt[hihee]: local_Pp
    for(int i=0; i<BATTERY_CELLS; i++){
        if(!battery_state[i].init){
            battery_state[i].F[0][0] = 1.0; battery_state[i].F[0][1] = 0.0;
            battery_state[i].F[1][0] = 0.0; battery_state[i].F[1][1] = exp(-DELTA_TIME / (cell_data[i].R1 * cell_data[i].C1));
            battery_state[i].Q[0][0] = 0.0000001; battery_state[i].Q[0][1] = 0.0;
            battery_state[i].Q[1][0] = 0.0; battery_state[i].Q[1][1] = 0.0000001;
            battery_state[i].R = 500.0;
            battery_state[i].P[0][0] = 3000.0; battery_state[i].P[0][1] = 0.0;
            battery_state[i].P[1][0] = 0.0; battery_state[i].P[1][1] = 3000.0;
            battery_state[i].init = 1;
            //battery[i].SOC, battery[i].voltage_delay
        }
        EKFpredict(i);
        /*
            EKF formula
            K = P^- * H^T * (H * P^- * H^T + R)^-1
            H = [d(Vt) / d(SOC), d(Vt) / d(V1)]
            Compute F * P array matrix multiplication
        */
        for(int k=0; k<2; ++k) for(int j=0; j<2; ++j){
            local_FP[k][j] = battery_state[i].F[k][0] * battery_state[i].P[0][j] + battery_state[i].F[k][1] * battery_state[i].P[1][j];
        }
        //Compute Pp -> F * P * F^T + Q (T meaning -> [0][2] makes -> [2][0] matrix)
        for(int k=0; k<2; ++k) for(int j=0; j<2; ++j){
            battery_state[i].Pp[k][j] = local_FP[k][0] * battery_state[i].F[j][0] + local_FP[k][1] * battery_state[i].F[j][1] + battery_state[i].Q[k][j];
        }
        //Linearize nonlinearity through gradient
        ComputeJacobianH(i, local_H);
        //Compute local_H * Pp
        local_HP[0] = local_H[0] * battery_state[i].Pp[0][0] + local_H[1] * battery_state[i].Pp[1][0];
        local_HP[1] = local_H[0] * battery_state[i].Pp[0][1] + local_H[1] * battery_state[i].Pp[1][1];
        //Residual
        estimate[i].Voltage_terminal = OcvFromSoc(estimate[i].SOC) - battery[i].voltage_delay - cell_data[i].R0 * cell_data[i].charge_current;
        local_y = cell_data[i].voltage - estimate[i].Voltage_terminal; //local_y is residual
        // EKF update step
        double S = local_H[0] * local_HP[0] + local_H[1] * local_HP[1] + battery_state[i].R;
        double K[2];
        K[0] = local_HP[0] / S;
        K[1] = local_HP[1] / S;

        estimate[i].SOC += K[0] * local_y;
        estimate[i].V1  += K[1] * local_y;

        for (int k = 0; k < 2; k++) for (int j = 0; j < 2; j++) local_I_KH[k][j] = (k == j ? 1 : 0) - K[k] * local_H[j];
        //Update P
        for(int k=0; k<2; ++k) for(int j=0; j<2; ++j){
            local_error[k][j] = local_I_KH[k][0] * battery_state[i].Pp[0][j] + local_I_KH[k][1] * battery_state[i].Pp[1][j];
        }
        memcpy(battery_state[i].P, local_error, sizeof(battery_state[i].P));
    }
}

void ChargeCurrentLimits(){
    double local_charge_current_cc = -1 * CELL_CAPACITY;
    double local_charge_current_min_cv = -0.05 * CELL_CAPACITY;
    double local_hystersis = 0.01;
    double local_voltage_control = 50;
    double local_ratio = 0;
    double local_charge_current_limits = local_charge_current_cc;
    for(int i=0; i<BATTERY_CELLS; i++){
        if(estimate[i].SOC > SOC_TAPER_START){
            local_ratio = (estimate[i].SOC - SOC_TAPER_START) / (SOC_TAPER_END - SOC_TAPER_START);
            if(local_ratio > 1) local_ratio = 1;
            local_charge_current_limits = local_charge_current_cc * (1 - 0.8 * local_ratio);
        }
        if(cell_data[i].voltage >= VOLTAGE_MAX - local_hystersis){
            local_charge_current_limits = local_charge_current_cc + local_voltage_control * (battery[i].voltage_terminal - VOLTAGE_MAX);
            if(local_charge_current_limits < local_charge_current_min_cv) local_charge_current_limits = local_charge_current_min_cv;
            if(local_charge_current_limits > 0.0) local_charge_current_limits = 0;
        }
        if(cell_data[i].Temperature < 0.0) local_charge_current_limits = 0.0;
        else if(cell_data[i].Temperature < 15.0) local_charge_current_cc *= 0.5;
        battery[i].charge_current = local_charge_current_limits;
    }
}
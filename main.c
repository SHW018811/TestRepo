
/*================================================================
main.c
This program is a CAN packet sender which updates BMS status live-on-time.

Parameters:
-For each cells..
    - batterytemp
    - batteryvoltage
    - batterycurrent
    - batterySOH
==================================================================*/
#include "all_headers.h"
#define VERSION "0.4"

//input_thread
#define MAX_STRUCTS 9
//CAN_sender_thread
#define SLEEPTIME 100000  //100ms
//print_screen_thread
#define BAR_WIDTH 50        // MAX-length of loading bar
#define LOAD_TIME 100       // load time (unit: ms)
//charging_batterypack_thread
#define RANDOM_PERCENT 10

//handle arrow key input
#define LEFT 'D'
#define RIGHT 'C'
#define UP 'A'
#define DOWN 'B'

pthread_mutex_t lock; // mutex to use structure-located-mem

typedef struct {
    uint32_t id;      // CAN ID
    uint8_t data[8];  // CAN data
    uint8_t len;      // data length
} CAN_Message;

int ifrunning = 1;
int input_mode = 0;     //which value will be change
int ifvoltageerror = 0; //check if battery voltage value is in range
int iftempfan = 0;      //if fan works

static double SOC_estimate = 0;
static double voltage_delay_est = 0;
/*================================================================
functions for print screen
=================================================================*/

double scale_voltage(uint16_t raw) {
    return raw * 0.1;
}

uint16_t descale_voltage(double voltage) {
    return (uint16_t)(voltage * 10.0f);
}

void init_battery_array() { //Initialize까지 진행됐고 이제 이거 수정해보자.
    //초기 배터리 셀 정보를 모든 배터리 셀 배열에 default_battery 정보를 넘긴다
    //최저 2.5 - 4.2 정격 전압 3.65 이므로 192개면 아이오닉 배터리 정격 전압이 나올듯함
    for (int i = 0; i < BATTERY_CELLS; i++) {
        memcpy(&battery[i], &default_battery, sizeof(Battery_t));
    }
}

void print_battery_bar(int soc){                // soc stands on 0x626, BMS_SOC_t
    int bar_length = (soc * BAR_WIDTH) / 100;
    int i;
    
    printf("Battery: [");
    for (i = 0; i < bar_length; i++) {
        printf("█");
    }
    for (; i < BAR_WIDTH; i++) {
        printf(" ");
    }
    printf("] %d%%  ", soc);
    if (ifvoltageerror) {
        printf(HIGHLIGHT"voltage or soc error"RESET"               \n");
    }
    else printf("                                 \n");
    fflush(stdout);
}

void print_inputmode(int mode) {
    const char* items[] = {"air_temp", "C1temp", "C1voltage", "C2temp", "C2voltage"};
    const int num_items = sizeof(items) / sizeof(items[0]);

    for (int i = 0; i < num_items; i++) {
        if (i == mode) {
            printf(HIGHLIGHT "[%s]" RESET " ", items[i]);
        } else {
            printf("[%s] ", items[i]);
        }
    }
    printf("\n\n");
}

void print_cell(){
    int temp[BATTERY_CELLS];
    double voltage[BATTERY_CELLS];
    pthread_mutex_lock(&lock);
    for (int i = 0; i < BATTERY_CELLS; i++) {
        temp[i] = battery[i].Temperature;
        voltage[i] = battery[i].voltage_terminal;
    }
    int local_status = bms_status.Status;
    int local_air_temp = bms_temperature.AirTemp;
    int local_iftempfan = iftempfan;
    pthread_mutex_unlock(&lock);

    for (int i = 0; i < BATTERY_CELLS; i++) {                       //print battery cells data
        // temperature color
        const char* temp_color = RESET;
        if (temp[i] <= -10) temp_color = BLUE;
        else if (temp[i] <= 0) temp_color = YELLOW;
        else if (temp[i] >= 45) temp_color = RED;
        else if (temp[i] >= 13) temp_color = GREEN;

        // voltage color
        const char* volt_color = RESET;
        if (voltage[i] <= 6.0) volt_color = RED;
        else if (voltage[i] <= 6.5) volt_color = YELLOW;
        else if (voltage[i] >= 8.4) volt_color = RED;
        else if (voltage[i] >= 8.0) volt_color = GREEN;

        if ((i + 1) == bms_battery_info.MaxVoltageID) volt_color = MAXHIGHLIGHT;
        if ((i + 1) == bms_battery_info.MinVoltageID) volt_color = MINHIGHLIGHT;
        if ((i + 1) == bms_temperature.MaxTempID) temp_color = MAXHIGHLIGHT;
        if ((i + 1) == bms_temperature.MinTempID) temp_color = MINHIGHLIGHT;

        printf("[C%.3d:%s%4.3d°C%s, %s%.2fv%s] ", i + 1, temp_color, temp[i], RESET, volt_color, voltage[i], RESET);

        if ( (i + 1) % CELLS_IN_LINE == 0) printf("\n");
    }
    printf("\n\n[air_temp: %d]", local_air_temp);

    if (local_status) printf(GREEN "  [charging] " RESET);
    else printf(RED "  [not charging now]" RESET);
    if (local_iftempfan == 1) printf(BLUE "  [Cooling fan active]               " RESET);
    else if (local_iftempfan == 2) printf(RED "  [Heater fan active]                " RESET);
    else printf("  [fan not activate]               " RESET);
}

void print_logo(int option) {
    if (option == 0){
        const char *logo =
            "██████╗ ███╗   ███╗███████╗\n"
            "██╔══██╗████╗ ████║██╔════╝      \n"
            "██████╔╝██╔████╔██║███████╗      \n"
            "██╔══██╗██║╚██╔╝██║╚════██║      \n"
            "██████╔╝██║ ╚═╝ ██║███████║      \n"
            "╚═════╝ ╚═╝     ╚═╝╚══════╝\n"
            "                           \n"
            "███████╗  ██╗  ███╗   ███╗     \n"
            "██╔════╝  ██║  ████╗ ████║     \n"
            "███████╗  ██║  ██╔████╔██║     \n"
            "╚════██║  ██║  ██║╚██╔╝██║     \n"
            "███████║  ██║  ██║ ╚═╝ ██║     \n"
            "╚══════╝  ╚═╝  ╚═╝     ╚═╝_ver "VERSION" \n"
            "                           \n";

        printf("%s", logo);
    }
    if (option == 1) {
        const char *logo =
        "██████╗ ███╗   ███╗███████╗\n"
        "██╔══██╗████╗ ████║██╔════╝        Use ' ← → ' arrow keys to select value you want to change\n"
        "██████╔╝██╔████╔██║███████╗        Use ' ↑ ↓ ' arrow keys to change value [increase or decrease]\n"
        "██╔══██╗██║╚██╔╝██║╚════██║        Press space to toggle charging\n"
        "██████╔╝██║ ╚═╝ ██║███████║        Press ESC twice to quit program\n"
        "╚═════╝ ╚═╝     ╚═╝╚══════╝\n"
        "                           \n"
        "███████╗  ██╗  ███╗   ███╗     \n"
        "██╔════╝  ██║  ████╗ ████║     \n"
        "███████╗  ██║  ██╔████╔██║     \n"
        "╚════██║  ██║  ██║╚██╔╝██║     \n"
        "███████║  ██║  ██║ ╚═╝ ██║     \n"
        "╚══════╝  ╚═╝  ╚═╝     ╚═╝_ver "VERSION" \n"
        "                           \n";

    printf("%s", logo);
    }
}


double get_correct(double battery_temp) {           // no mutex lock
    double correct = 0;
    if (battery_temp >= 45) correct = -0.02;
    else if (battery_temp >= 12 && battery_temp < 45) correct = 0;
    else if (battery_temp >= -10 && battery_temp < 12) correct = 0.02;
    else if (battery_temp < -10) correct = 0.04;
    return correct;
}



CAN_Message can_msgs[MAX_STRUCTS] = {
    {0x620, {0}, 8},        //bms_company_info
    {0x621, {0}, 8},        //vin_car_info
    {0x622, {0}, 6},        //bms_status
    {0x623, {0}, 6},        //bms_battery_info
    {0x624, {0}, 6},        //bms_charge_current_limits
    {0x626, {0}, 6},        //bms_soc
    {0x627, {0}, 6},        //bms_temperature
    {0x628, {0}, 6},        //bms_resistance
    {0x629, {0}, 8}         //bms_dc_charging
};

// User defined function
void refresh_CAN_container() {
    // Copy bms_structure into can sender
    // mutex already locked before calling refresh_CAN_container
    memcpy(can_msgs[0].data, &bms_company_info, 8);
    memcpy(can_msgs[1].data, &vin_car_info, 8);
    memcpy(can_msgs[2].data, &bms_status, 6);
    memcpy(can_msgs[3].data, &bms_battery_info, 6);
    memcpy(can_msgs[4].data, &bms_charge_current_limits, 6);
    memcpy(can_msgs[5].data, &bms_soc, 6);
    memcpy(can_msgs[6].data, &bms_temperature, 6);
    memcpy(can_msgs[7].data, &bms_resistance, 6);
    memcpy(can_msgs[8].data, &bms_dc_charging, 8);
}

void change_value(int mode, int ifup) {
    if (ifup) {
        switch(mode) {
            case 0:
                if (bms_temperature.AirTemp < 127) bms_temperature.AirTemp ++; break;
            case 1:
                if (battery[0].Temperature < 127) battery[0].Temperature++; break;
            case 2:
                if (battery[0].voltage_terminal < 4.2){
                    battery[0].voltage_terminal += 0.1;
                    bms_soc.SOC = SOC_from_OCV(battery[0].voltage_terminal);
                    break;
                }
            case 3:
                if (battery[1].Temperature < 127) battery[1].Temperature++; break;
            case 4:
                if (battery[1].voltage_terminal < 4.2){
                    battery[1].voltage_terminal += 0.1;
                    bms_soc.SOC = SOC_from_OCV(battery[1].voltage_terminal);
                    break;
                }
            default:
                break;
        }
    }
    else if (!ifup) {
        switch(mode) {
            case 0:
                if (bms_temperature.AirTemp > -127) bms_temperature.AirTemp --; break;
            case 1:
                if (battery[0].Temperature > -127) battery[0].Temperature--; break;
            case 2:
                if (battery[0].voltage_terminal > 2.5){
                    battery[0].voltage_terminal -= 0.1;
                    bms_soc.SOC = SOC_from_OCV(battery[0].voltage_terminal);
                    break;
                }
            case 3:
                if (battery[1].Temperature > -127) battery[1].Temperature--; break;
            case 4:
                if (battery[1].voltage_terminal > 2.5){
                    battery[1].voltage_terminal -= 0.1;
                    bms_soc.SOC = SOC_from_OCV(battery[1].voltage_terminal);break;
                }
            default:
                break;
        }
    }
}

void initializer(){
    int air_temp;
    printf("input air temp you want (℃): ");
    scanf("%d", &air_temp);
    if (air_temp < -40) air_temp = -40;
    if (air_temp > 127) air_temp = 127;

    pthread_mutex_lock(&lock);
    default_battery.coulombic_efficiency = 1.0f;
    default_battery.SOC_Initial = SOC_from_OCV(2.5); //2.5V부터 시작
    default_battery.ChargeCurrent = -0.41f;
    default_battery.noiseincurrent = 0.41f;
    default_battery.Capacity = 4.07611f;
    default_battery.capacity1c = (default_battery.Capacity * 3600)/ 100;
    default_battery.R0 = 0.00005884314f;
    default_battery.R1 = 0.01145801322f;
    default_battery.C1 = 4846.080679f;
    default_battery.voltage_delay_Initial = default_battery.ChargeCurrent * default_battery.R1 * (1 - exp(-1/default_battery.R1 * default_battery.C1));
    default_battery.voltage_delay = default_battery. voltage_delay_Initial;
    default_battery.Temperature = 25.0f;
    default_battery.SOC = default_battery.SOC_Initial;
    bms_temperature.AirTemp = air_temp;
    bms_soc.SOH = 100;
    bms_soc.Capacity = 407611;
    //초기화 단계를 거치고 수식 적용하면 될 듯 함...
    /*
    default_battery.batteryvoltage = VOLTAGE_MIN + ((VOLTAGE_MAX - VOLTAGE_MIN) * soc / 100);
    bms_soc.SOH = soh;
    batterypack.DesignedCapacity = designed_capacity;
    bms_temperature.AirTemp = air_temp;
    */
    pthread_mutex_unlock(&lock);
}

// User input thread    ||fix CAN data belongs to user input
void *input_thread(void *arg) {                                     //tid1
    char key_input = 0;
    int invalid_input = 0;
    while(ifrunning) {
        key_input = getchar();
        pthread_mutex_lock(&lock);
        switch(key_input) {
            case ' ':       //whiespace key pressed
                bms_status.Status =!bms_status.Status;
                break;
            case 'a':
                if (battery[0].Temperature > 0) battery[0].Temperature--;
                break;
            case 'A':
                if (battery[0].Temperature < 100) battery[0].Temperature++;
                break;
            case 'g':
                if (bms_soc.SOC > 0) bms_soc.SOC--;
                break;
            case 'G':
                if (bms_soc.SOC < 100) bms_soc.SOC++;
                break;
            case '\033': { // ESC || arrow key and...
                // Check if this is an arrow key sequence
                char next_char = getchar();
                if (next_char == '[') {
                    char arrow = getchar();
                    switch (arrow) {        //use change_value()
                        case UP:   // Up arrow
                            change_value(input_mode, 1); break;
                        case DOWN:   // Down arrow
                            change_value(input_mode, 0); break;
                        case RIGHT: // Right arrow
                            if (input_mode < 4) input_mode++;
                            break;
                        case LEFT: // Left arrow
                            if (input_mode > 0) input_mode--;
                            break;
                        default:
                            invalid_input = 1;
                    }
                } else {
                    // simple ESC input
                    ifrunning = 0;
                    pthread_mutex_unlock(&lock);
                    return NULL;
                }
                break;
            }
            default:
                invalid_input = 1;
                break;
        }
        pthread_mutex_unlock(&lock);
    }
}

// CAN tx thread
void *can_sender_thread(void *arg) {                                        //tid2
    const char *interface_name = (const char *)arg;
    int tx_sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    
    // Generate CAN socket
    if ((tx_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("소켓 생성 실패");
        return NULL;
    }

    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(tx_sock, SIOCGIFINDEX, &ifr) < 0) {
        printf(HIGHLIGHT);
        perror("\npress any key to continue\n인터페이스 인덱스 가져오기 실패");
        printf(RESET);
        close(tx_sock);
        pthread_mutex_lock(&lock);
        ifrunning = 0;
        pthread_mutex_unlock(&lock);
        return NULL;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(tx_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("소켓 바인드 실패");
        close(tx_sock);
        return NULL;
    }

    // CAN packet tx loop
    while (ifrunning) {
        for (int i = 0; i < MAX_STRUCTS; i++) {
            pthread_mutex_lock(&lock);
            refresh_CAN_container();
            frame.can_id = can_msgs[i].id;
            frame.can_dlc = can_msgs[i].len;
            memcpy(frame.data, can_msgs[i].data, frame.can_dlc);
            pthread_mutex_unlock(&lock);

            if (write(tx_sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                perror("CAN 패킷 전송 실패");
            }
            usleep(SLEEPTIME);
        }
    }
    
    close(tx_sock);
    return NULL;
}

void *can_receiver_thread(void *arg) {              //tid10
    const char *interface_name = (const char *)arg;
    int rx_sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    
    // Generate CAN socket
    if ((rx_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("소켓 생성 실패");
        return NULL;
    }

    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(rx_sock, SIOCGIFINDEX, &ifr) < 0) {
        printf(HIGHLIGHT);
        perror("\npress any key to continue\n인터페이스 인덱스 가져오기 실패");
        printf(RESET);
        close(rx_sock);
        pthread_mutex_lock(&lock);
        ifrunning = 0;
        pthread_mutex_unlock(&lock);
        return NULL;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(rx_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("소켓 바인드 실패");
        close(rx_sock);
        return NULL;
    }

    while (ifrunning) {
        int nbytes = read(rx_sock, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            if (frame.can_id == 0x010) {
                if (frame.data[0] == 0x01) {
                    pthread_mutex_lock(&lock);
                    bms_status.Status = 1;
                    pthread_mutex_unlock(&lock);
                }
                else if (frame.data[0] == 0x00) {
                    pthread_mutex_lock(&lock);
                    bms_status.Status = 0;
                    pthread_mutex_unlock(&lock);
                }
            }
        } else if (nbytes < 0) {
            perror("CAN 수신 실패");
            break;
        }
    }
}

void *print_screen_thread(void *arg) {              //tid3
    printf(CLEAR_SCREEN);
    printf(CURSOR_HIDE);
    while(ifrunning) {
        printf(SET_CURSOR_UL);
        pthread_mutex_lock(&lock);
        int soc = bms_soc.SOC;
        int local_input_mode = input_mode;
        pthread_mutex_unlock(&lock);
        print_logo(1);
        print_inputmode(local_input_mode);
        print_battery_bar(soc);
        print_cell();
        usleep(100000);
    }
}

void *temp_batterypack_thread(void *arg){           //tid4
    while(ifrunning){
        double mintemp = 1e9;
        int mintempid = 0;
        double maxtemp = -1e9;
        int maxtempid = 0;
        double internal_heat = 0; //충전 중 내부 열 발생
        double heater_power_w = 5.0; //히터 출력
        double cooler_power_w = 5.0; //쿨러 출력
        double totaltemps = 0;
        double totalreg0 = 0;
        double totalreg1 = 0;
        double minreg0 = 1e9;
        int minreg0id = 0;
        double maxreg0 = -1e9;
        int maxreg0id = 0;
        usleep(500000);

        pthread_mutex_lock(&lock);
        double local_air_temp = bms_temperature.AirTemp;
        for(int i=0; i<BATTERY_CELLS; i++){
            internal_heat = battery[i].R0 * pow(battery[i].ChargeCurrent,2);
            double heater_power = (battery[i].Temperature <= 15)? heater_power_w : 0;
            double cooler_power = (battery[i].Temperature >= 35)? cooler_power_w : 0;
            double totalheat = internal_heat + heater_power - cooler_power;
            if(heater_power != 0) iftempfan = 2;
            else if(cooler_power != 0) iftempfan = 1;
            else iftempfan = 0;
            battery[i].Temperature += (1.0 / 200.0 * (totalheat - (battery[i].Temperature - local_air_temp) / 3.0));
            if(mintemp > battery[i].Temperature){
                mintemp = battery[i].Temperature;
                mintempid = i + 1;
            }
            if(maxtemp < battery[i].Temperature){
                maxtemp = battery[i].Temperature;
                maxtempid = i + 1;
            }
            totaltemps += battery[i].Temperature;
        }
        bms_temperature.Temperature = (int)(totaltemps / BATTERY_CELLS);
        bms_temperature.MaxTemp = maxtemp;
        bms_temperature.MaxTempID = maxtempid;
        bms_temperature.MinTemp = mintemp;
        bms_temperature.MinTempID = mintempid;
        // 충전 과정 중 발열과 히팅/쿨링 과정 완료
        // 온도에 따른 저항 갱신 완료
        for(int i=0; i<BATTERY_CELLS; i++){
            double diff_temperature = battery[i].Temperature - 25; //셀 현재 온도 - 셀 표준 온도
            battery[i].R0 = 0.00005884314 * (1 + 0.003 * diff_temperature);
            battery[i].R1 = 0.01145801322 * (1 + 0.003 * diff_temperature);
            totalreg0 += battery[i].R0;
            totalreg1 += battery[i].R1;
            if(minreg0id > battery[i].R0){
                minreg0 = battery[i].R0;
                minreg0id = i + 1;
            }
            if(maxreg0 < battery[i].R0){
                maxreg0 = battery[i].R0;
                maxreg0id = i + 1;
            }
        }
        bms_resistance.Resistance0 = (totalreg0 / BATTERY_CELLS);
        bms_resistance.Resistance1 = (totalreg1 / BATTERY_CELLS);
        pthread_mutex_unlock(&lock);
    }
}//UpdateTemperature || UpdateResistance 병합

void *charge_batterypack_thread(void *arg){         //tid5

    while(ifrunning){
        pthread_mutex_lock(&lock);
        int local_status = bms_status.Status;
        for(int i=0; i<BATTERY_CELLS; i++) ekf.init[BATTERY_CELLS] = 0;
        pthread_mutex_unlock(&lock);
        if(local_status){
            usleep(300000);
            /*
            기존 로직은 모든 셀에 0.01만큼 전압을 올리는 형식
            0~10 중 난수가 0일 때 셀 온도를 0.5만큼 올리는 형식 - battery[i].batteryvoltage
            -> 충전 저압 갱신, 터미널 전압 측정, EKF 추정 SOC 계산 로직으로 변경할 예정
            */
            pthread_mutex_lock(&lock);
            for(int i=0; i<BATTERY_CELLS; i++){
                //SimulateTerminalVoltage
                battery[i].SOC -= 1 * 1 / battery[i].capacity1c * battery[i].ChargeCurrent;
                if(battery[i].SOC < 0.) battery[i].SOC = 0.;
                if(battery[i].SOC > 100.) battery[i].SOC = 100.;
                double tau = battery[i].R1 * battery[i].C1;
                double e = exp(-1 / tau);
                battery[i].voltage_delay = battery[i].voltage_delay * e + battery[i].R1 * (1. - e) * battery[i].ChargeCurrent;
                //수정 필요 -> 갱신 시
                double ocv = OCV_from_SOC(battery[i].SOC); //모두 구현 끝났다면 OCV_from_SOC 함수 가져오는 방법을 구현해놓자.
                battery[i].voltage_terminal = ocv - battery[i].voltage_delay - battery[i].R0 * battery[i].ChargeCurrent;
                battery[i].noiseincurrent = battery[i].ChargeCurrent;
                //SOCEKF
                if(!ekf.init[i]){ //초기 상태라면 설정하는 것
                    //F:상태 전이 행렬, Q:시스템 노이즈(예측 불확실성), P:추정 오차 공분산, Pp:예측 공분산
                    ekf.F[0][0] = 1.0; ekf.F[0][1] = 0.0;
                    ekf.F[1][0] = 0.0; ekf.F[1][1] = exp(-1 / (battery[i].R1 * battery[i].C1));
                    ekf.Q[0][0] = 0.0000001; ekf.Q[0][1] = 0.0;
                    ekf.Q[1][0] = 0.0; ekf.Q[1][1] = 0.0000001;
                    ekf.R = 500.0;
                    ekf.P[0][0] = 3000.0; ekf.P[0][1] = 0.0;
                    ekf.P[1][0] = 0.0; ekf.P[1][1] = 3000.0;
                    //초기 상태이므로 초기 데이터 적용
                    ekf.previous_vector[0] = battery[i].SOC_Initial;
                    ekf.previous_vector[1] = battery[i].voltage_delay_Initial;
                    ekf.init[i] = 1;
                }
                double soc_voltagedelay_after[2];
                //SOC, V1 예측
                soc_voltagedelay_after[0] = ekf.previous_vector[0] - 1 * 1 / battery[i].capacity1c * battery[i].noiseincurrent;
                soc_voltagedelay_after[1] = exp(-1 / (battery[i].R1 * battery[i].C1)) * ekf.previous_vector[1] + battery[i].R1 * (1.0 - exp(-1 / (battery[i].R1 * battery[i].C1))) * battery[i].noiseincurrent;
                double FP[2][2];
                for(int i=0; i<2; ++i) for(int j=0; j<2; ++j) FP[i][j] = ekf.F[i][0] * ekf.P[0][j] + ekf.F[i][1] * ekf.P[1][j];
                for(int i=0; i<2; ++i) for(int j=0; j<2; ++j) ekf.Pp[i][j] = FP[i][0] * ekf.F[j][0] + FP[i][1] * ekf.F[j][1] + ekf.Q[i][j];
                double Jacobian_vector[2];
                //ComputeJacobianH -> 측정된 전압
                //ComputeJacobianH(soc_voltagedelay_after[0], previous_vector[0], Jacobian_vector);
                double soc_hi = soc_voltagedelay_after[0] + 0.05;
                double soc_low = soc_voltagedelay_after[0] - 0.05;
                if(soc_hi > 100.0) soc_hi = 100.0;
                if(soc_low < 0.0) soc_low = 0.0;
                double ocv_hi = OCV_from_SOC(soc_hi);
                double ocv_low = OCV_from_SOC(soc_low);
                Jacobian_vector[0] = (ocv_hi - ocv_low) / (soc_hi - soc_low);
                Jacobian_vector[1] = -1.0;
                if(fabs(Jacobian_vector[0]) < 1e-4f) Jacobian_vector[0] = (Jacobian_vector[0] >= 0.0)? 1e-4f : -1e-4f;
                double HP[2];
                //HP -> H * Pp 구현
                HP[0] = Jacobian_vector[0] * ekf.Pp[0][0] + Jacobian_vector[1] * ekf.Pp[1][0];
                HP[1] = Jacobian_vector[0] * ekf.Pp[0][1] + Jacobian_vector[1] * ekf.Pp[1][1];
                //칼만 이득 분모값 구하기 H * Pp * (H^T + R) T는 역행렬
                double kalman_gain_denom = HP[0] * Jacobian_vector[0] + HP[1] * Jacobian_vector[1] + ekf.R;
                double kalman_gain[2] = {HP[0] / kalman_gain_denom, HP[1] / kalman_gain_denom};
                //residual 예측치 신뢰도 구하기
                if(soc_voltagedelay_after[0] >= 100.0) soc_voltagedelay_after[0] = 100.0;
                else if(soc_voltagedelay_after[0] <= 0.0) soc_voltagedelay_after[0] = 0.0;
                double predict_voltage_terminal = OCV_from_SOC(soc_voltagedelay_after[0]) - battery[i].voltage_delay - battery[i].R0 * battery[i].noiseincurrent;
                double y = battery[i].voltage_terminal - predict_voltage_terminal;
                ekf.estimate_SOC_Voltagedelay[0] = soc_voltagedelay_after[0] + kalman_gain[0] * y;
                ekf.estimate_SOC_Voltagedelay[1] = soc_voltagedelay_after[1] + kalman_gain[1] * y;
                if(ekf.estimate_SOC_Voltagedelay[0] < 0.0) ekf.estimate_SOC_Voltagedelay[0] = 0.0;
                else if(ekf.estimate_SOC_Voltagedelay[0] > 100.0) ekf.estimate_SOC_Voltagedelay[1] = 100.0;
                //공분산 갱신 (I - kalman_gain * H) * Pp
                double kalman_gain_h[2][2]; // K * H
                for(int i=0; i<2; ++i) for(int j=0; j<2; ++j) kalman_gain_h[i][j] = kalman_gain[i] * Jacobian_vector[j];
                double I_KH[2][2] = {{1.0 - kalman_gain_h[0][0], -kalman_gain_h[0][1]},
                                    {-kalman_gain_h[1][0], 1.0 - kalman_gain_h[1][1]}};
                double update_error[2][2];
                for(int i=0; i<2; ++i) for(int j=0; j<2; ++j) update_error[i][j] = I_KH[i][0] * ekf.Pp[0][j] + I_KH[i][1] * ekf.Pp[1][j];
                ekf.previous_vector[0] = ekf.estimate_SOC_Voltagedelay[0];
                ekf.previous_vector[1] = ekf.estimate_SOC_Voltagedelay[1];
                //SOCEKF(&cell, delta_time, &SOC_estimate, &voltage_delay_est);
                SOC_estimate = ekf.estimate_SOC_Voltagedelay[0];
                voltage_delay_est  = ekf.estimate_SOC_Voltagedelay[1];
            }
            pthread_mutex_unlock(&lock);
        }
    }
} //SimulateTerminalVoltage || SOCEKF

void *voltage_batterypack_thread(void *arg){        //tid6
    //maybe change fucntion name -> ChargeCurrentLimit
    while(ifrunning){
        double total_corrected_voltage = 0;
        double minvoltage = 1e9;
        double minvoltageid = 0;
        double maxvoltage = -1e9;
        double maxvoltageid = 0;
        usleep(100000);
        pthread_mutex_lock(&lock);
        ifvoltageerror = 0;
        const double voltage_max_charge = 4.200;
        for(int i=0; i<BATTERY_CELLS; i++){
            double charge_current_cc = -1.0 * battery[i].Capacity;
            double charge_current_min_CV = -0.05 * battery[i].Capacity;
            double voltage_hysteresis = 0.010;
            double voltage_control = 50.0; //P-gain 전압
            double charge_current_limits = charge_current_cc;
            if(SOC_estimate > 80.0){
                double SOC_TAPER_RATIO = (SOC_estimate - 80.0) / (98.0 - 80.0);
                if(SOC_TAPER_RATIO > 1.0) SOC_TAPER_RATIO = 1.0;
                charge_current_limits = charge_current_cc * (1.0 - 0.8 * SOC_TAPER_RATIO);
                //80 ~ 100% 전류 80% 감소
            }
            if(battery[i].voltage_terminal >= voltage_max_charge - voltage_hysteresis){
                charge_current_limits = charge_current_cc + voltage_control * (battery[i].voltage_terminal - voltage_max_charge);
                if(charge_current_limits < charge_current_min_CV) charge_current_limits = charge_current_min_CV;
                if(charge_current_limits > 0.0) charge_current_limits = 0.0;
            }
            if(battery[i].Temperature < 0.0) charge_current_limits = 0.0;
            else if(battery[i].Temperature < 15.0) charge_current_limits *= 0.5; //50% 제한
            battery[i].ChargeCurrent = charge_current_limits;
            battery[i].noiseincurrent = battery[i].ChargeCurrent;
            //OCV_SOC_T 구현까지하고 테스트해보자 -> Battery_t는 구현이 마친 상태임
            //dbc에 넣을 방법과 셀 개수 증가로 인한 방법 고안하자.
            ekf.estimate_SOC_Voltagedelay[0] = SOC_estimate;
            ekf.estimate_SOC_Voltagedelay[1] = voltage_delay_est;
            //최종 셀 soc, voltage
            if(minvoltage > ekf.estimate_SOC_Voltagedelay[1]){
                minvoltage = ekf.estimate_SOC_Voltagedelay[1];
                minvoltageid = i + 1;
            }
            if(maxvoltage < ekf.estimate_SOC_Voltagedelay[1]){
                maxvoltage = ekf.estimate_SOC_Voltagedelay[1];
                maxvoltageid = i + 1;
            }
            //voltage or soc error 발생 이유가 뭐지?
            if(ekf.estimate_SOC_Voltagedelay[1] > VOLTAGE_MAX || ekf.estimate_SOC_Voltagedelay[1] < VOLTAGE_MIN) ifvoltageerror = 1;
            else ifvoltageerror = 0;
        } //셀 추가 시 +로 변경 후 셀 개수 만큼 나누면 될 듯 함... + estimate_SOC_Voltage[BatSIZE][1]
        if(ekf.estimate_SOC_Voltagedelay[0] >= 100.0) ekf.estimate_SOC_Voltagedelay[0] = 100;
        if(ekf.estimate_SOC_Voltagedelay[0] <= 0.0) ekf.estimate_SOC_Voltagedelay[0] = 0; 
        bms_soc.SOC = (uint8_t)ekf.estimate_SOC_Voltagedelay[0];
        //bms_soc.DOD =
        bms_battery_info.Voltage = (uint16_t)(ekf.estimate_SOC_Voltagedelay[1]);
        bms_battery_info.MinVoltage = (uint8_t)(minvoltage * 10);
        bms_battery_info.MinVoltageID = minvoltageid;
        bms_battery_info.MaxVoltage = (uint8_t)(maxvoltage * 10);
        bms_battery_info.MaxVoltageID = maxvoltageid;
        pthread_mutex_unlock(&lock);
    }
}//ChargeCurrent || 

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, 0);  // turn off buffering for stdout, print screen instantly (without enter)

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <can_interface>\n", argv[0]);
        return 1;
    }

    printf(CLEAR_SCREEN);              // clear whole screen
    printf(SET_CURSOR_UL);             // set cursor UpLeft
    print_logo(0);
    printf("\n\n");
    initializer();

    init_battery_array();
    printf("waiting for start .");
    usleep(1000000);
    printf("\rwaiting for start ..");
    usleep(300000);
    printf("\rwaiting for start ...");
    usleep(700000); //완료

    //bms_soc.Capacity = batterypack.DesignedCapacity;

    // Get input without buffer ('\n')
    struct termios newt, oldt;
    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);
    // Apply new settings immediately
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    pthread_t tid1, tid2, tid3, tid4, tid5, tid6, tid7;
    
    pthread_mutex_init(&lock, NULL);
    
    // start InputThread && CANtxThread
    pthread_create(&tid1, NULL, input_thread, NULL); //키보드 입력 변환
    pthread_create(&tid2, NULL, can_sender_thread, argv[1]); //can frame을 can_msgs에 저장 후 주기적 전송
    pthread_create(&tid3, NULL, can_receiver_thread, argv[1]); //can message 수신
    pthread_create(&tid4, NULL, print_screen_thread, NULL); //화면 갱신
    /*
        온도 갱신
        온도 기반 저항 갱신
        충전 전압 갱신
        터미널 전압 측정
        EKF 추정 SOC 계산
        온도 기반 충전 전류 제한
        추정 최종 SOC, 터미널 전압 갱신

        -> 현재 주어진 스레드
            charge_batterypack_thread : 충전 상태 전압, 온도 증가 로직 포함
            temp_batterypack_thread : 외기 온도, 배터리 온도 차 반영 쿨링/히팅 로직 포함
            voltage_batterypack_thread : Voltage 보정 평균 전압 계산 로직 포함
        -> 스레드 순서
            temp_batterypack_thread : 외기 온도, 배터리 온도 차 반영 쿨링/히팅 로직 포함 -> 온도 갱신, 온도 기반 저항 갱신
            charge_batterypack_thread : 충전 저압 갱신, 터미널 전압 측정, EKF 추정 SOC 계산
            voltage_baytterypack_thread : 온도 기반 충전 전류 제한, 최종 SOC,터미널 전압 갱신
    */
    pthread_create(&tid5, NULL, temp_batterypack_thread, NULL); //외기 온도, 배터리 온도 차 반영 쿨링/히팅 로직 (구현)
    pthread_create(&tid6, NULL, charge_batterypack_thread, NULL); //충전 상태 전압, 온도 증가 로직 (구현)
    pthread_create(&tid7, NULL, voltage_batterypack_thread, NULL); //voltage 보정 평균 전압 계산 (구현)

    // Main Thread wait for both threads
    pthread_join(tid1, NULL);
    pthread_join(tid2, NULL);
    pthread_join(tid3, NULL);
    pthread_join(tid4, NULL);
    pthread_join(tid5, NULL);
    pthread_join(tid6, NULL);
    pthread_join(tid7, NULL);

    pthread_mutex_destroy(&lock);
    printf(CURSOR_SHOW);
    printf("\n");

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return 0;
}
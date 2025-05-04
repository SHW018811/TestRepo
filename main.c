
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

/*================================================================
functions for print screen
=================================================================*/


void init_battery_array() {
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
        temp[i] = battery[i].batterytemp;
        voltage[i] = battery[i].batteryvoltage;
    }
    int local_status = bms_status.Status;
    int local_air_temp = bms_temperature.AirTemp;
    int local_iftempfan = iftempfan;
    pthread_mutex_unlock(&lock);

    for (int i = 0; i < BATTERY_CELLS; i++) {                       //print battery cells data
        // temperature color
        const char* temp_color = RESET;
        if (temp[i] <= 0) temp_color = BLUE;
        else if (temp[i] <= 5) temp_color = YELLOW;
        else if (temp[i] >= 35) temp_color = RED;
        else if (temp[i] >= 13) temp_color = GREEN;

        // voltage color
        const char* volt_color = RESET;
        if (voltage[i] <= 2.5) volt_color = RED;
        else if (voltage[i] <= 3.0) volt_color = YELLOW;
        else if (voltage[i] >= 4.2) volt_color = RED;
        else if (voltage[i] >= 4.0) volt_color = GREEN;

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
}//Voltage correction -> resistance?



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
                if (battery[0].batterytemp < 127) battery[0].batterytemp++; break;
            case 2:
                if (battery[0].batteryvoltage < 9.0) battery[0].batteryvoltage += 0.1; break;
            case 3:
                if (battery[1].batterytemp < 127) battery[1].batterytemp++; break;
            case 4:
                if (battery[1].batteryvoltage < 9.0) battery[1].batteryvoltage += 0.1; break;
            default:
                break;
        }
    }
    else if (!ifup) {
        switch(mode) {
            case 0:
                if (bms_temperature.AirTemp > -127) bms_temperature.AirTemp --; break;
            case 1:
                if (battery[0].batterytemp > -127) battery[0].batterytemp--; break;
            case 2:
                if (battery[0].batteryvoltage > 5.5) battery[0].batteryvoltage -= 0.1; break;
            case 3:
                if (battery[1].batterytemp > -127) battery[1].batterytemp--; break;
            case 4:
                if (battery[1].batteryvoltage > 5.5) battery[1].batteryvoltage -= 0.1; break;
            default:
                break;
        }
    }
}

float SOC_from_OCV(float ocv) {
    if (ocv <= CHG_OCV[0]) return CHG_SOC[0];
    if (ocv >= CHG_OCV[OCV_SOC_T_SIZE-1]) return CHG_SOC[OCV_SOC_T_SIZE-1];
    int i = 0;
    while (i < OCV_SOC_T_SIZE - 1 && CHG_OCV[i+1] < ocv) i++;
    //선형 보간으로 테이블 값들보다 저 정확하게 사이 값을 계산해주는 함수
    float ocv_low  = CHG_OCV[i];
    float ocv_high = CHG_OCV[i+1];
    float soc_low  = CHG_SOC[i];
    float soc_high = CHG_SOC[i+1];
    float t = (ocv - ocv_low) / (ocv_high - ocv_low);
    return (soc_low + t * (soc_high - soc_low));
}

void initializer(){
    pthread_mutex_lock(&lock);
    default_battery.coulombic_efficiency = 1.0f;
    default_battery.SOC = 5.0f; // 5% 초기 SOC
    default_battery.ChargeCurrent = -0.41f;
    default_battery.noiseincurrent = default_battery.ChargeCurrent;
    default_battery.DesignedCapacity = 4.07611f;
    default_battery.Resistance0 = 0.00005884314f;
    default_battery.Resistance1 = 0.01145801322f;
    default_battery.C1 = 4846.080679f;
    default_battery.voltage_delay = default_battery.ChargeCurrent * default_battery.Resistance1 * (1 - exp(-1.0f / (default_battery.Resistance1 * default_battery.C1)));
    default_battery.batteryvoltage = SOC_from_OCV(default_battery.SOC) - default_battery.Resistance0 * default_battery.batterycurrent;
    default_battery.batterycurrent = default_battery.ChargeCurrent;
    default_battery.batterytemp = 25.0f;
    default_battery.Temperature = 25.0f;
    bms_temperature.AirTemp = 25;
    bms_soc.SOH = 100;
    pthread_mutex_unlock(&lock);

    bms_soc.Capacity = batterypack.DesignedCapacity * ((double)bms_soc.SOH / 100);
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
                if (battery[0].batterytemp > 0) battery[0].batterytemp--;
                break;
            case 'A':
                if (battery[0].batterytemp < 100) battery[0].batterytemp++;
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

void *charge_batterypack_thread(void *arg) {            //tid4
    srand(time(NULL));
    while (ifrunning) {
        pthread_mutex_lock(&lock);
        int local_status = bms_status.Status;
        pthread_mutex_unlock(&lock);
        if (local_status) {
            usleep (300000);
            //choose random chance
            int random = 0;
            random = rand() % RANDOM_PERCENT;
            pthread_mutex_lock(&lock);
            //increase voltage
            for (int i = 0 ; i < BATTERY_CELLS; i++) {
                battery[i].batteryvoltage += 0.01;
            }
            //randomly increase temp for a few random cells
            for (int j = 0; j < BATTERY_CELLS; j++) {
                if ((rand() % RANDOM_PERCENT) == 0) {
                    battery[j].batterytemp += 0.5;
                }
            }
            pthread_mutex_unlock(&lock);
        } else {
            usleep(SLEEPTIME);
        }
    }
}

void *temp_batterypack_thread(void *arg) {              //tid5
    while(ifrunning) {                                  //every logics work on runtime, always. (if there's any input or not)ㅋ
        double mintemp = 1e9;
        int mintempid = 0;
        double maxtemp = -1e9;
        int maxtempid = 0;
        double totaltemps = 0;
        usleep(500000);
        pthread_mutex_lock(&lock);
        double local_air_temp = bms_temperature.AirTemp;
        for (int i = 0; i < BATTERY_CELLS; i++) {
            if (mintemp > battery[i].batterytemp) {
                mintemp = battery[i].batterytemp;
                mintempid = i + 1;
            }
            if (maxtemp < battery[i].batterytemp) {
                maxtemp = battery[i].batterytemp;
                maxtempid = i + 1;
            }
            totaltemps += battery[i].batterytemp;

            double temp_gap = local_air_temp - battery[i].batterytemp;
            if (bms_temperature.Temperature > 46) {
                battery[i].batterytemp -= 0.5;
                iftempfan = 1;            //cooler fan
            }
            else if (bms_temperature.Temperature < -11) {
                battery[i].batterytemp += 0.5;
                iftempfan = 2;      //heater fan
            }
            else iftempfan = 0;
            battery[i].batterytemp += (temp_gap / 20);
        }
        bms_temperature.Temperature = (totaltemps / BATTERY_CELLS);     //get average temps
        bms_temperature.MaxTemp = maxtemp;
        bms_temperature.MaxTempID = maxtempid;
        bms_temperature.MinTemp = mintemp;
        bms_temperature.MinTempID = mintempid;
        pthread_mutex_unlock(&lock);
    }
}

void *voltage_batterypack_thread(void *arg) {                   //tid6
    while (ifrunning) {
        double total_corrected_voltages = 0;
        double minvoltage = 1e9;
        double minvoltageid = 0;
        double maxvoltage = -1e0;
        double maxvoltageid = 0;
        usleep(100000);
        pthread_mutex_lock(&lock);
        ifvoltageerror = 0;
        for (int i = 0; i < BATTERY_CELLS; i++){
            // Get the calibration adjustment based on the battery cell's temperature
            double corrected_voltage = battery[i].batteryvoltage + get_correct(battery[i].batterytemp);
            if (minvoltage > corrected_voltage) {
                minvoltage = corrected_voltage;
                minvoltageid = i + 1;
            }
            if (maxvoltage < corrected_voltage) {
                maxvoltage = corrected_voltage;
                maxvoltageid = i + 1;
            }
            total_corrected_voltages += corrected_voltage;
            if (corrected_voltage > VOLTAGE_MAX || corrected_voltage < VOLTAGE_MIN) ifvoltageerror = 1;
        }
        //percent = ((value - min) / (max - min)) * 100.0
        int percent = (int)ceil(((total_corrected_voltages / BATTERY_CELLS) - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100.0);
        if (percent > 100) percent = 100;
        if (percent < 0) percent = 0;
        bms_soc.SOC = percent;
        bms_soc.DOD = bms_soc.Capacity * ((double)(100 - percent) / 100);
        bms_battery_info.Voltage = (uint16_t)(total_corrected_voltages / 2);
        bms_battery_info.MinVoltage = (uint8_t)(minvoltage * 10);
        bms_battery_info.MinVoltageID = minvoltageid;
        bms_battery_info.MaxVoltage = (uint8_t)(maxvoltage * 10);
        bms_battery_info.MaxVoltageID = maxvoltageid;
        pthread_mutex_unlock(&lock);
    }
}

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
    usleep(700000);
    
    bms_soc.Capacity = batterypack.DesignedCapacity * ((double)bms_soc.SOH / 100);

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
    pthread_create(&tid1, NULL, input_thread, NULL);
    pthread_create(&tid2, NULL, can_sender_thread, argv[1]);
    pthread_create(&tid3, NULL, can_receiver_thread, argv[1]);
    pthread_create(&tid4, NULL, print_screen_thread, NULL);
    pthread_create(&tid5, NULL, charge_batterypack_thread, NULL);
    pthread_create(&tid6, NULL, temp_batterypack_thread, NULL);
    pthread_create(&tid7, NULL, voltage_batterypack_thread, NULL);

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
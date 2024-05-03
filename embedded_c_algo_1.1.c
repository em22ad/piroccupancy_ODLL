#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unitstd.h>
#include <windows.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define DIST "EUC"
#define FEATS 345
#define OBS_DUR 20
#define OBS_DUR_SH 7
#define TARGET_CLASS 0
#define OBS4PROC 5

int init_serial_unix();
HANDLE init_serial_win();
double GetPIRVpp(double *data, int size);
void consolidate(double *xData, int xData_size, double *zData, int zData_size, int *bData, int bData_size, int *cData, int cData_size);
void get_nrfeat_window(double *wnd_ts, int wnd_ts_size, double *wnd_adc1, int wnd_adc1_size, int *wnd_pir, int wnd_pir_size, int *wnd_drv, int wnd_drv_size, double *vmax1, double *vmin1, double *vmean1, double *vstd1, double *hpu1, double *hpd1);
int window_finder(int drv, int drv_p, int up_flag, int down_flag, int ctr, int trig, int dur);
void observation_processor(double ts_c, int drv_c, double adc1_c, int pir_c, double *buf_ts, int *buf_ts_size, int *buf_drv, int *buf_drv_size, double *buf_adc1, int *buf_adc1_size, int *buf_pir, int *buf_pir_size, int *up, int *down, int *ctr, int *shutter_o, int label);
double cosine_distance(double *p1, double *p2, int size);
double euclidean_distance(double *p1, double *p2, int size);
int classify_knn(double **training_data, int training_data_rows, int training_data_cols, double *unknown, int k);

int main() {
    float pair_data[1];
    int drv_c, head1 = 0, head2 = 0, head3 = 0;
    int *drv = NULL, *pir = NULL;
    float *adc1_r = NULL;
	time_t *ts = NULL;
    time_t ts_st=0, ts_end=0;
    int obs = 0;
    int detectR = 0; //0="Unoccupied(PIR)", 1="Occupied(PIR)"
    time_t last_sec = 1609459200;

    //drv = (int*)malloc(OBS_DUR * sizeof(int));
    //pir = (int*)malloc(OBS_DUR * sizeof(int));
    //adc1_r = (float*)malloc(OBS_DUR * sizeof(float));
    //ts = (float*)malloc(OBS_DUR * sizeof(time_t));

	HANDLE fd=init_serial_win();
	int fd=init_serial_unix();
    while (1) {
        unsigned char byte;
		read(fd, &byte, 1); 
		int data = (int)byte;  // convert to integer
        head1 = head2;
        head2 = head3;
        head3 = data;

        if (round(ts_end - ts_st) != last_sec) {
            printf("%ld ", round(ts_end - ts_st));
        }
        last_sec = round(ts_end - ts_st);

        if (double(ts_end - ts_st) >= OBS_DUR) {
            obs=obs+1;
            int i;
            for (i = 5; i < 100; i++) {
                adc1_r[i - 5] = adc1_r[i];
                drv[i - 5] = drv[i];
                pir[i - 5] = pir[i];
                ts[i - 5] = ts[i];
            }

            for (i = 0; i < 100; i++) {
                adc1_r[i] /= 200.0;
            }

            // Detection algorithm
            int act_pir = 0;
            int tot_pir = sizeof(pir) / sizeof(pir[0]);
            for (int i = 0; i < tot_pir; i++) {
                act_pir += pir[i];
            }

            if (act_pir > 0.05 * tot_pir) {
                detectR=1;
                for (int i = 0; i < sizeof(pir) / sizeof(pir[0]); i++) {
                    pir[i] = 1;
                }
            } else {
                for (int i = 0; i < sizeof(pir) / sizeof(pir[0]); i++) {
                    pir[i] = 0;
                }
            }

            consolidate(ts, adc1_r, pir, drv);
        }

        if (detectR != 1) {
            int up = 1, down = 1, ctr = 0, shutter_o = 0;
            float buf_ts[1], buf_adc1[1], buf_pir[1], buf_drv[1];
            for (int i = 0; i < sizeof(drv) / sizeof(drv[0]); i++) {
                observation_processor(ts[i], drv[i], adc1_r[i], pir[i], buf_ts, buf_drv, buf_adc1, buf_pir, &up, &down, &ctr, &shutter_o, TARGET_CLASS);
            }
        }

        if (detectR == 1) {
            printf("IO_pinPIR,GPIO.HIGH\n");
        } else {
            printf("IO_pinPIR,GPIO.LOW\n");
        }

        // Reset the arrays
        for (int i = 0; i < sizeof(drv) / sizeof(drv[0]); i++) {
            drv[i] = 0;
            pir[i] = 0;
            adc1_r[i] = 0;
            ts[i] = 0;
        }

        pair_data[0] = 0;
        head1 = 0;
        head2 = 0;
        head3 = 0;
        detectR=0;
        ts_st = 0;
        ts_end = 0;
        last_sec = 1609459200;
		
		if (head2 == 13 && head3 == 10) {
            // Get the current time
			time_t dt_c;
			time(&dt_c);
			// Convert to local time
			struct tm *t_c = localtime(&dt_c);

			// Format the datetime as a string
			char formatted_time[20];
			strftime(formatted_time, sizeof(formatted_time), "%Y-%m-%d %H:%M:%S", t_c);
			printf("Formatted local time: %s\n", formatted_time);

			// Convert local time to UTC
			struct tm *ts_gm = gmtime(&ct_c);
			// Get the Unix timestamp from the UTC time
			time_t ts_c = mktime(ts_gm);
			printf("UTC timestamp: %ld\n", ts_c);
			
			if (ts_st == 0) {
                ts_st = ts_c;
            }
            ts_end = ts_c;

            ts[0] = ts_c;
            if (pair_data[1] > 0) {
                drv_c = 1;
            } else {
                drv_c = 0;
            }
			
            // Append to arrays
            ts = realloc(ts, (OBS_DUR + 1) * sizeof(float));
            ts[OBS_DUR] = ts_c;

            drv = realloc(drv, (OBS_DUR + 1) * sizeof(int));
            drv[OBS_DUR] = drv_c;

            adc1_r = realloc(adc1_r, (OBS_DUR + 1) * sizeof(float));
            adc1_r[OBS_DUR] = (pair_data[3] * 256 + pair_data[4]);

            pir = realloc(pir, (OBS_DUR + 1) * sizeof(int));
            pir[OBS_DUR] = pair_data[6];
        }
    }
    // Close serial port
    CloseHandle(fd);
    //close(fd);

    return 0;
}
int init_serial_unix()
{
	int fd;
    struct termios tty;

    // Open the serial port
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        //fprintf(stderr, "Error opening /dev/ttyUSB0: %s\n", strerror(errno));
        return 1;
    }

    // Get current serial port settings
    if (tcgetattr(fd, &tty) != 0) {
        //fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        close(fd);
        return 1;
    }

    // Set Baud Rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Setting other Port Stuff
    tty.c_cflag &= ~PARENB;         // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;        // no flow control
    tty.c_cc[VMIN]  = 1;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

    // Make raw
    cfmakeraw(&tty);

    // Flush Port, then applies attributes
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        close(fd);
        return 1;
    }

    //printf("Serial port /dev/ttyUSB0 opened and configured successfully.\n");
}
HANDLE init_serial_win()
{ 
    HANDLE hSerial;
	DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};

    // Open the serial port
    hSerial = CreateFile("COM1", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error opening serial port\n");
        return 1;
    }

    // Set device parameters (115200 baud, 1 start bit, 1 stop bit, no parity)
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Error getting device state\n");
        CloseHandle(hSerial);
        return 1;
    }

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Error setting device parameters\n");
        CloseHandle(hSerial);
        return 1;
    }

    // Set timeouts
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        fprintf(stderr, "Error setting timeouts\n");
        CloseHandle(hSerial);
        return 1;
    }

    // Flush the input buffer
    if (!PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_RXABORT)) {
        fprintf(stderr, "Error flushing the serial port\n");
        CloseHandle(hSerial);
        return 1;
    }

    printf("Serial port COM1 opened and configured successfully.\n");

    return hSerial;
}
double GetPIRVpp(double *data, int size) {
    double vpp = data[160] - data[0];
    for (int i = 161; i < size; i++) {
        if (data[i] > data[160]) {
            vpp = data[i] - data[160];
        } else if (data[i] < data[0]) {
            vpp = data[160] - data[i];
        }
    }
    return vpp;
}
//////////////////
void consolidate(double *xData, int xData_size, double *zData, int zData_size, int *bData, int bData_size, int *cData, int cData_size) {
    double *xDataloc = (double *)malloc(xData_size * sizeof(double));
    double *zDataloc = (double *)malloc(zData_size * sizeof(double));
    int *bDataloc = (int *)malloc(bData_size * sizeof(int));
    int *cDataloc = (int *)malloc(cData_size * sizeof(int));

    memcpy(xDataloc, xData, xData_size * sizeof(double));
    memcpy(zDataloc, zData, zData_size * sizeof(double));
    memcpy(bDataloc, bData, bData_size * sizeof(int));
    memcpy(cDataloc, cData, cData_size * sizeof(int));

    int size = xData_size;
    int firstrun = 0;
    int ctr = 0;
    double sum2, sum4, sum5;
    double ts;

    xData_size = 0;
    zData_size = 0;
    bData_size = 0;
    cData_size = 0;

    for (int i = 1; i < size; i++) {
        if (round(xDataloc[i]) == round(xDataloc[i - 1])) {
            if (firstrun == 0) {
                firstrun = 1;
                sum2 = zDataloc[i - 1] + zDataloc[i];
                sum4 = bDataloc[i - 1] + bDataloc[i];
                sum5 = cDataloc[i - 1] + cDataloc[i];
                ts = round(xDataloc[i - 1]);
                ctr = 2;
            } else {
                sum2 += zDataloc[i];
                sum4 += bDataloc[i];
                sum5 += cDataloc[i];
                ctr++;
            }
        } else {
            if (ctr > 0) {
                firstrun = 0;
                xData[xData_size++] = ts;
                zData[zData_size++] = sum2 / ctr;
                if ((sum4 / ctr) > 0) {
                    bData[bData_size++] = 1;
                } else {
                    bData[bData_size++] = 0;
                }
                if ((sum5 / ctr) > 0) {
                    cData[cData_size++] = 1;
                } else {
                    cData[cData_size++] = 0;
                }
            }
        }
    }

    free(xDataloc);
    free(zDataloc);
    free(bDataloc);
    free(cDataloc);
}
///////////////////
void get_nrfeat_window(int wnd_ts_len, float *wnd_ts, float *wnd_adc1, float *wnd_pir, float *wnd_drv,
                       float *vmax1, float *vmin1, float *vmean1, float *vstd1, float *hpu1, float *hpd1) {
    int x_tam = wnd_ts_len;
    int xx_tam = wnd_ts_len;
    float *xts = (float *)malloc(xx_tam * sizeof(float));
    float *yadc1 = (float *)malloc(xx_tam * sizeof(float));
    float *ypir = (float *)malloc(xx_tam * sizeof(float));
    float *ydrv = (float *)malloc(xx_tam * sizeof(float));

    // Set up data points for interpolation
    for (int i = 0; i < x_tam; i++) {
        xts[i] = wnd_ts[i];
    }

    // Interpolate
    for (int i = x_tam; i < xx_tam; i++) {
        xts[i] = xts[i - 1] + 1;
    }

    interp_linear(wnd_ts, wnd_adc1, x_tam, xts, yadc1, xx_tam);
    interp_linear(wnd_ts, wnd_pir, x_tam, xts, ypir, xx_tam);
    interp_linear(wnd_ts, wnd_drv, x_tam, xts, ydrv, xx_tam);

    *vmax1 = max_arr(yadc1, xx_tam);
    *vmin1 = min_arr(yadc1, xx_tam);
    *vmean1 = *vmax1 - *vmin1;
    *vstd1 = std_dev(yadc1, xx_tam);

    *hpu1 = 0;
    *hpd1 = 0;

    int flagup = (ydrv[0] >= 1) ? 1 : 0;
    int flagdn = (ydrv[0] < 1) ? 1 : 0;
    int flipup = 0, flipdn = 0;
    float *subwu = (float *)malloc(xx_tam * sizeof(float));
    float *subwd = (float *)malloc(xx_tam * sizeof(float));
    int subwu_len = 0, subwd_len = 0;

    for (int i = 0; i < xx_tam - 1; i++) {
        if (flagup) {
            subwu[subwu_len++] = yadc1[i];
        }
        if (flagdn) {
            subwd[subwd_len++] = yadc1[i];
        }

        if (ydrv[i] < 1 && ydrv[i + 1] >= 1) {
            flipup = 1;
            flagup = 1;
            flagdn = 0;
            flipdn = 0;
        } else if (ydrv[i] > 0 && ydrv[i + 1] <= 0) {
            flipdn = 1;
            flagdn = 1;
            flagup = 0;
            flipup = 0;
        }

        if (flipdn) {
            if (subwu_len > 1) {
                float subwu_min = min_arr(subwu, subwu_len);
                float subwu_max = max_arr(subwu, subwu_len);
                float *subwu_norm = (float *)malloc(subwu_len * sizeof(float));
                normalize_arr(subwu, subwu_len, subwu_min, subwu_max, subwu_norm);
                float submax = max_arr(subwu_norm, subwu_len);
                float submin = min_arr(subwu_norm, subwu_len);
                float diff = submax - subwu_norm[0];
                *hpu1 = (*hpu1 + diff) / 2.0;
                free(subwu_norm);
            }
            subwu_len = 0;
            flipdn = 0;
            flagup = 0;
        }

        if (flipup) {
            if (subwd_len > 1) {
                float subwd_min = min_arr(subwd, subwd_len);
                float subwd_max = max_arr(subwd, subwd_len);
                float *subwd_norm = (float *)malloc(subwd_len * sizeof(float));
                normalize_arr(subwd, subwd_len, subwd_min, subwd_max, subwd_norm);
                float submin = min_arr(subwd_norm, subwd_len);
                float submax = max_arr(subwd_norm, subwd_len);
                int idxmin = 0, idxmax = 0;
                for (int ii = 0; ii < subwd_len; ii++) {
                    if (subwd_norm[ii] <= 0.7 * submin) {
                        idxmin = ii;
                    }
                    if (subwd_norm[ii] >= 0.7 * submax) {
                        idxmax = ii;
                    }
                }
                *hpd1 = fminf(idxmin, idxmax);
                free(subwd_norm);
            }
            subwd_len = 0;
            flipup = 0;
            flagdn = 0;
        }
    }

    free(xts);
    free(yadc1);
    free(ypir);
    free(ydrv);
    free(subwu);
    free(subwd);
}

void interp_linear(float *x, float *y, int x_len, float *x_new, float *y_new, int x_new_len) {
    int i, j;
    for (i = 0; i < x_new_len; i++) {
        if (x_new[i] <= x[0]) {
            y_new[i] = y[0];
        } else if (x_new[i] >= x[x_len - 1]) {
            y_new[i] = y[x_len - 1];
        } else {
            for (j = 0; j < x_len - 1; j++) {
                if (x_new[i] >= x[j] && x_new[i] <= x[j + 1]) {
                    y_new[i] = y[j] + (y[j + 1] - y[j]) * (x_new[i] - x[j]) / (x[j + 1] - x[j]);
                    break;
                }
            }
        }
    }
}

float max_arr(float *arr, int len) {
    float max_val = arr[0];
    for (int i = 1; i < len; i++) {
        if (arr[i] > max_val) {
            max_val = arr[i];
        }
    }
    return max_val;
}

float min_arr(float *arr, int len) {
    float min_val = arr[0];
    for (int i = 1; i < len; i++) {
        if (arr[i] < min_val) {
            min_val = arr[i];
        }
    }
    return min_val;
}

// Function to calculate the standard deviation of an array
float std_dev(float *arr, int n) {
    float sum = 0.0, mean, std_dev = 0.0;

    // Calculate the mean
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    mean = sum / n;

    // Calculate the standard deviation
    for (int i = 0; i < n; i++) {
        std_dev += pow(arr[i] - mean, 2);
    }
    std_dev = sqrt(std_dev / (n - 1));

    return std_dev;
}

// Function to normalize an array
void normalize_arr(float *arr, int n, float min_val, float max_val, float *arr_norm) {
    for (int i = 0; i < n; i++) {
        arr_norm[i] = (arr[i] - min_val) / (max_val - min_val);
    }
}
///////////////
int window_finder(int drv, int drv_p, int up_flag, int down_flag, int ctr, int trig, int dur) {
    if (drv - drv_p >= 0.5) {
        if (up_flag == 0) {
            up_flag = 1;
        }
        if (up_flag == 1) {
            ctr = ctr + 1;
        }
    } else {
        down_flag = 1;
        if (down_flag == 1) {
            if (ctr >= OBS_DUR_SH) {
                trig = 1;
                dur = OBS_DUR_SH;
                ctr = 0;
                up_flag = 0;
            }
        }
    }
    return trig, dur, up_flag, down_flag, ctr;
}
///////////////
void observation_processor(float ts_c, int drv_c, float adc1_c, int pir_c, float *buf_ts, int *buf_drv, float *buf_adc1, int *buf_pir, int up, int down, int ctr, int shutter_o, int label) {
    int trig = 0;
    int dur = -1;

    if (buf_ts != NULL && buf_ts->size > 0) {
        window_finder(drv_c, buf_drv[buf_drv->size - 1], &up, &down, &ctr, &trig, &dur);
    }

    if (drv_c > 0) {
        shutter_o = 1;
    }

    if (shutter_o == 1) {
        buf_ts->push_back(ts_c);
        buf_adc1->push_back(adc1_c);
        buf_pir->push_back(pir_c);
        buf_drv->push_back(drv_c);
    }

    if (trig == 1) {
        trig = 0;
        shutter_o = 0;

        int ST = 0;
        std::vector<float> wnd_ts(buf_ts->begin() + ST, buf_ts->end());
        std::vector<float> wnd_adc1(buf_adc1->begin() + ST, buf_adc1->end());
        std::vector<int> wnd_pir(buf_pir->begin() + ST, buf_pir->end());
        std::vector<int> wnd_drv(buf_drv->begin() + ST, buf_drv->end());

        float vmax1, vmin1, vmean1, vstd1, hpu1, hpd1;
        get_nrfeat_window(wnd_ts, wnd_adc1, wnd_pir, wnd_drv, &vmax1, &vmin1, &vmean1, &vstd1, &hpu1, &hpd1);

        if (label == 1) {
            std::ofstream file("./occfeat_output.txt", std::ios::app);
            file << wnd_ts[wnd_ts.size() - 1] << "," << vmax1 << "," << vmin1 << "," << vmean1 << "," << vstd1 << "," << hpu1 << "," << hpd1 << std::endl;
            file.close();
        } else {
            std::ofstream file("./uoccfeat_output.txt", std::ios::app);
            file << wnd_ts[wnd_ts.size() - 1] << "," << vmax1 << "," << vmin1 << "," << vmean1 << "," << vstd1 << "," << hpu1 << "," << hpd1 << std::endl;
            file.close();
        }

        buf_ts->clear();
        buf_adc1->clear();
        buf_pir->clear();
        buf_drv->clear();
    }
}
/////////////////
double cosine_distance(double *p1, double *p2, int n) {
    double dot_product = 0.0;
    double norm_p1 = 0.0;
    double norm_p2 = 0.0;

    for (int i = 0; i < n; i++) {
        dot_product += p1[i] * p2[i];
        norm_p1 += p1[i] * p1[i];
        norm_p2 += p2[i] * p2[i];
    }

    norm_p1 = sqrt(norm_p1);
    norm_p2 = sqrt(norm_p2);

    if (norm_p1 == 0.0 || norm_p2 == 0.0) {
        return 1.0; // Cosine distance is 1.0 when one of the vectors is the zero vector
    }

    return 1.0 - (dot_product / (norm_p1 * norm_p2));
}
////////////////
double euclidean_distance(double *p1, double *p2, int n) {
    double sum_squares = 0.0;

    for (int i = 0; i < n; i++) {
        sum_squares += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    }

    return sqrt(sum_squares);
}
////////////////
int classify_knn(double **training_data, double *unknown, int k, int num_features, int num_training_samples, char **labels, char *dist_type) {
    double *distances = (double *)malloc(num_training_samples * sizeof(double));
    int *indices = (int *)malloc(k * sizeof(int));
    int *votes = (int *)calloc(2, sizeof(int)); // Assuming binary classification (0 or 1)

    // Calculate distances between unknown sample and training samples
    for (int i = 0; i < num_training_samples; i++) {
        if (strcmp(dist_type, "EUC") == 0) {
            distances[i] = euclidean_distance(training_data[i], unknown, num_features);
        } else {
            distances[i] = cosine_distance(training_data[i], unknown, num_features);
        }
    }

    // Sort the distances and get the indices of the k nearest neighbors
    for (int i = 0; i < k; i++) {
        int min_idx = 0;
        double min_dist = distances[0];
        for (int j = 1; j < num_training_samples - i; j++) {
            if (distances[j] < min_dist) {
                min_idx = j;
                min_dist = distances[j];
            }
        }
        indices[i] = min_idx;
        distances[min_idx] = INFINITY; // Mark the current minimum as visited
    }

    // Count the votes for the k nearest neighbors
    for (int i = 0; i < k; i++) {
        votes[(int)atoi(labels[indices[i]])]++;
    }

    // Determine the most common label
    int max_votes = 0;
    int predicted_label = -1;
    for (int i = 0; i < 2; i++) {
        if (votes[i] > max_votes) {
            max_votes = votes[i];
            predicted_label = i;
        }
    }

    free(distances);
    free(indices);
    free(votes);
    return predicted_label;
}
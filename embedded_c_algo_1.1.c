#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define PIR_SENST 0.4 //less is more sensitive digital PIR
#define DIST "EUC"
#define FEATS 345
#define OBS_DUR 20
#define OBS_DUR_SH 7
#define TARGET_CLASS 0
#define OBS4PROC 5
#define FLT_MAX 3.402823466e+38F// max value

int init_serial_unix()
{
    int fd;
    struct termios tty;

    // Open the serial port
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        //fprintf(stderr, "Error opening /dev/ttyUSB0: %s\n", strerror(errno));
        return -1;
    }

    // Get current serial port settings
    if (tcgetattr(fd, &tty) != 0) {
        //fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
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
        return -1;
    }

    //printf("Serial port /dev/ttyUSB0 opened and configured successfully.\n");
    return fd;
}

float GetPIRVpp(float *data, int size) {
    float vpp = data[160] - data[0];
    for (int i = 161; i < size; i++) {
        if (data[i] > data[160]) {
            vpp = data[i] - data[160];
        } else if (data[i] < data[0]) {
            vpp = data[160] - data[i];
        }
    }
    return vpp;
}
int findNearestNeighbourIndex(float value, float *x, int len) {
    float dist;
    int idx;
    int i;
    idx = -1;
    dist = FLT_MAX;
    for (i = 0; i < len; i++) {
        float newDist = value - x[i];
        if (newDist >= 0 && newDist < dist) {
            dist = newDist;
            idx = i;
        }
    }
    return idx;
}

void interp1(float *x, int x_tam, float *y, float *xx, int xx_tam, float *yy, int *yy_size) {
    float dx, dy;
    float *slope, *intercept;
    int i, indiceEnVector;
    slope = (float *)calloc(x_tam, sizeof(float));
    intercept = (float *)calloc(x_tam, sizeof(float));
    for (i = 0; i < x_tam; i++) {
        if (i < x_tam - 1) {
            dx = x[i + 1] - x[i];
            dy = y[i + 1] - y[i];
            slope[i] = dy / dx;
            intercept[i] = y[i] - x[i] * slope[i];
        } else {
            slope[i] = slope[i - 1];
            intercept[i] = intercept[i - 1];
        }
    }
    for (i = 0; i < xx_tam; i++) {
        indiceEnVector = findNearestNeighbourIndex(xx[i], x, x_tam);
        if (indiceEnVector != -1) {
            yy[*yy_size] = slope[indiceEnVector] * xx[i] + intercept[indiceEnVector];
            (*yy_size)++;
        } else {
            yy[*yy_size] = FLT_MAX;
            (*yy_size)++;
        }
    }
    free(slope);
    free(intercept);
}
void consolidate(time_t *xData, int *xData_size, float *zData, int *zData_size, float *bData, int *bData_size, float *cData, int *cData_size) {
    time_t *xDataloc = (time_t *)malloc(*xData_size * sizeof(time_t));
    float *zDataloc = (float *)malloc(*zData_size * sizeof(float));
    float *bDataloc = (float *)malloc(*bData_size * sizeof(float));
    float *cDataloc = (float *)malloc(*cData_size * sizeof(float));

    memcpy(xDataloc, xData, *xData_size * sizeof(time_t));
    memcpy(zDataloc, zData, *zData_size * sizeof(float));
    memcpy(bDataloc, bData, *bData_size * sizeof(float));
    memcpy(cDataloc, cData, *cData_size * sizeof(float));

    int size = *xData_size;
    int firstrun = 0;
    int ctr = 0;
    float sum2, sum4, sum5;
    time_t ts;

    *xData_size = 0;
    *zData_size = 0;
    *bData_size = 0;
    *cData_size = 0;

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
                xData[*xData_size++] = ts;
                zData[*zData_size++] = sum2 / ctr;
                if ((sum4 / ctr) > 0) {
                    bData[*bData_size++] = 1.0;
                } else {
                    bData[*bData_size++] = 0.0;
                }
                if ((sum5 / ctr) > 0) {
                    cData[*cData_size++] = 1.0;
                } else {
                    cData[*cData_size++] = 0.0;
                }
            }
        }
    }

    free(xDataloc);
    free(zDataloc);
    free(bDataloc);
    free(cDataloc);
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

void normalize_arr(float *arr, int n, float min_val, float max_val, float *arr_norm) {
    for (int i = 0; i < n; i++) {
        arr_norm[i] = (arr[i] - min_val) / (max_val - min_val);
    }
}

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
///////////////
void window_finder(int drv, int drv_p, int *up_flag, int *down_flag, int *ctr, int *trig2, int *dur){
    if (drv - drv_p >= 0.5) {
        if (*up_flag == 0) {
            *up_flag = 1;
        }
        if (*up_flag == 1) {
            *ctr++;
        }
    } else {
        *down_flag = 1;
        if (*down_flag == 1) {
            if (*ctr >= OBS_DUR_SH) {
                *trig2 = 1;
                *dur = OBS_DUR_SH;
                *ctr = 0;
                *up_flag = 0;
            }
        }
    }
}
void observation_processor_rt(float ts_c, int drv_c, float adc1_c, int pir_c, float *buf_ts, int *buf_ts_size, int *buf_drv, int *buf_drv_size, float *buf_adc1, int *buf_adc1_size, int *buf_pir, int *buf_pir_size, int *up, int *down, int *ctr, int *shutter_o, float *obs_rt, int *obs_pr){
    *obs_pr = 0;
	int trig2 = 0;
    int dur = -1;
		
    if (*buf_ts_size > 0) {
        window_finder(drv_c, buf_drv[*buf_drv_size - 1], up, down, ctr, &trig2, &dur);
    }

    if (drv_c > 0) {
        *shutter_o = 1;
    }

    if (*shutter_o == 1) {
        buf_ts[*buf_ts_size] = ts_c;
        (*buf_ts_size)++;

        buf_adc1[*buf_adc1_size] = adc1_c;
        (*buf_adc1_size)++;

        buf_pir[*buf_pir_size] = pir_c;
        (*buf_pir_size)++;

        buf_drv[*buf_drv_size] = drv_c;
        (*buf_drv_size)++;
    }

    if (trig2 == 1) {
        trig2 = 0;
        *shutter_o = 0;
        int ST = 0;
        float wnd_ts[OBS_DUR];
        float wnd_adc1[OBS_DUR];
        float wnd_pir[OBS_DUR];
        float wnd_drv[OBS_DUR];

        for (int i = ST; i < *buf_ts_size; i++) {
            wnd_ts[i - ST] = buf_ts[i];
        }
        for (int i = ST; i < *buf_adc1_size; i++) {
            wnd_adc1[i - ST] = buf_adc1[i];
        }
        for (int i = ST; i < *buf_pir_size; i++) {
            wnd_pir[i - ST] = buf_pir[i];
        }
        for (int i = ST; i < *buf_drv_size; i++) {
            wnd_drv[i - ST] = buf_drv[i];
        }

        float vmax1, vmin1, vmean1, vstd1, hpu1, hpd1;
        get_nrfeat_window(*buf_ts_size - ST, wnd_ts, wnd_adc1, wnd_pir, wnd_drv, &vmax1, &vmin1, &vmean1, &vstd1, &hpu1, &hpd1);

        *buf_ts_size = 0;
        *buf_adc1_size = 0;
        *buf_pir_size = 0;
        *buf_drv_size = 0;
		
		if (FEATS == 12456) {
            obs_rt[0] = vmax1;
            obs_rt[1] = vmin1;
            obs_rt[2] = vstd1;
            obs_rt[3] = hpu1;
            obs_rt[4] = hpd1;
			*obs_pr = 5;
        } else if (FEATS == 2456) {
            obs_rt[0] = vmin1;
            obs_rt[1] = vstd1;
            obs_rt[2] = hpu1;
            obs_rt[3] = hpd1;
			*obs_pr = 4;
        } else if (FEATS == 1245) {
            obs_rt[0] = vmax1;
            obs_rt[1] = vmin1;
            obs_rt[2] = vstd1;
            obs_rt[3] = hpu1;
			*obs_pr = 4;
        } else if (FEATS == 124) {
            obs_rt[0] = vmax1;
            obs_rt[1] = vmin1;
            obs_rt[2] = vstd1;
			*obs_pr = 3;
        } else if (FEATS == 145) {
            obs_rt[0] = vmax1;
            obs_rt[1] = vstd1;
            obs_rt[2] = hpu1;
			*obs_pr = 3;
        } else if (FEATS == 146) {
            obs_rt[0] = vmax1;
            obs_rt[1] = vstd1;
            obs_rt[2] = hpd1;
			*obs_pr = 3;
        } else if (FEATS == 235) {
            obs_rt[0] = vmin1;
            obs_rt[1] = vmean1;
            obs_rt[2] = hpu1;
			*obs_pr = 3;
        } else if (FEATS == 245) {
            obs_rt[0] = vmin1;
            obs_rt[1] = vstd1;
            obs_rt[2] = hpu1;
			*obs_pr = 3;
        } else if (FEATS == 345) {
            obs_rt[0] = vmean1;
            obs_rt[1] = vstd1;
            obs_rt[2] = hpu1;
			*obs_pr = 3;
        } else if (FEATS == 346) {
            obs_rt[0] = vmean1;
            obs_rt[1] = vstd1;
            obs_rt[2] = hpd1;
			*obs_pr = 3;
        }
    }
}

float cosine_distance(float *p1, float *p2, int n) {
    float dot_product = 0.0;
    float norm_p1 = 0.0;
    float norm_p2 = 0.0;

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

float euclidean_distance(float *p1, float *p2, int n) {
    float sum_squares = 0.0;

    for (int i = 0; i < n; i++) {
        sum_squares += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    }

    return sqrt(sum_squares);
}

int classify_knn(float **training_data, float *unknown, int k, int num_features, int num_training_samples, char **labels, char *dist_type) {
    float *distances = (float *)malloc(num_training_samples * sizeof(float));
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
        int min_idx = i;
        float min_dist = distances[i];
        for (int j = i + 1; j < num_training_samples; j++) {
            if (distances[j] < min_dist) {
                min_idx = j;
                min_dist = distances[j];
            }
        }
        indices[i] = min_idx;
        distances[min_idx] = distances[i];
        distances[i] = min_dist;
    }

    // Count the votes for the k nearest neighbors
    for (int i = 0; i < k; i++) {
        votes[atoi(labels[indices[i]])]++;
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
int main() {
    float pair_data[OBS_DUR] = {0};
    int drv_c, head1 = 0, head2 = 0, head3 = 0;
    float drv[OBS_DUR] = {0}, pir[OBS_DUR] = {0};
    float adc1_r[OBS_DUR] = {0};
    time_t ts[OBS_DUR] = {0};
    int pair_size=0, drv_size = 0, pir_size = 0, adc1_r_size = 0, ts_size = 0;
    time_t ts_st = 0, ts_end = 0;
    int obs = 0;
    int detectR = 0; //0="Unoccupied(PIR)", 1="Occupied(PIR)"
    time_t last_sec = 1609459200;

    //HANDLE fd = init_serial_win();
    int fd = init_serial_unix();
    while (obs < 1) {
        unsigned char byte;
        //ReadFile(fd, &byte, 1, NULL, NULL);
        read(fd, &byte, 1);
        int data = (int)byte;  // convert to integer
        head1 = head2;
        head2 = head3;
        head3 = data;
        
		if (round(ts_end - ts_st) != last_sec) {
            printf("%ld ", round(ts_end - ts_st));
        }
        last_sec = round(ts_end - ts_st);
		
		int slpr_op=0;
        if ((ts_end - ts_st) >= OBS_DUR) {
            obs++;
            for (int i = 5; i < ts_size; i++) {
                adc1_r[i - 5] = adc1_r[i];
                drv[i - 5] = drv[i];
                pir[i - 5] = pir[i];
                ts[i - 5] = ts[i];
            }

            for (int i = 0; i < adc1_r_size; i++) {
                adc1_r[i]=adc1_r[i]/50.0;
            }

            // Detection algorithm
            int act_pir = 0;
            for (int i = 0; i < pir_size; i++) {
                act_pir = act_pir+pir[i];
            }

            if (act_pir > PIR_SENST * pir_size) {
                detectR = 1;
                for (int i = 0; i < pir_size; i++) {
                    pir[i] = 1;
                }
            } else {
                for (int i = 0; i < pir_size; i++) {
                    pir[i] = 0;
                }
            }

            consolidate(ts, &ts_size, adc1_r, &adc1_r_size, pir, &pir_size, drv, &drv_size);
			
			if (detectR != 1) {
				int up = 1, down = 1, ctr = 0, shutter_o = 0;
				float buf_ts[OBS_DUR] = {0};
				float buf_adc1[OBS_DUR] = {0};
				int buf_pir[OBS_DUR] = {0};
				int buf_drv[OBS_DUR] = {0};
				int buf_ts_size = 0, buf_adc1_size = 0, buf_pir_size = 0, buf_drv_size = 0;
				int obs_pr=0;
				for (int i = 0; i < drv_size; i++) {
					float obs_rt[5] = {0};
					observation_processor_rt(ts[i], drv[i], adc1_r[i], pir[i], buf_ts, &buf_ts_size, buf_drv, &buf_drv_size, buf_adc1, &buf_adc1_size, buf_pir, &buf_pir_size, &up, &down, &ctr, &shutter_o, obs_rt, &obs_pr);
					
					if (obs_pr > 0)
					{
						//slpr_op=learn_infer(obs_rt,obs_pr);
						slpr_op=1;
						break; //TBD
					}
					
				}
			}


        }
		
        if (detectR == 1) {
            printf("IO_pinPIR,GPIO.HIGH\n");
        } else {
            printf("IO_pinPIR,GPIO.LOW\n");
        }
		
		if (slpr_op == 1) {
            printf("IO_pinPIR,GPIO.HIGH\n");
        } else {
            printf("IO_pinPIR,GPIO.LOW\n");
        }
		
		// Reset variables
        drv_size = 0;
        pir_size = 0;
        adc1_r_size = 0;
        ts_size = 0;
        ts_st = 0;
        ts_end = 0;
		head1 = 0;
        head2 = 0;
        head3 = 0;
		slpr_op = 0;
		detectR = 0;
		////////////////////////////////////////////////
        if (head2 == 13 && head3 == 10) { //carriage return followed by line feed
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
            struct tm *ts_gm = gmtime(&dt_c);
            // Get the Unix timestamp from the UTC time
            time_t ts_c = mktime(ts_gm);
            printf("UTC timestamp: %ld\n", ts_c);

            if (ts_st == 0) {
                ts_st = ts_c;
            }
            ts_end = ts_c;

            ts[ts_size++] = ts_c;
            if (pair_data[1] > 0) {
                drv_c = 1;
            } else {
                drv_c = 0;
            }

            drv[drv_size++] = drv_c;
            adc1_r[adc1_r_size++] = (pair_data[3] * 256 + pair_data[4]);
            pir[pir_size++] = pair_data[6];

            pair_data[0] = 0;
			pair_size = 0;
        }

        pair_data[pair_size++]=data;
    }

    // Close serial port
    //CloseHandle(fd);
    close(fd);

    return 0;
}

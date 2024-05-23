// read_preprocess.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define PIR_SENST 0.4 //less is more sensitive digital PIR
#define DIST "EUC"
#define OBS_DUR 20
#define OBS_DUR_SH 9 //4
#define TARGET_CLASS 0 //0,1,-1(RT MODE)
#define IGNORE1ST 1
#define FLT_MAX 3.402823466e+38F// max value
#define MAX_DATASET_SIZE 200
#define MAX_FEATURES 6
#define SAVE_FILE "2024-05-19-21-20" //#define SAVE_FILE "2024-05-19-21-20_uocc.txt"

typedef struct {
	float obs[MAX_FEATURES];
	int label;
} DataPoint;
DataPoint dataset[MAX_DATASET_SIZE];
int dataset_size = 0;
int feats[MAX_FEATURES] = { 0,0,1,1,1,0 };

float GetPIRVpp(float *data, int size) {
	float vpp = data[160] - data[0];
	for (int i = 161; i < size; i++) {
		if (data[i] > data[160]) {
			vpp = data[i] - data[160];
		}
		else if (data[i] < data[0]) {
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
	float slope, intercept;
	int i;
	for (i = 0; i < x_tam; i++) {
		if (i < x_tam - 1) {
			dx = x[i + 1] - x[i];
			dy = y[i + 1] - y[i];
			slope = dy / dx;
			intercept = y[i] - x[i] * slope;
		}
		for (int j = 0; j < xx_tam; j++) {
			if (xx[j] >= x[i] && (i == x_tam - 1 || xx[j] < x[i + 1])) {
				yy[*yy_size] = slope * xx[j] + intercept;
				(*yy_size)++;
			}
		}
	}
}

void consolidate(time_t *xData, int *xData_size, float *zData, int *zData_size, float *bData, int *bData_size, float *cData, int *cData_size) {
	int size = *xData_size;
	int ctr = 1;
	float sum2 = zData[0], sum4 = bData[0], sum5 = cData[0];
	time_t ts = xData[0];

	*xData_size = 0;
	*zData_size = 0;
	*bData_size = 0;
	*cData_size = 0;

	for (int i = 1; i < size; i++) {
		if (xData[i] == ts) {
			sum2 += zData[i];
			sum4 += bData[i];
			sum5 += cData[i];
			ctr++;
		}
		else {
			xData[*xData_size] = ts;
			(*xData_size)++;
			zData[*zData_size] = sum2 / ctr;
			(*zData_size)++;
			bData[*bData_size] = ((sum4 / ctr) > 0) ? 1.0 : 0.0;
			(*bData_size)++;
			cData[*cData_size] = ((sum5 / ctr) > 0) ? 1.0 : 0.0;
			(*cData_size)++;

			ts = xData[i];
			sum2 = zData[i];
			sum4 = bData[i];
			sum5 = cData[i];
			ctr = 1;
		}
	}

	// Process the last timestamp
	xData[*xData_size] = ts;
	(*xData_size)++;
	zData[*zData_size] = sum2 / ctr;
	(*zData_size)++;
	bData[*bData_size] = (sum4 / ctr) > 0 ? 1.0 : 0.0;
	(*bData_size)++;
	cData[*cData_size] = (sum5 / ctr) > 0 ? 1.0 : 0.0;
	(*cData_size)++;
}

void interp_linear(time_t *x, float *y, int x_len, time_t *x_new, float *y_new, int x_new_len) {
	int i, j;
	for (i = 0; i < x_new_len; i++) {
		if (x_new[i] <= x[0]) {
			y_new[i] = y[0];
		}
		else if (x_new[i] >= x[x_len - 1]) {
			y_new[i] = y[x_len - 1];
		}
		else {
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

float mean_arr(float *arr, int len) {
	float sum = 0.0;
	for (int i = 0; i < len; i++) {
		sum += arr[i];
	}
	return sum / len;
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
	float range = max_val - min_val;
	for (int i = 0; i < n; i++) {
		arr_norm[i] = (arr[i] - min_val) / range;
	}
}

void get_nrfeat_window(int wnd_ts_len, time_t *wnd_ts, float *wnd_adc1, float *wnd_pir, float *wnd_drv,
	float *vmax1, float *vmin1, float *vmean1, float *vstd1, float *hpu1, float *hpd1) {
	int x_tam = wnd_ts_len;
	int xx_tam = wnd_ts[wnd_ts_len - 1] - wnd_ts[0] + 1;

	time_t xts[OBS_DUR];
	float yadc1[OBS_DUR];
	float ypir[OBS_DUR];
	float ydrv[OBS_DUR];

	// Set up data points for interpolation
	for (int i = 0; i < xx_tam; i++) {
		xts[i] = wnd_ts[0] + i;
	}

	// Interpolate
	int yadc1_size = 0, ypir_size = 0, ydrv_size = 0;
	interp_linear(wnd_ts, wnd_adc1, x_tam, xts, yadc1, xx_tam);
	interp_linear(wnd_ts, wnd_pir, x_tam, xts, ypir, xx_tam);
	interp_linear(wnd_ts, wnd_drv, x_tam, xts, ydrv, xx_tam);

	*vmax1 = max_arr(yadc1, xx_tam);
	*vmin1 = min_arr(yadc1, xx_tam);
	*vmean1 = *vmax1 - *vmin1;// mean_arr(yadc1, xx_tam);
	*vstd1 = std_dev(yadc1, xx_tam);
	*hpu1 = 0;
	*hpd1 = 0;

	int flagup = 0;
	int flagdn = 0;
	float subwu[OBS_DUR], subwd[OBS_DUR];
	int subwu_len = 0, subwd_len = 0, uptrend_count = 0, dntrend_count = 0;
	float submin = 0, submax = 0;

	for (int i = 0; i < xx_tam - 1; i++) {
		if (yadc1[i + 1] < yadc1[i]) {
			if (uptrend_count >= 2) {
				flagup = 1;
				flagdn = 0;
				uptrend_count = 0;
			}
			else {
				dntrend_count++;
				uptrend_count = 0;
				subwu_len = 0;
				subwd[subwd_len++] = yadc1[i];
			}
		}

		if (yadc1[i + 1] > yadc1[i]) {
			if (dntrend_count >= 2) {
				flagdn = 1;
				flagup = 0;
				dntrend_count = 0;
			}
			else {
				uptrend_count++;
				dntrend_count = 0;
				subwd_len = 0;
				subwu[subwu_len++] = yadc1[i];
			}
		}

		if (flagup == 1) {
			if (subwu_len > 1) {
				float subwu_min = min_arr(subwu, subwu_len);
				float subwu_max = max_arr(subwu, subwu_len);
				float subwu_norm[OBS_DUR];
				normalize_arr(subwu, subwu_len, subwu_min, subwu_max, subwu_norm);
				submax = max_arr(subwu_norm, subwu_len);
				submin = min_arr(subwu_norm, subwu_len);

				float diff = submax - subwu_norm[0];
				*hpu1 = (*hpu1 + diff) / 2.0;
			}
			subwu_len = 0;
			flagup = 0;
		}

		if (flagdn == 1) {
			if (subwd_len > 1) {
				float subwd_min = min_arr(subwd, subwd_len);
				float subwd_max = max_arr(subwd, subwd_len);
				float subwd_norm[OBS_DUR];
				normalize_arr(subwd, subwd_len, subwd_min, subwd_max, subwd_norm);
				submax = max_arr(subwd_norm, subwd_len);
				submin = min_arr(subwd_norm, subwd_len);

				float diff = subwd_norm[0] - submin;
				*hpd1 = (*hpd1 + diff) / 2.0;
			}
			subwd_len = 0;
			flagdn = 0;
		}
	}
}

void window_finder(int drv, int drv_p, int *up_flag, int *down_flag, int *ctr, int *trig2, int *dur) {
	if (drv - drv_p >= 0.5) {
		if (*up_flag == 0) {
			*up_flag = 1;
		}
		if (*up_flag == 1) {
			(*ctr)++;
		}
	}
	else {
		*down_flag = 1;
	}

	if (*down_flag == 1) {
		if (*ctr >= OBS_DUR_SH) {
			*trig2 = 1;
			*dur = OBS_DUR_SH;
			*ctr = 0;
			*up_flag = 0;
		}
	}
}
void observation_processor_calib(time_t ts_c, int drv_c, float adc1_c, int pir_c, time_t *buf_ts, int *buf_ts_size, int *buf_drv, int *buf_drv_size, float *buf_adc1, int *buf_adc1_size, int *buf_pir, int *buf_pir_size, int *up, int *down, int *ctr, int *shutter_o, float *obs_rt, int *obs_pr, int label) {
	*obs_pr = 0;
	int trig = 0;
	int dur = -1;
	printf("observation_processor_calib buf_ts_size %d\n", *buf_ts_size);
	printf("observation_processor_calib drv_c: %d, drv_p: %d\n", drv_c, buf_drv[(*buf_drv_size)]);

	if (*buf_ts_size > 0) {
		window_finder(drv_c, buf_drv[(*buf_drv_size)], up, down, ctr, &trig, &dur);
	}
	printf("observation_processor_calib after up: %d, down: %d, ctr: %d, trig: %d, dur: %d\n", *up, *down, *ctr, trig, dur);

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

	if (trig == 1) {
		trig = 0;
		*shutter_o = 0;
		int ST = 0;
		time_t wnd_ts[OBS_DUR];
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

		for (int i = 0; i < *buf_adc1_size; i++) {
			printf("wnd_ts[i] %ld, wnd_drv[i] %f,wnd_adc1[i] %f, wnd_pir[i] %f\n", wnd_ts[i], wnd_drv[i], wnd_adc1[i], wnd_pir[i]);
		}
		float vmax1, vmin1, vmean1, vstd1, hpu1, hpd1;
		get_nrfeat_window(*buf_ts_size - ST, wnd_ts, wnd_adc1, wnd_pir, wnd_drv, &vmax1, &vmin1, &vmean1, &vstd1, &hpu1, &hpd1);

		char filename[100];
		sprintf(filename, "%s_%s.txt", SAVE_FILE, (label == 1) ? "occ" : "uocc");
		printf("saving feature in file: %s\n", filename);
		FILE *file = fopen(filename, "a");
		if (file != NULL) {
			fprintf(file, "%ld,%f,%f,%f,%f,%f,%f\n", wnd_ts[*buf_ts_size - ST - 1], vmax1, vmin1, vmean1, vstd1, hpu1, hpd1);
			fclose(file);
		}
		else
		{
			printf("saving feature file failed at %s", filename);
		}

		*buf_ts_size = 0;
		*buf_adc1_size = 0;
		*buf_pir_size = 0;
		*buf_drv_size = 0;
	}
}
void observation_processor_rt(time_t ts_c, int drv_c, float adc1_c, int pir_c, time_t *buf_ts, int *buf_ts_size, int *buf_drv, int *buf_drv_size, float *buf_adc1, int *buf_adc1_size, int *buf_pir, int *buf_pir_size, int *up, int *down, int *ctr, int *shutter_o, float *obs_rt, int *obs_pr) {
	*obs_pr = 0;
	int trig2 = 0;
	int dur = -1;
	printf("observation_processor_rt buf_ts_size %d\n", *buf_ts_size);
	printf("observation_processor_rt drv_c: %d, drv_p: %d\n", drv_c, buf_drv[(*buf_drv_size)]);

	if (*buf_ts_size > 0) {
		window_finder(drv_c, buf_drv[(*buf_drv_size)], up, down, ctr, &trig2, &dur);
	}
	printf("observation_processor_rt after up: %d, down: %d, ctr: %d, trig2: %d, dur: %d\n", *up, *down, *ctr, trig2, dur);

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
		time_t wnd_ts[OBS_DUR];
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

		obs_rt[0] = vmax1;
		obs_rt[1] = vmin1;
		obs_rt[2] = vmean1;
		obs_rt[3] = vstd1;
		obs_rt[4] = hpu1;
		obs_rt[5] = hpd1;
		*obs_pr = 6;
	}
}

float cosine_distance(float *p1, float *p2, int n) {
	float dot_product = 0.0;
	float norm_p1 = 0.0;
	float norm_p2 = 0.0;

	for (int i = 0; i < n; i++) {
		if (feats[i] > 0) {
			dot_product += p1[i] * p2[i];
			norm_p1 += p1[i] * p1[i];
			norm_p2 += p2[i] * p2[i];
		}
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
		if (feats[i] > 0) {
			sum_squares += (p1[i] - p2[i]) * (p1[i] - p2[i]);
		}
	}

	return sqrt(sum_squares);
}

int classify_knn(DataPoint *dataset, float *unknown, int k, int num_features, int num_training_samples, char *dist_type) {
	float distances[MAX_DATASET_SIZE];
	int indices[MAX_DATASET_SIZE];
	int votes[2] = { 0 }; // Assuming binary classification (0 or 1)
	int euc = 0;

	if (strcmp(dist_type, "EUC") == 0)
		euc = 1;
	// Calculate distances between unknown sample and training samples
	for (int i = 0; i < num_training_samples; i++) {
		if (euc == 1) {
			distances[i] = euclidean_distance(dataset[i].obs, unknown, num_features);
		}
		else {
			distances[i] = cosine_distance(dataset[i].obs, unknown, num_features);
		}
	}

	printf("num_training_samples %d\n", num_training_samples);
	for (int i = 0; i < num_features; i++) {
		printf("unknown[i] %f\n", unknown[i]);
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
		votes[dataset[indices[i]].label]++;
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

	printf("predicted_label %d\n", predicted_label);
	return predicted_label;
}

void get_obs_rt2(float *data, int row, int num_features, float *obs) {
	for (int i = 0; i < num_features; i++) {
		obs[i] = data[row * num_features + i];
	}
}

int learn_infer(float *obsi, int num_features) {
	int K = 3;
	char dist[4] = "EUC";

	float so[MAX_DATASET_SIZE][MAX_FEATURES];
	float uo[MAX_DATASET_SIZE][MAX_FEATURES];
	int predicted_label = -1;

	FILE *occ_file = fopen("2024-02-28-21-20_occ.txt", "r");
	FILE *uocc_file = fopen("2024-02-28-21-20_uocc.txt", "r");

	if (occ_file == NULL || uocc_file == NULL) {
		printf("Invalid calibration profile. Please calibrate or choose profile\n");
		return predicted_label;
	}

	float occ_data[MAX_DATASET_SIZE * MAX_FEATURES];
	float uocc_data[MAX_DATASET_SIZE * MAX_FEATURES];
	int occ_rows = 0, uocc_rows = 0;

	char line[100];
	char *token;

	while (fgets(line, sizeof(line), occ_file)) {
		token = strtok(line, ",");
		int i = 0;
		while (token != NULL && i < num_features) {
			occ_data[occ_rows * num_features + i] = atof(token);
			token = strtok(NULL, ",");
			i++;
		}
		occ_rows++;
	}

	while (fgets(line, sizeof(line), uocc_file)) {
		token = strtok(line, ",");
		int i = 0;
		while (token != NULL && i < num_features) {
			uocc_data[uocc_rows * num_features + i] = atof(token);
			token = strtok(NULL, ",");
			i++;
		}
		uocc_rows++;
	}

	fclose(occ_file);
	fclose(uocc_file);

	dataset_size = 0; // Reset dataset_size before populating dataset

	for (int i = 0; i < occ_rows; i++) {
		float obs[MAX_FEATURES];
		get_obs_rt2(occ_data, i, num_features, obs);
		for (int j = 0; j < num_features; j++) {
			dataset[dataset_size].obs[j] = obs[j + 1];
		}
		dataset[dataset_size].label = 1;
		dataset_size++;
		memcpy(so[i], obs, num_features * sizeof(float));
	}

	for (int i = 0; i < uocc_rows; i++) {
		float obs[MAX_FEATURES];
		get_obs_rt2(uocc_data, i, num_features, obs);
		for (int j = 0; j < num_features; j++) {
			dataset[dataset_size].obs[j] = obs[j + 1];
		}
		dataset[dataset_size].label = 0;
		dataset_size++;
		memcpy(uo[i], obs, num_features * sizeof(float));
	}

	// KNN Classification
	predicted_label = classify_knn(dataset, obsi, K, num_features, dataset_size, dist);

	return predicted_label;
}

void waitFor(unsigned int secs) {
	unsigned int retTime = time(0) + secs;   // Get finishing time.
	while (time(0) < retTime);               // Loop until it arrives.
}
int obs_loop() {
	float pair_data[7] = { 0 };
	int drv_c, head1 = 0, head2 = 0, head3 = 0;
	float drv[OBS_DUR] = { 0 }, pir[OBS_DUR] = { 0 }, adc1_r[OBS_DUR] = { 0 };
	time_t ts[OBS_DUR] = { 0 };
	int pair_size = 0, drv_size = 0, pir_size = 0, adc1_r_size = 0, ts_size = 0;
	time_t ts_st = 0, ts_end = 0;
	int obs = 0;
	int detectR = 0; //0="Unoccupied(PIR)", 1="Occupied(PIR)"
	time_t last_sec = 1609459200;

	FILE *file = fopen("data.txt", "r"); // Open the file for reading
	if (file == NULL) {
		printf("Error opening file.\n");
		return 1;
	}

	int data;
	unsigned char byte;
	while ((byte = fgetc(file)) != EOF) {
		data = byte - 48;
		head1 = head2;
		head2 = data;

		if (data == 10) //If A is read
			break;

		if ((ts_end - ts_st) != last_sec) {
			printf("Diff: %ld\n", (ts_end - ts_st));
		}
		last_sec = ts_end - ts_st;

		int slpr_op = 0;
		if ((last_sec >= OBS_DUR) && (ts_size > IGNORE1ST + 5)) {
			obs++;
			for (int i = IGNORE1ST; i < ts_size; i++) {
				adc1_r[i - IGNORE1ST] = adc1_r[i];
				drv[i - IGNORE1ST] = drv[i];
				pir[i - IGNORE1ST] = pir[i];
				ts[i - IGNORE1ST] = ts[i];
			}

			ts_size = ts_size - IGNORE1ST;
			adc1_r_size = adc1_r_size - IGNORE1ST;
			pir_size = pir_size - IGNORE1ST;
			drv_size = drv_size - IGNORE1ST;

			for (int i = 0; i < adc1_r_size; i++) {
				adc1_r[i] = adc1_r[i] / 50.0;
			}

			// Detection algorithm
			int act_pir = 0;
			for (int i = 0; i < pir_size; i++) {
				act_pir = act_pir + pir[i];
			}

			if (act_pir > PIR_SENST * pir_size) {
				detectR = 1;
				for (int i = 0; i < pir_size; i++) {
					pir[i] = 1;
				}
			}
			else {
				for (int i = 0; i < pir_size; i++) {
					pir[i] = 0;
				}
			}
			printf("ts_size: %d, adc1_r_size: %d, pir_size: %d, drv_size: %d\n", ts_size, adc1_r_size, pir_size, drv_size);
			for (int i = 0; i < ts_size; i++) {
				printf("ts[i] %ld, drv[i] %f,adc1_r[i] %f, pir[i] %f\n", ts[i], drv[i], adc1_r[i], pir[i]);
			}
			consolidate(ts, &ts_size, adc1_r, &adc1_r_size, pir, &pir_size, drv, &drv_size);
			printf("after_ts_size: %d, adc1_r_size: %d, pir_size: %d, drv_size: %d\n", ts_size, adc1_r_size, pir_size, drv_size);
			for (int i = 0; i < ts_size; i++) {
				printf("ts[i] %ld, drv[i] %f,adc1_r[i] %f, pir[i] %f\n", ts[i], drv[i], adc1_r[i], pir[i]);
			}
			if (detectR == 0) {
				int up = 1, down = 1, ctr = 0, shutter_o = 0;
				time_t buf_ts[OBS_DUR] = { 0 };
				float buf_adc1[OBS_DUR] = { 0 };
				int buf_pir[OBS_DUR] = { 0 };
				int buf_drv[OBS_DUR] = { 0 };
				int buf_ts_size = 0, buf_adc1_size = 0, buf_pir_size = 0, buf_drv_size = 0;
				int obs_pr = 0;

				for (int i = 0; i < drv_size; i++) {
					float obs_rt[MAX_FEATURES] = { 0 };
					observation_processor_rt(ts[i], drv[i], adc1_r[i], pir[i], buf_ts, &buf_ts_size, buf_drv, &buf_drv_size, buf_adc1, &buf_adc1_size, buf_pir, &buf_pir_size, &up, &down, &ctr, &shutter_o, obs_rt, &obs_pr);
					printf("obs_pr: %d\n", obs_pr);
					if (obs_pr > 0)
					{
						slpr_op = learn_infer(obs_rt, MAX_FEATURES + 1);
						printf("SLEEPIR Output: %d\n", slpr_op);
						break;
					}

				}
			}

			if (detectR == 1) {
				printf("MOTION-HIGH\n");
			}
			else {
				printf("MOTION-LOW\n");
			}

			if (slpr_op == 1) {
				printf("STATIONARY-HIGH\n");
			}
			else {
				printf("STATIONARY-LOW\n");
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

		}
		////////////////////////////////////////////////
		if (head1 == -38) {
			// Get the current time
			time_t dt_c;
			time(&dt_c);
			// Convert to local time
			struct tm *t_c = localtime(&dt_c);

			// Convert local time to UTC
			struct tm *ts_gm = gmtime(&dt_c);
			// Get the Unix timestamp from the UTC time
			time_t ts_c = mktime(ts_gm);
			printf("UTC timestamp: %ld\n", ts_c);

			if (ts_st == 0) {
				ts_st = ts_c;
			}
			ts_end = ts_c;

			if (ts_size < OBS_DUR) {
				ts[ts_size++] = ts_c;
			}

			if (pair_data[1 - 1] > 0.0) {
				drv_c = 1;
			}
			else {
				drv_c = 0;
			}

			if (drv_size < OBS_DUR) {
				drv[drv_size++] = drv_c;
			}

			if (adc1_r_size < OBS_DUR) {
				adc1_r[adc1_r_size++] = (pair_data[3 - 1] * 10 + pair_data[4 - 1]);
			}

			if (pir_size < OBS_DUR) {
				pir[pir_size++] = pair_data[6 - 1];
			}

			pair_size = 0;
			waitFor(1);
		}
		if (pair_size > 6)
			pair_size = 0;
		pair_data[pair_size++] = data;
	}

	fclose(file);
	return 0;
}
int obs_loop_calib() {
	int turn = 0;
	while (turn <= 1) {
		if (turn > 0) {
			if (TARGET_CLASS == 0) {
				printf("Unoccupancy calibrated.Moving to occupancy calibration.\n");
			}
			else {
				printf("Occupancy calibrated.Moving to unoccupancy calibration.\n");
			}
		}
		turn = turn + 1;
		float pair_data[7] = { 0 };
		int drv_c, head1 = 0, head2 = 0, head3 = 0;
		float drv[OBS_DUR] = { 0 }, pir[OBS_DUR] = { 0 }, adc1_r[OBS_DUR] = { 0 };
		time_t ts[OBS_DUR] = { 0 };
		int pair_size = 0, drv_size = 0, pir_size = 0, adc1_r_size = 0, ts_size = 0;
		time_t ts_st = 0, ts_end = 0;
		int obs = 0;
		int detectR = 0; //0="Unoccupied(PIR)", 1="Occupied(PIR)"
		time_t last_sec = 1609459200;

		FILE *file = fopen("data.txt", "r"); // Open the file for reading
		if (file == NULL) {
			printf("Error opening file.\n");
			return 1;
		}

		int data;
		unsigned char byte;
		while ((byte = fgetc(file)) != EOF) {
			data = byte - 48;
			head1 = head2;
			head2 = data;

			if (data == 10) //If A is read
				break;

			if ((ts_end - ts_st) != last_sec) {
				printf("Diff: %ld\n", (ts_end - ts_st));
			}
			last_sec = ts_end - ts_st;

			int slpr_op = 0;
			if ((last_sec >= OBS_DUR) && (ts_size > IGNORE1ST + 5)) {
				obs++;
				for (int i = IGNORE1ST; i < ts_size; i++) {
					adc1_r[i - IGNORE1ST] = adc1_r[i];
					drv[i - IGNORE1ST] = drv[i];
					pir[i - IGNORE1ST] = pir[i];
					ts[i - IGNORE1ST] = ts[i];
				}

				ts_size = ts_size - IGNORE1ST;
				adc1_r_size = adc1_r_size - IGNORE1ST;
				pir_size = pir_size - IGNORE1ST;
				drv_size = drv_size - IGNORE1ST;

				for (int i = 0; i < adc1_r_size; i++) {
					adc1_r[i] = adc1_r[i] / 50.0;
				}

				// Detection algorithm
				int act_pir = 0;
				for (int i = 0; i < pir_size; i++) {
					act_pir = act_pir + pir[i];
				}

				if (act_pir > PIR_SENST * pir_size) {
					detectR = 1;
					for (int i = 0; i < pir_size; i++) {
						pir[i] = 1;
					}
				}
				else {
					for (int i = 0; i < pir_size; i++) {
						pir[i] = 0;
					}
				}
				printf("ts_size: %d, adc1_r_size: %d, pir_size: %d, drv_size: %d\n", ts_size, adc1_r_size, pir_size, drv_size);
				for (int i = 0; i < ts_size; i++) {
					printf("ts[i] %ld, drv[i] %f,adc1_r[i] %f, pir[i] %f\n", ts[i], drv[i], adc1_r[i], pir[i]);
				}
				consolidate(ts, &ts_size, adc1_r, &adc1_r_size, pir, &pir_size, drv, &drv_size);
				printf("after_ts_size: %d, adc1_r_size: %d, pir_size: %d, drv_size: %d\n", ts_size, adc1_r_size, pir_size, drv_size);
				for (int i = 0; i < ts_size; i++) {
					printf("ts[i] %ld, drv[i] %f,adc1_r[i] %f, pir[i] %f\n", ts[i], drv[i], adc1_r[i], pir[i]);
				}

				if (detectR == 0) {
					int up = 1, down = 1, ctr = 0, shutter_o = 0;
					time_t buf_ts[OBS_DUR] = { 0 };
					float buf_adc1[OBS_DUR] = { 0 };
					int buf_pir[OBS_DUR] = { 0 };
					int buf_drv[OBS_DUR] = { 0 };
					int buf_ts_size = 0, buf_adc1_size = 0, buf_pir_size = 0, buf_drv_size = 0;
					int obs_pr = 0;

					for (int i = 0; i < drv_size; i++) {
						float obs_rt[MAX_FEATURES] = { 0 };
						observation_processor_calib(ts[i], drv[i], adc1_r[i], pir[i], buf_ts, &buf_ts_size, buf_drv, &buf_drv_size, buf_adc1, &buf_adc1_size, buf_pir, &buf_pir_size, &up, &down, &ctr, &shutter_o, obs_rt, &obs_pr, TARGET_CLASS);
					}
				}

				if (detectR == 1) {
					printf("MOTION-HIGH\n");
				}
				else {
					printf("MOTION-LOW\n");
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

			}
			////////////////////////////////////////////////
			if (head1 == -38) {
				// Get the current time
				time_t dt_c;
				time(&dt_c);
				// Convert to local time
				struct tm *t_c = localtime(&dt_c);

				// Convert local time to UTC
				struct tm *ts_gm = gmtime(&dt_c);
				// Get the Unix timestamp from the UTC time
				time_t ts_c = mktime(ts_gm);
				printf("UTC timestamp: %ld\n", ts_c);

				if (ts_st == 0) {
					ts_st = ts_c;
				}
				ts_end = ts_c;

				if (ts_size < OBS_DUR) {
					ts[ts_size++] = ts_c;
				}

				if (pair_data[1 - 1] > 0.0) {
					drv_c = 1;
				}
				else {
					drv_c = 0;
				}

				if (drv_size < OBS_DUR) {
					drv[drv_size++] = drv_c;
				}

				if (adc1_r_size < OBS_DUR) {
					adc1_r[adc1_r_size++] = (pair_data[3 - 1] * 10 + pair_data[4 - 1]);
				}

				if (pir_size < OBS_DUR) {
					pir[pir_size++] = pair_data[6 - 1];
				}

				pair_size = 0;
				waitFor(1);
			}
			if (pair_size > 6)
				pair_size = 0;
			pair_data[pair_size++] = data;
		}
		fclose(file);
	}
	return 0;
}

int main()
{
	int error = 0;
	if (TARGET_CLASS == -1)
		error = obs_loop();
	else
		error = obs_loop_calib();
	return 0;
}

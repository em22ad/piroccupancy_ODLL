# piroccupancy_ODLL
An On-Device Lifelong Learning algorithm that provides robust stationary human occupancy utilizing a proprietary PIR signal

The RAM usage for the ODLL algorithm is evaluated as follows:

Training data matrix: vector<vector<double>> xl 

MAX_SIZE is the maximum observations dataset can contain i.e. 1000
Size: MAX_SIZE x num_features

Each double is 8 bytes. Let's assume num_features is 10 so size is 1000 * 10 * 8 = 80KB

Training labels vector: vector<int> yl

Size: MAX_SIZE
Each int is 4 bytes
So size is 1000 * 4 = 4KB

Test data matrix: vector<vector<double>> xtest

Let's assume size is 100 observations
Size is 100 * 10 * 8 = 8KB

Cluster centers: vector<vector<double>> clusterCenters

Let's assume k-means finds 10 clusters
Size is 10 * 10 * 8 = 8KB

Other variables

minDistOcc, minDistUnocc: 8 bytes each

Loop counters: Negligible size

Total RAM needed:

xl: 80KB
yl: 4KB
xtest: 8KB

clusterCenters: 8KB

Other variables: ~1KB

Total: 80 + 4 + 8 + 8 + 1 = 101KB

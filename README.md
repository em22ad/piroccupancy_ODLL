# piroccupancy_ODLL
An On-Device Lifelong Learning algorithm that provides robust stationary human occupancy utilizing a proprietary PIR signal

The RAM usage for the ODLL algorithm is evaluated as follows:

vector<vector<double>> xl - Training data matrix

Size: MAX_SIZE x num_features
Each double is 8 bytes
Let's assume num_features is 10
So size is 1000 * 10 * 8 = 80KB
vector<int> yl - Training labels vector

Size: MAX_SIZE
Each int is 4 bytes
So size is 1000 * 4 = 4KB
vector<vector<double>> xtest - Test data matrix

Let's assume size is 100 observations
Size is 100 * 10 * 8 = 8KB
vector<vector<double>> clusterCenters - Cluster centers

Let's assume k-means finds 10 clusters
Size is 10 * 10 * 8 = 8KB
Other variables

minDistOcc, minDistUnocc - 8 bytes each
Loop counters - Negligible size
Total RAM needed:

xl: 80KB
yl: 4KB
xtest: 8KB
clusterCenters: 8KB
Other variables: ~1KB
Total: 80 + 4 + 8 + 8 + 1 = 101KB
